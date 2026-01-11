%% Main_Online_Rolling_Control.m
clc; clear; close all;
%% 1. 参数设置
PHY_SLOT_DURATION_MS = 10; 
Ntotal = 120;
Non_control_N = 2;
Control_N = 4;
N = Non_control_N + Control_N; 
Tnoncontrol = [15 30]; 
channel_num = 1; slot_num = 10; 
T_stb_multipliers = [2, 3, 4];

% 1. 加载物理模型参数
physical_model; 
T_i = [Tnoncontrol, T1, T2, T3, T4];
% 2. 加载路由
load('Route_set_cell.mat'); 
Route_hop = zeros(1, N); 
for i = 1:N, if ~isempty(Route{i}), Route_hop(i) = size(Route{i}, 1); else, Route_hop(i)=0; end; end
Route_hop_noncontrol = Route_hop(1:Non_control_N);
% 3. 加载矩阵数据并从 Struct 转为 Cell
load('physical_matrices_struct.mat'); 
CM_cell_array = cell(1, N); % 创建一个空的 Cell 数组
for i = (Non_control_N + 1) : N
    % 根据流ID和基础周期，拼凑出结构体里的变量名
    varname = sprintf('CM_%d_%d', i, T_i(i));
    
    if exist('CM_data', 'var') && isfield(CM_data, varname)
        CM_cell_array{i} = CM_data.(varname);
    else
        error('无法在 CM_data 中找到变量 %s。检查生成脚本的周期设置是否包含 %d', varname, T_i(i));
    end
end

% 4. 打包环境参数
Env.N = N;
Env.Non_control_N = Non_control_N;
Env.Control_N = Control_N;
Env.T_i = T_i;
Env.T_stb_multipliers = T_stb_multipliers;
Env.Route_hop = Route_hop;
Env.Route_hop_noncontrol = Route_hop_noncontrol;
Env.Tnoncontrol = Tnoncontrol;
Env.Route = Route;
Env.Ntotal = Ntotal;
Env.channel_num = channel_num;
Env.slot_num = slot_num;
Env.CM_data = CM_cell_array; % 转换好的 Cell

K = cell(1, Env.N);K{3} = K1;K{4} = K2; K{5} = K3;K{6} = K4;

History_X = cell(1,Env.N);
History_Delay = cell(1,Env.N);
History_J_Cost = [];
History_Priority_Matrix = [];

Q_weight = eye(2);
Lambda_delay = 0;

% 载入离线最优配置
offline_P = [5, 6, 3, 2, 4, 1];
offline_T_is = [1, 1, 3, 3, 4, 2];
offline_E = [0, 21, 6, 32, 41, 36];

%% 2. 状态初始化
Total_Sim_Slots = 9600;
X_real = cell(1, N);
U_last = zeros(1, N);
Control_Idx = (Non_control_N + 1) : N;

for i = Control_Idx
    X_real{i} = [10; -5];
    History_X{i} = X_real{i};
end

%% 3. 在线滚动优化主循环
fprintf('开始在线滚动优化仿真...\n');

for t_start = 1 : Env.Ntotal : Total_Sim_Slots

    if t_start == 121 % 比如在第 2 个窗口开始时
    disp('【调试】在优化前强行注入大误差！');
    X_real{3} = [50; 0]; % 给流 3 一个巨大的位置偏差
    end
    
    P_perf_perms = perms((N + 1) : (N + Control_N));
    best_P_perf = [];
    min_J = Inf;
    
    % 遍历每一种优先级组合进行预测
    for p_idx = 1 : size(P_perf_perms, 1)
        current_P_try = P_perf_perms(p_idx, :);
        
        [sv_seq_pred] = predict_Tc_delays(t_start, offline_P, current_P_try, offline_T_is, offline_E, Env);
        
        % 预测控制性能
        current_J_total = 0;
        for i = Control_Idx
            x_temp = X_real{i};
            u_old = U_last(i);
            
            for k_step = 1 : length(sv_seq_pred{i})
                dk = sv_seq_pred{i}(k_step);
                
                if dk <= Env.T_i(i)
                    mat_idx = max(1, dk); 
                else
                    mat_idx = Env.T_i(i) + 1; 
                end
                
                A_cl = Env.CM_data{i}{mat_idx};
                z_now = [x_temp; u_old];
                z_next = A_cl * z_now;
                x_temp = z_next(1:size(X_real{i}, 1));

                current_J_total = current_J_total + Env.T_i(i) * (x_temp' * Q_weight * x_temp) + Env.Route_hop(i) * Lambda_delay * dk;
                % 在计算完 current_J_total 后
                if p_idx == 1 || p_idx == size(P_perf_perms, 1) % 只看第一个和最后一个组合
                    fprintf('组合 %d: 流3优先级 %d, 预测时延序列(前3个) = [%s], 代价 J = %.4f\n', ...
                        p_idx, ...
                        find(current_P_try == 3), ... % 假设流3是受害流
                        num2str(sv_seq_pred{3}(1:3)), ...
                        current_J_total);
                end

                u_old = K{i} * x_temp; 
            end
        end
        
        % 记录最优
        if current_J_total < min_J
            min_J = current_J_total;
            best_P_perf = current_P_try;
        end
    end
    
    History_J_Cost = [History_J_Cost, min_J];% 代价下降曲线
    History_Priority_Matrix = [History_Priority_Matrix; best_P_perf];% 最优优先级变更图

    % 执行
    [sv_seq_real] = predict_Tc_delays(t_start, offline_P, best_P_perf, offline_T_is, offline_E, Env);
    
    for i = Control_Idx
        History_Delay{i} = [History_Delay{i}, sv_seq_real{i}];
        for k_r = 1 : length(sv_seq_real{i})
            dk_r = sv_seq_real{i}(k_r);
            idx_r = min(dk_r, Env.T_i(i) + 1);
            if dk_r <= 0, idx_r = 1; end
            
            A_cl_r = Env.CM_data{i}{idx_r};
            z_real = [X_real{i}; U_last(i)];
            z_next_real = A_cl_r * z_real;
            
            % 下一步的状态取出来
            x_next_temp = z_next_real(1:size(X_real{i},1)); 
            
            % 给流在某些时间窗口内增加脉冲干扰（先预测再脉冲，可能导致下一个滚动窗口的状态下降已经被抵消了
            if (t_start == 50 * Env.Ntotal + 1 && i == (3) && k_r ==1) || (t_start == 30 * Env.Ntotal + 1 && i == (5) && k_r ==1)
                % 给第一维加 10 的偏差
                x_next_temp(1) = x_next_temp(1) + 10; 

                disp([' 流 ', num2str(i), ' 加入脉冲干扰']);
            end
            
            % 把状态赋值给 X_real
            X_real{i} = x_next_temp;
            U_last(i) = K{i} * X_real{i}; % 真实物理指令更新

            History_X{i} = [History_X{i}, X_real{i}];
        end
    end
    disp(['时间', num2str((t_start-1) * 0.01), ' s: ', num2str(best_P_perf)]);
end

%% 绘图阶段
Control_Start_Idx = Env.Non_control_N + 1;
Control_End_Idx = Env.N;
Colors = {'r', 'b', 'k', 'm', 'g', 'c'}; % 颜色库

% 图 1: 状态收敛轨迹
figure('Name', '状态收敛轨迹', 'Color', 'w', 'Position', [100, 100, 800, 600]);
for i = Control_Start_Idx : Control_End_Idx
    idx_plot = i - Env.Non_control_N;
    subplot(2, 2, idx_plot);

    % 获取该流的历史轨迹
    traj = History_X{i}'; 

    % 绘制所有状态维度
    plot(traj, 'LineWidth', 1.5); 
    grid on;

    title(['Loop ', num2str(i-2)]);
    xlabel('采样步数 k');
    ylabel('状态偏差 x');
    legend('x_1', 'x_2');
    xlim([0, length(traj)]);
end
sgtitle('State Trajectories');

% % 2: 累积误差代价
% 
% figure('Name', '累积误差代价', 'Color', 'w');
% plot(History_J_Cost, '.-', 'LineWidth', 1, 'Color', 'b', 'MarkerFaceColor', 'b');
% grid on;
% title('预测代价');
% xlabel('滚动窗口索引');
% ylabel('代价');
% ylim([0,5]);

% % 3: 时延分布直方图
% figure('Name', '时延分布直方图', 'Color', 'w', 'Position', [100, 100, 800, 600]);
% for i = Control_Start_Idx : Control_End_Idx
%     idx_plot = i - Env.Non_control_N;
%     subplot(2, 2, idx_plot);
% 
%     delays = History_Delay{i};
%     max_delay = Env.T_i(i) + 1; % Ti+1 代表丢包
% 
%     % 绘制直方图
%     histogram(delays, 'BinMethod', 'integers', 'FaceColor', Colors{idx_plot});
%     grid on;
% 
%     % 标记丢包区域
%     hold on;
%     xline(Env.T_i(i) + 0.5, 'r--', 'LineWidth', 2, 'Label', 'Deadline');
% 
%     title(['Loop ', num2str(i-2)]);
%     xlabel('时延 (Slots)');
%     ylabel('频次');
%     xlim([0, max_delay + 1]);
% end
% sgtitle('直方图');

% % 4: 优先级演变
% figure('Name', '在线优先级演变图', 'Color', 'w', 'Position', [100, 100, 800, 200]);
% 
% % History_Priority_Matrix 是 (80 x 4) 的矩阵
% Data_Plot = History_Priority_Matrix'; % 转置后变成 (4 x 80): 纵轴是流，横轴是时间
% [num_rows, num_cols] = size(Data_Plot);
% 
% imagesc(Data_Plot); 
% hold on; % 保持图像，以便在其上画线
% 
% % 绘制垂直分割线 (在每个时间窗之间)
% for x = 0.5 : 1 : num_cols + 0.5
%     xline(x, 'Color', 'w', 'LineWidth', 1); % 白色实线，宽度可调
% end
% 
% % 绘制水平分割线
% for y = 0.5 : 1 : num_rows + 0.5
%     yline(y, 'Color', 'w', 'LineWidth', 1.5);
% end
% 
% % 7红, 8橙, 9蓝, 10灰
% cmap = [
%     0.85, 0.33, 0.10; % 7: 红色 
%     0.93, 0.69, 0.13; % 8: 橙色
%     0.00, 0.45, 0.74; % 9: 蓝色
%     0.60, 0.60, 0.60  % 10: 灰色 
% ];
% colormap(gca, cmap);
% clim([6.5 10.5]); % 锁定颜色范围
% set(gca, 'YTick', 1:num_rows, 'YTickLabel', {'Loop 1', 'Loop 2', 'Loop 3', 'Loop 4'}, ...
%     'TickLength', [0 0]); % 隐藏刻度线，只留文字
% ylabel('控制回路)', 'FontSize', 10, 'FontWeight', 'bold');
% 
% xlabel('滚动优化窗口索引', 'FontSize', 10, 'FontWeight', 'bold');
% title('优先级演变', 'FontSize', 12);
% cb = colorbar;
% cb.Label.String = '优先级';
% cb.Ticks = [7 8 9 10];
% cb.TickLabels = {'7 ', '8', '9', '10 '};
% axis tight; % 紧贴数据边缘
% box on;     % 给整个图加个外框
% set(gca, 'LineWidth', 1.5); % 坐标轴框线加粗

% % 5: 特定时间窗口内的逐包时延
% View_Start_Time_Sec = 20.0;
% View_End_Time_Sec   = 40.0;
% 
% Slot_Duration_Sec = PHY_SLOT_DURATION_MS / 1000;
% View_Start_Slot = round(View_Start_Time_Sec / Slot_Duration_Sec);
% View_End_Slot   = round(View_End_Time_Sec   / Slot_Duration_Sec);
% 
% figure('Name', '特定窗口逐包时延', 'Color', 'w', 'Position', [100, 100, 1000, 600]);
% 
% Control_Start_Idx = Env.Non_control_N + 1;
% Control_End_Idx = Env.N;
% 
% for i = Control_Start_Idx : Control_End_Idx
% 
%     % 准备数据
%     all_delays = History_Delay{i}; 
%     packet_indices = 0 : length(all_delays) - 1;
%     send_times = packet_indices * Env.T_i(i);
% 
%     % 筛选
%     valid_idx = find(send_times >= View_Start_Slot & send_times <= View_End_Slot);
% 
%     if isempty(valid_idx)
%         continue; 
%     end
% 
%     window_send_times = send_times(valid_idx);
%     window_delays = all_delays(valid_idx);
% 
%     % 绘制子图
%     idx_plot = i - Env.Non_control_N;
%     subplot(2, 2, idx_plot);
% 
%     % 这里可以把 X 轴也转换成秒，让图表更易读
%     stem(window_send_times * Slot_Duration_Sec, window_delays, 'filled', 'LineWidth', 1.5, 'MarkerSize', 4);
% 
%     hold on;
%     yline(Env.T_i(i), 'r--', 'LineWidth', 1.5, 'Label', 'Deadline');
% 
%     grid on;
%     title(['Loop ', num2str(idx_plot), ' (Period = ', num2str(Env.T_i(i)*Slot_Duration_Sec), 's)']);
% 
%     % 修改 X 轴标签为
%     xlabel('仿真时间 (Seconds)'); 
%     ylabel('实际延迟 (Slots)');
% 
%     % 锁定 X 轴范围 (使用秒)
%     xlim([View_Start_Time_Sec, View_End_Time_Sec]); 
%     ylim([0, max(Env.T_i(i) * 1.5, max(window_delays) + 2)]); 
% end
% 
% sgtitle(['Packet Delays: ', num2str(View_Start_Time_Sec), 's - ', num2str(View_End_Time_Sec), 's']);