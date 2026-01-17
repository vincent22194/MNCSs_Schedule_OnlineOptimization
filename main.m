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

% 加载物理模型参数
physical_model; 
T_i = [Tnoncontrol, T1, T2, T3, T4];
% 加载路由
load('Route_set_cell.mat'); 
Route_hop = zeros(1, N); 
for i = 1:N, if ~isempty(Route{i}), Route_hop(i) = size(Route{i}, 1); else, Route_hop(i)=0; end; end
Route_hop_noncontrol = Route_hop(1:Non_control_N);
% 加载矩阵数据并从 Struct 转为 Cell
load('physical_matrices_struct.mat'); 
CM_cell_array = cell(1, N); 
for i = (Non_control_N + 1) : N
    varname = sprintf('CM_%d_%d', i, T_i(i));
    
    if exist('CM_data', 'var') && isfield(CM_data, varname)
        CM_cell_array{i} = CM_data.(varname);
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


History_Individual_J = cell(1, Env.N);
K = cell(1, Env.N);K{3} = K1;K{4} = K2; K{5} = K3;K{6} = K4;

History_X = cell(1,Env.N);
History_Delay = cell(1,Env.N);
History_J_Cost = [];
History_Priority_Matrix = [];
Global_Schedule_History = [];

Q_weight = eye(2);
Lambda_delay = 0.001;

% 载入离线最优配置
offline_P = [3, 2, 1, 4, 5, 6];
offline_T_is = [1, 1, 3, 3, 4, 2];
offline_E = [10, 14, 45, 30, 21, 3];

%% 2. 状态初始化
Total_Sim_Slots = 2400;
X_real = cell(1, N);
U_last = zeros(1, N);
Control_Idx = (Non_control_N + 1) : N;
Current_NetState = [];

for i = Control_Idx
    X_real{i} = [10; -5];
    History_X{i} = X_real{i};
end

%% 3. 在线滚动优化主循环
fprintf('开始在线滚动优化仿真...\n');

for t_start = 1 : Env.Ntotal : Total_Sim_Slots

    P_perf_perms = perms((N + 1) : (N + Control_N));
    best_P_perf = [];
    min_J = Inf;
    
    % 遍历每一种优先级组合进行预测
    for p_idx = 1 : size(P_perf_perms, 1)
        current_P_try = P_perf_perms(p_idx, :);
        
        [sv_seq_pred, ~] = predict_Tc_delays(t_start, offline_P, offline_T_is, offline_E, current_P_try, Env, Current_NetState);
        
        % 预测控制性能
        current_J_total = 0;
        for i = Control_Idx
            x_temp = X_real{i};
            u_old = U_last(i);
            
            for k_step = 1 : length(sv_seq_pred{i})
                dk = sv_seq_pred{i}(k_step);
                
                A_cl = Env.CM_data{i}{dk};
                z_now = [x_temp; u_old];
                z_next = A_cl * z_now;
                x_temp = z_next(1:size(X_real{i}, 1));

                current_J_total = current_J_total + Env.T_i(i) * (x_temp' * Q_weight * x_temp) +  Lambda_delay * dk;

                u_old = z_next(size(X_real{i}, 1) + 1); 
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
    [sv_seq_real, Next_NetState, Current_Log] = predict_Tc_delays(t_start, offline_P, offline_T_is, offline_E, best_P_perf, Env, Current_NetState);
    Current_NetState = Next_NetState;
    Global_Schedule_History = [Global_Schedule_History; Current_Log];

    for i = Control_Idx
        History_Delay{i} = [History_Delay{i}, sv_seq_real{i}];
        for k_r = 1 : length(sv_seq_real{i})
            dk_r = sv_seq_real{i}(k_r);
            idx_r = min(dk_r, Env.T_i(i) + 1);
            if dk_r <= 0, idx_r = 1; end
            
            A_cl_r = Env.CM_data{i}{idx_r};
            z_real = [X_real{i}; U_last(i)];
            z_next_real = A_cl_r * z_real;
            
            % 把状态赋值给 X_real
            X_real{i} = z_next_real(1 : size(X_real{i},1));
            U_last(i) = z_next_real(size(X_real{i},1)+1 : end);

            History_X{i} = [History_X{i}, X_real{i}];

            state_cost = Env.T_i(i) * (X_real{i}' * Q_weight * X_real{i});
            comm_penalty = Env.Route_hop(i) * Lambda_delay * dk_r;
        
            step_total_J = state_cost + comm_penalty;
            History_Individual_J{i} = [History_Individual_J{i}, step_total_J];
        end
    end
    disp(['时间', num2str((t_start-1) * 0.01,'%.2f'), 's : ', num2str(best_P_perf)]);
end

%% 绘图阶段
Colors = {'r', 'b', 'k', 'm', 'g', 'c'}; % 颜色库

% % 1: 状态收敛轨迹
% figure('Name', '状态收敛轨迹', 'Color', 'w', 'Position', [100, 100, 800, 600]);
% for i = Control_Start_Idx : Control_End_Idx
%     idx_plot = i - Env.Non_control_N;
%     subplot(2, 2, idx_plot);
% 
%     % 获取该流的历史轨迹
%     traj = History_X{i}'; 
%     k = 0:length(traj)-1;
% 
%     % 绘制所有状态维度
%     plot(k, traj, 'LineWidth', 1.5); 
%     grid on;
% 
%     title(['Loop ', num2str(i-2)]);
%     xlabel('采样步数 k');
%     ylabel('状态偏差 x');
%     legend('x_1', 'x_2');
%     xlim([0, length(traj)]);
% end
% sgtitle('State Trajectories');

% %% 1 状态收敛轨迹
% figure('Name', '状态收敛轨迹', 'Color', 'w', 'Position', [100, 100, 900, 700]);
% Control_Start_Idx = Env.Non_control_N + 1;
% Control_End_Idx = Env.N;
% 
% for i = Control_Start_Idx : Control_End_Id
%     idx_plot = i - Env.Non_control_N;
%     subplot(2, 2, idx_plot);
% 
%     % 1. 获取该流的历史轨迹
%     traj = History_X{i}'; 
%     num_steps = length(traj);
% 
%     % 2. 计算真实时间轴 (秒)
%     step_duration_sec = Env.T_i(i) * PHY_SLOT_DURATION_MS / 1000;
%     time_axis = (0 : num_steps-1) * step_duration_sec;
% 
%     % 3. 绘图
%     plot(time_axis, traj, 'LineWidth', 1.5); 
%     grid on; hold on;
% 
%     title(['Control Flow ', num2str(i), ' (T_i = ', num2str(Env.T_i(i)), ' slots)']);
%     xlabel('时间 (Seconds)');
%     ylabel('状态偏差 x');
%     legend('x_1', 'x_2', 'Location', 'northeast');
% 
%     % 设置坐标轴范围，使图像更美观
%     xlim([0, max(time_axis)]);
% end
% sgtitle('State Trajectories over Real Time');
% 
% % 2: 累积误差代价
% 
% figure('Name', '累积误差代价', 'Color', 'w');
% plot(History_J_Cost, '.-', 'LineWidth', 1, 'Color', 'b', 'MarkerFaceColor', 'b');
% grid on;
% title('预测代价');
% xlabel('时间');
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
% Data_Plot = History_Priority_Matrix'; 
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
% clim([6.5 10.5]);
% set(gca, 'YTick', 1:num_rows, 'YTickLabel', {'Loop 1', 'Loop 2', 'Loop 3', 'Loop 4'}, ...
%     'TickLength', [0 0]); 
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
% 
%% 5: 特定时间窗口内的逐包时延
View_Start_Time_Sec = 0;
View_End_Time_Sec   = 6;

Slot_Duration_Sec = PHY_SLOT_DURATION_MS / 1000;
View_Start_Slot = round(View_Start_Time_Sec / Slot_Duration_Sec);
View_End_Slot   = round(View_End_Time_Sec   / Slot_Duration_Sec);

figure('Name', '窗口逐包时延', 'Color', 'w', 'Position', [350, 100, 800, 650]);

Control_Start_Idx = Env.Non_control_N + 1;
Control_End_Idx = Env.N;

for i = Control_Start_Idx : Control_End_Idx

    % 准备数据
    all_delays = History_Delay{i}; 
    packet_indices = 0 : length(all_delays) - 1;
    send_times = packet_indices * Env.T_i(i);

    % 筛选
    valid_idx = find(send_times >= View_Start_Slot & send_times <= View_End_Slot);

    window_send_times = send_times(valid_idx);
    window_delays = all_delays(valid_idx);
    window_global_idxs = packet_indices(valid_idx);

    istable_mask = (mod(window_global_idxs, offline_T_is(i)) == floor(offline_E(i) / Env.T_i(i)));

    x_axis_sec = window_send_times * Slot_Duration_Sec;

    idx_plot = i - Env.Non_control_N;
    subplot(2, 2, idx_plot);

    stem(x_axis_sec(~istable_mask), window_delays(~istable_mask), 'filled', 'Color','#0072BD','LineWidth', 1.5, 'MarkerSize', 4);
    hold on;

    stem(x_axis_sec(istable_mask), window_delays(istable_mask), 'filled', 'Color','r','LineWidth', 1.5, 'MarkerSize', 4);

    hold on;
    yline(Env.T_i(i), 'r--', 'LineWidth', 1.5, 'Label', 'Deadline');

    grid on;
    title(['Loop ', num2str(idx_plot), ' (Period = ', num2str(Env.T_i(i)*Slot_Duration_Sec), 's)']);

    xlabel('仿真时间 (s)'); 
    ylabel('实际延迟 (Slots)');

    xlim([View_Start_Time_Sec, View_End_Time_Sec]); 
    ylim([0, max(Env.T_i(i) * 1.5, max(window_delays) + 2)]); 
end

sgtitle(['Packet Delays: ', num2str(View_Start_Time_Sec), 's - ', num2str(View_End_Time_Sec), 's']);

% 
% %% 6 绘制各流独立代价收敛曲线
% figure('Name', '代价收敛曲线', 'Color', 'w', 'Position', [350, 100, 800, 650]);
% 
% % 定义控制流索引
% C_Start = Env.Non_control_N + 1;
% C_End = Env.N;
% 
% for i = C_Start : C_End
%     subplot(2, 2, i - Env.Non_control_N);
% 
%     % 1. 准备数据
%     y_data = History_Individual_J{i};
%     if isempty(y_data), continue; end
% 
%     % 2. 计算真实物理时间轴 (单位: 秒)
%     % 采样间隔 = T_i * 10ms
%     dt = Env.T_i(i) * PHY_SLOT_DURATION_MS / 1000;
%     x_time = (1:length(y_data)) * dt;
% 
%     % 3. 绘图
%     % 使用 semilogy 可以更好地观察从脉冲高点到稳态基值的收敛过程
%     plot(x_time, y_data, 'LineWidth', 1.8, 'Color', [0.2 0.6 0.4]);
%     grid on; hold on;
% 
%     % 5. 格式化
%     title(['Flow ', num2str(i), ' Total Cost']);
%     xlabel('Real Time (s)');
%     ylabel('Cost value (Log Scale)');
% 
%     ylim([min(y_data(y_data>0))*0.8, max(y_data)*1.5]);
% end
% 
% sgtitle('收敛曲线', 'FontSize', 14);

%% 仿真结束后绘制甘特图
figure('Name', '网络调度甘特图', 'Color', 'w', 'Position', [100, 100, 1200, 100]);
hold on;

% 为每个流分配固定颜色
colors = lines(Env.N); 

for i = 1:size(Global_Schedule_History, 1)
    t_slot = Global_Schedule_History(i, 1);
    f_id = Global_Schedule_History(i, 2);
    ch_id = Global_Schedule_History(i, 5);
    h_idx = Global_Schedule_History(i, 6);
    
    % 绘制矩形：x=起始时间, y=流ID(或信道ID), width=1, height=0.8
    % 这里 Y 轴设为流 ID，可以清晰观察每个流的传输间隔
    rectangle('Position', [t_slot, 0, 1, 1], ...
              'FaceColor', colors(f_id, :), 'EdgeColor', 'none');
    % 
    % % 在矩形上标注跳数
    text(t_slot+0.1, 0.5, num2str(h_idx), 'FontSize', 8, 'Color', 'w');
end

% 装饰图表
xlabel('时间(s)');
set(gca,'YTick',[]);
title('全流程在线调度甘特图');
grid on;
xlim([1, 200]); 
