%% 1. 定义变量并加载模型
% 逻辑单位1 slot
% 1 Slot = 10 ms 
PHY_SLOT_DURATION_MS = 10; 

% 定义仿真变量
num_iterations = 300; 
Ntotal = 240;
Non_control_N = 2;
Control_N = 4;
N = Non_control_N + Control_N; 
Tnoncontrol = [15 30]; 

% 定义全局变量
channel_num = 1;
slot_num = 10; 
unit = 1; count = 1; filed_loops = []; k_counter = 0;
En_fault_rcd = 0; Route_type = 1; 
num_rt_max = 0; period_selection_bit = 0;
counter_in_loop = 0;

% T_stb 搜索倍数
T_stb_multipliers = [2, 3, 4];

% 加载控制流物理模型参数
physical_model; 
T_i = [Tnoncontrol, T1, T2, T3, T4];

% 离线偏移 offline_E
currentEgili = [10, 14, 45, 30, 21, 3];
% 离线优先级 offline_P (越小越高)
current_priority = [3, 2, 1, 4, 5, 6];

% 离线周期倍数 T_is (用于绘图识别稳定实例)
offline_T_is = [1, 1, 3, 3, 4, 2];

% 仿真时长: 跑足够长的时间来看看规律 (例如 12秒)
During_total = 1200; % 1200 slots

% 加载路由数据
load('Route_set_cell.mat'); 
Route_hop = zeros(1, N); 
for iloop = 1:N
    if ~isempty(Route{iloop})
        Route_hop(iloop) = size(Route{iloop}, 1);
    else
        Route_hop(iloop) = 0;
    end
end
Route_hop = Route_hop(1:N);
Route_hop_noncontrol = Route_hop(1:Non_control_N);

% 加载候选矩阵
load('physical_matrices_struct.mat'); 
CM = cell(1, N); 
for iloop = (Non_control_N + 1) : N
    varname = sprintf('CM_%d_%d', iloop, T_i(iloop));
    
    if isfield(CM_data, varname)
        CM{iloop} = CM_data.(varname);
    else
        error('无法找到变量 %s。请检查数据生成脚本是否覆盖了 T=%d', varname, T_i(iloop));
    end
end

%% 3. 调度核心逻辑 (完全复刻你提供的原型代码)
% 初始化输出
sv_seq = cell(1, Ntotal);
Route_dyn = cell(1, Ntotal);
for iiu = 1:Ntotal
    sv_seq{iiu} = [];
    Route_dyn{iiu} = [];
end

% 状态变量初始化
k_lock = zeros(1, N);
delay = zeros(1, Ntotal);        
dur_countv = zeros(1, Ntotal);    
counterv = 0;                     
Superframe_success1v = zeros(1, N); 
is_first_release = true(1,N);

% 调度表
schedule_csp_loopv = zeros(channel_num, slot_num);
schedule_table_titv = zeros(channel_num, slot_num);
schedule_table_revv = zeros(channel_num, slot_num);
hop_counterv = Route_hop;         
latency_recrv = zeros(1, Ntotal);
hop_remain = zeros(1, N);         

% 变量名映射 (为了适配你的代码片段)
currentPeriod = T_i; 

fprintf('开始运行纯离线配置调度验证...\n');

% 计算总帧数
num_frames = ceil(During_total / slot_num);

% === 调度主循环 (你的代码片段开始) ===
for frame_idx = 1 : num_frames
    
    % 每个 Frame 重置调度表
    schedule_table_titv(:) = 0;
    schedule_table_revv(:) = 0;
    schedule_csp_loopv(:) = 0;
    
    for slot_idx = 1 : slot_num
        
        counterv = counterv + 1; 
        if counterv > During_total, break; end
        
        % A. 任务释放与截止期检查
        for iloop = 1:N
            % 判定是否到达释放时间
            if mod(counterv - 1 - currentEgili(iloop), currentPeriod(iloop)) == 0
                if ~is_first_release(iloop)
                    if ~Superframe_success1v(iloop)
                        actual_delay = T_i(iloop) + 1;
                        sv_seq{iloop} = [sv_seq{iloop}, actual_delay];
                    else
                        final_delay = delay(iloop);
                        sv_seq{iloop} = [sv_seq{iloop}, final_delay];
                    end
                end
                is_first_release(iloop) = false;
            
                % 2. 释放新实例
                k_lock(iloop) = 0;
                Superframe_success1v(iloop) = 0;      
                Route_dyn{iloop} = Route{iloop};      
                dur_countv(iloop) = currentPeriod(iloop); 
                hop_remain(iloop) = Route_hop(iloop); 
                hop_counterv(iloop) = 0;              
                delay(iloop) = 0;
            end
        end
        
        % B. 时间流逝更新
        for hji = 1:N
            if dur_countv(hji) > 0 && Superframe_success1v(hji) == 0 
                dur_countv(hji) = dur_countv(hji) - 1;
                
                % === [潜在 BUG 重点关注区域] ===
                eff_offset = mod(currentEgili(hji), T_i(hji));
                phy_elapsed = (currentPeriod(hji) - dur_countv(hji)) + eff_offset;
                
                % 如果计算出的流逝时间(含offset) > 周期，则 Kill
                if phy_elapsed > T_i(hji)
                    dur_countv(hji) = 0; % 超时 Kill
                end
                % =============================
            end
        end
        
        % C. 资源调度 
        active_flows = find(dur_countv(1:N) > 0 & Superframe_success1v(1:N) == 0);
        
        if ~isempty(active_flows)
            % 这里的 current_priority 就是 offline_P (固定值)
            priority_dv = intersect(current_priority, active_flows, 'stable');
            
            for idx = 1:length(priority_dv)
                flow_id = priority_dv(idx);
                route = Route_dyn{flow_id};
                
                if hop_remain(flow_id) > 0
                    current_hop_idx = hop_counterv(flow_id) + 1;
                    src_node = route(current_hop_idx, 1);
                    dst_node = route(current_hop_idx, 2);
                    
                    conflict = 0;
                    for ch = 1:channel_num
                        if schedule_table_titv(ch, slot_idx) == 0
                            node_busy = 0;
                            for other_ch = 1:channel_num
                                if other_ch ~= ch
                                    if schedule_table_titv(other_ch, slot_idx) == src_node || ...
                                       schedule_table_revv(other_ch, slot_idx) == src_node || ...
                                       schedule_table_titv(other_ch, slot_idx) == dst_node || ...
                                       schedule_table_revv(other_ch, slot_idx) == dst_node
                                        node_busy = 1; break;
                                    end
                                end
                            end
                            
                            if ~node_busy
                                schedule_table_titv(ch, slot_idx) = src_node;
                                schedule_table_revv(ch, slot_idx) = dst_node;
                                schedule_csp_loopv(ch, slot_idx) = flow_id;
                                hop_counterv(flow_id) = hop_counterv(flow_id) + 1;
                                hop_remain(flow_id) = hop_remain(flow_id) - 1;
                                
                                if hop_remain(flow_id) == 0
                                    Superframe_success1v(flow_id) = 1;
                                    % === [潜在 BUG 重点关注区域] ===
                                    % 时延计算包含 offset
                                    delay(flow_id) = currentPeriod(flow_id) - dur_countv(flow_id) + mod(currentEgili(flow_id),T_i(flow_id));
                                    dur_countv(flow_id) = 0;
                                end
                                break; 
                            end
                        end
                    end
                end
            end
        end
    end
end
% === 调度循环结束 ===

%% 4. 绘图代码 (复制自你的绘图部分)
View_Start_Time_Sec = 0.0;
View_End_Time_Sec   = 12;
View_Start_Slot = round(View_Start_Time_Sec / Slot_Duration_Sec);
View_End_Slot   = round(View_End_Time_Sec   / Slot_Duration_Sec);

figure('Name', '纯离线配置验证-逐包时延', 'Color', 'w', 'Position', [100, 100, 1000, 600]);

% 只画 Control Flows (3~6)
Control_Start_Idx = 3; 
Control_End_Idx = N;

for i = Control_Start_Idx : Control_End_Idx
    % 准备数据
    all_delays = sv_seq{i}; % 注意：这里换成了本脚本生成的 sv_seq
    if isempty(all_delays), continue; end
    
    packet_indices = 0 : length(all_delays) - 1;
    send_times = packet_indices * T_i(i);
    
    % 筛选
    valid_idx = find(send_times >= View_Start_Slot & send_times <= View_End_Slot);
    
    window_send_times = send_times(valid_idx);
    window_delays = all_delays(valid_idx);
    window_global_idxs = packet_indices(valid_idx);
    
    % 使用修正后的判断逻辑来染色，看是不是因为逻辑问题
    target_phase = mod(floor(currentEgili(i) / T_i(i)), offline_T_is(i));
    istable_mask = (mod(window_global_idxs, offline_T_is(i)) == target_phase);
    
    x_axis_sec = window_send_times * Slot_Duration_Sec;
    
    % 绘制子图
    idx_plot = i - 2; % 调整 plot 索引
    subplot(2, 2, idx_plot);
    
    % 绘制
    stem(x_axis_sec(~istable_mask), window_delays(~istable_mask), 'filled', 'Color','#0072BD','LineWidth', 1.5, 'MarkerSize', 4);
    hold on;
    stem(x_axis_sec(istable_mask), window_delays(istable_mask), 'filled', 'Color','r','LineWidth', 1.5, 'MarkerSize', 4);
    
    % 画 Deadline 线
    yline(T_i(i), 'r--', 'LineWidth', 1.5, 'Label', 'Deadline');
    
    grid on;
    title(['Flow ', num2str(i), ' (T=', num2str(T_i(i)), ', E=', num2str(currentEgili(i)), ')']);
    xlabel('Time (s)'); ylabel('Delay (Slots)');
    xlim([View_Start_Time_Sec, View_End_Time_Sec]); 
    ylim([0, T_i(i) + 5]); 
end
sgtitle('纯离线配置下的时延验证 (无动态性能流)');