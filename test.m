%% 清空环境
clear; clc;close all;

%% 1. 参数配置 (依据你提供的信息)
PHY_SLOT_DURATION_MS = 10; 
Ntotal = 600;
Non_control_N = 2;
Control_N = 4;
N = Non_control_N + Control_N; 
Tnoncontrol = [15, 30]; 
channel_num = 1; 
slot_num = 10; 

% 优先级与配置向量
offline_P = [3, 2, 1, 4, 5, 6];      % 假设这是按优先级排序后的流ID列表
offline_T_is = [1, 1, 3, 3, 4, 2];   % 周期倍数
offline_E = [10, 14, 45, 30, 21, 3]; % 偏移量

%% 2. 数据加载与 Mock 
T1 = 40; T2 = 30; T3 = 20; T4 = 30; 
T_base_vector = [Tnoncontrol, T1, T2, T3, T4]; % 基础周期向量 (对应 T_i)

% 尝试加载路由，如果没有文件则生成随机路由
load('Route_set_cell.mat');

if length(Route) > N
    Route_sim = Route(1:N);
else
    Route_sim = Route;
end

% 计算参数
% 真实周期 = 周期倍数 * 基础周期
currentPeriod = double(offline_T_is) .* double(T_base_vector); 
% 截止期通常等于真实周期
T_i_real = T_base_vector; 
% 偏移量
currentEgili = offline_E;
% 优先级
current_priority = offline_P;

fixed_duration = 600; 
%% 运行仿真
[sv_seq] = SimulateNetworkScheduling(Route_sim, currentPeriod, currentEgili, T_i_real, current_priority, channel_num, slot_num, fixed_duration);

%% 延迟火柴图
figure('Name', '流延迟分析', 'color', 'w', 'Position', [100, 150, 800, 600]);

rows = ceil(sqrt(N));
cols = ceil(N / rows);

for i = 1:N
    subplot(rows, cols, i);
    hold on;
    
    delays = sv_seq{i};
    num_packets = length(delays);
    
    if num_packets > 0
        % 绘制火柴图
        stem(1:num_packets, delays, 'filled', 'Color', 'b', 'LineWidth', 1.5);
        
        % 绘制截止期参考线
        yline(T_i_real(i), 'r--', 'LineWidth', 1.5, 'Label', 'Deadline');
        
        % 标记超时点
        timeout_idx = find(delays > T_i_real(i));
        if ~isempty(timeout_idx)
            plot(timeout_idx, delays(timeout_idx), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        end
        
        % 计算统计信息
        avg_delay = mean(delays(delays <= T_i_real(i)));
        loss_rate = length(timeout_idx) / num_packets * 100;
        
        title(sprintf('Flow %d (Pri:%d)\nAvg: %.1f | Loss: %.1f%%', ...
            i, find(current_priority==i), avg_delay, loss_rate));
    else
        title(sprintf('Flow %d (No Packets)', i));
    end
    
    xlabel('Packet Sequence');
    ylabel('Delay (slots)');
    grid on;
    box on;
end

sgtitle('所有数据流的端到端延迟序列');