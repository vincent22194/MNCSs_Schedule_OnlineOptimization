%% --- 帕累托前沿扫描脚本 (Pareto Frontier Scanner) ---
clc; clear; close all;

% 1. 定义我们要扫描的 Lambda 范围
% 这里用对数间距取点，涵盖从小 (0.001) 到大 (10) 的范围
Lambda_List = [0.001, 0.01, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10];
num_points = length(Lambda_List);

% 初始化记录本
Pareto_Control_Cost = zeros(1, num_points); % Y轴：控制误差
Pareto_Network_Delay = zeros(1, num_points); % X轴：网络时延

fprintf('开始自动化扫描帕累托前沿，共 %d 个点...\n', num_points);

%% 2. 自动化循环
for idx = 1 : num_points
    % --- A. 设置当前的权重 ---
    Current_Lambda = Lambda_List(idx);
    fprintf('正在运行第 %d/%d 轮仿真，Lambda = %.4f ... \n', idx, num_points, Current_Lambda);
    
    % =======================================================
    % 【请在此处粘贴您原来的仿真参数初始化代码】
    % (注意：不要写 clear，否则会把外层循环变量清空！)
    % 关键修改：把原来的 Lambda_delay = ... 改成下面这句：
    Lambda_delay = Current_Lambda; 
    
    % ... (此处省略您的 physical_model, 路由加载, 变量初始化等代码) ...
    % ... (如果您觉得代码太长，建议把您的仿真核心逻辑封装成一个函数调用) ...
    
    % 为了演示，我假设您已经运行完了仿真，并且有了 History_X 和 History_Delay
    % 这里必须运行您完整的 "Main" 循环 (Phase A + Phase B)
    % =======================================================
    
    % --- 【这里是您需要把 Main 里的代码搬过来的地方】 ---
    % 提示：为了方便，您可以把您的 Main.m 去掉开头的 clear，然后保存为 Run_One_Sim.m
    % 然后在这里直接写： run('Run_One_Sim.m'); 
    % 这样最省事！
    run('main.m'); % <--- 假设您的主程序叫这个名字
    
    
    % =======================================================
    % --- B. 仿真结束，统计本轮指标 ---
    
    % 1. 计算平均控制误差 (Y轴指标)
    % 遍历所有控制流，算出所有时刻的状态范数平方和
    total_ctrl_error = 0;
    total_samples = 0;
    for i = (Env.Non_control_N + 1) : Env.N
        % 取出该流的历史轨迹
        traj = History_X{i}; 
        % 计算 x'Qx (假设 Q=eye(2))，这里简化为 norm(x)^2
        total_ctrl_error = total_ctrl_error + sum(vecnorm(traj).^2);
        total_samples = total_samples + size(traj, 2);
    end
    avg_ctrl_cost = total_ctrl_error / total_samples;
    
    % 2. 计算平均网络时延 (X轴指标)
    total_delay = 0;
    total_pkts = 0;
    for i = (Env.Non_control_N + 1) : Env.N
        delays = History_Delay{i};
        total_delay = total_delay + sum(delays);
        total_pkts = total_pkts + length(delays);
    end
    avg_net_delay = total_delay / total_pkts;
    
    % 3. 记录到数组
    Pareto_Control_Cost(idx) = avg_ctrl_cost;
    Pareto_Network_Delay(idx) = avg_net_delay;
    
    fprintf('   --> 结果: Delay=%.2f, ControlErr=%.2f \n', avg_net_delay, avg_ctrl_cost);
end

%% 3. 绘制帕累托前沿图
figure('Name', 'Pareto Frontier', 'Color', 'w');
plot(Pareto_Network_Delay, Pareto_Control_Cost, '-o', ...
    'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
grid on;

% 美化图表
xlabel('平均网络时延 (Average Network Delay)', 'FontSize', 12);
ylabel('平均控制误差 (Average Control Error)', 'FontSize', 12);
title('控制性能与网络资源的权衡 (Pareto Frontier)', 'FontSize', 14);