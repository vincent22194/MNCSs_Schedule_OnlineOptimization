function [Phi, G_star, G_k] = get_mats(A_i, B_i, d_i_k_slot, Ti_slot)
    % 这里统一使用“秒”作为单位，1 Slot = 0.01s
    T_i = Ti_slot * 0.01;
    d_i = d_i_k_slot * 0.01;
    
    % 计算系统状态转移矩阵 Y_i = e^(A_i * T_i)
    Phi = expm(A_i * T_i);
    
    % 定义积分函数 exp(A_i * t) * B_i
    func = @(t) expm(A_i * t) * B_i;

    % 3. 根据时延是否超过周期判断分段逻辑
    if d_i_k_slot <= Ti_slot && d_i_k_slot > 0
        % 情况 A: 正常传输（或时延小于周期）
        % G_star 对应上一周期指令作用的部分（积分区间 T_i-d_i 到 T_i）
        G_star = integral(func, T_i - d_i, T_i, 'ArrayValued', true);
        % G_k 对应本周期新指令作用的部分（积分区间 0 到 T_i-d_i）
        G_k    = integral(func, 0, T_i - d_i, 'ArrayValued', true);
    else
        % 情况 B: 丢包（Kill 策略）或时延超过周期
        % 根据公式(4)和Kill策略，新指令没传到，整个周期都在运行上一个指令
        G_star = integral(func, 0, T_i, 'ArrayValued', true);
        G_k    = zeros(size(B_i)); % 本周期新指令作用为 0
    end
end