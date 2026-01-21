function Debug = debug_simulator(t_start, offline_P, offline_T_is, offline_E, P_perf_perms, Env, Current_NetState, X_real, U_last, Q_weight, Lambda_delay)
% 返回 Debug 结构体：包含 24种perm 的总代价、流代价、delay
Control_Idx = (Env.Non_control_N + 1) : Env.N;
Control_N = Env.Control_N;

num_perm = size(P_perf_perms, 1);

J_total = zeros(num_perm, 1);
J_per_flow = zeros(num_perm, Control_N); % 每行对应一个perm，每列对应一个控制流
DelayLen_per_flow = zeros(num_perm, Control_N);
First5Delay = cell(num_perm, Control_N);
PermMat = P_perf_perms;

for p_idx = 1:num_perm
    P_perf = P_perf_perms(p_idx, :);

    % 每个perm都用同一个 NetState初始条件
    [sv_seq_pred, ~, ~] = predict_Tc_delays(t_start, offline_P, offline_T_is, offline_E, P_perf, Env, Current_NetState);

    Ji_vec = zeros(1, Control_N);

    for ii = 1:Control_N
        i = Control_Idx(ii);

        x_temp = X_real{i};
        u_old = U_last(i);

        for k = 1:length(sv_seq_pred{i})
            dk = sv_seq_pred{i}(k);

            A_cl = Env.CM_data{i}{dk};
            z_next = A_cl * [x_temp; u_old];
            x_temp = z_next(1:size(x_temp,1));
            u_old  = z_next(size(x_temp,1)+1);

            Ji_vec(ii) = Ji_vec(ii) + Env.T_i(i) * (x_temp' * Q_weight * x_temp) + Lambda_delay * dk;
        end

        DelayLen_per_flow(p_idx, ii) = length(sv_seq_pred{i});
        d = sv_seq_pred{i};
        First5Delay{p_idx, ii} = d(1:min(5,end));
    end

    J_per_flow(p_idx, :) = Ji_vec;
    J_total(p_idx) = sum(Ji_vec);
end

T = table((1:num_perm)', PermMat, J_total, ...
          J_per_flow(:,1), J_per_flow(:,2), J_per_flow(:,3), J_per_flow(:,4), ...
          DelayLen_per_flow(:,1), DelayLen_per_flow(:,2), DelayLen_per_flow(:,3), DelayLen_per_flow(:,4), ...
          'VariableNames', {'perm_id','P_perf','J_total','J_loop1','J_loop2','J_loop3','J_loop4', ...
                            'len_loop1','len_loop2','len_loop3','len_loop4'});

T_sorted = sortrows(T, 'J_total', 'ascend');

Debug.Table = T_sorted;
end
