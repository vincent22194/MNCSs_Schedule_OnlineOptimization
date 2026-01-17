clc; clear; close all;

%% ====== 复制你主程序里的环境初始化（尽量保持一致） ======
PHY_SLOT_DURATION_MS = 10;
Ntotal_list = [20 38 40 50 60 70 88 90 120 110 220 194 128];  % 你可改

Non_control_N = 2;
Control_N = 4;
N = Non_control_N + Control_N;

Tnoncontrol = [15 30];
channel_num = 1; slot_num = 10;
T_stb_multipliers = [2, 3, 4];

physical_model;
T_i = [Tnoncontrol, T1, T2, T3, T4];

load('Route_set_cell.mat');
Route_hop = zeros(1, N);
for i = 1:N
    if ~isempty(Route{i}), Route_hop(i) = size(Route{i}, 1); else, Route_hop(i)=0; end
end

load('physical_matrices_struct.mat');
CM_cell_array = cell(1, N);
for i = (Non_control_N + 1) : N
    varname = sprintf('CM_%d_%d', i, T_i(i));
    if exist('CM_data', 'var') && isfield(CM_data, varname)
        CM_cell_array{i} = CM_data.(varname);
    end
end

offline_P = [3, 2, 1, 4, 5, 6];
offline_T_is = [1, 1, 3, 3, 4, 2];
offline_E = [10, 14, 45, 30, 21, 3];

P_perf_fixed = [10 9 8 7]; % 固定一个（避免最优策略掩盖悬挂）
Control_Idx = (Non_control_N+1):N;

%% ====== 扫描测试 ======
Total_Sim_Slots = 600;    % 不用太长，够看现象
t_start0 = 1;

fprintf('==== Pending-packet scan start ====\n');

for Ntotal = Ntotal_list
    Env.N = N;
    Env.Non_control_N = Non_control_N;
    Env.Control_N = Control_N;
    Env.T_i = T_i;
    Env.T_stb_multipliers = T_stb_multipliers;
    Env.Route_hop = Route_hop;
    Env.Tnoncontrol = Tnoncontrol;
    Env.Route = Route;
    Env.Ntotal = Ntotal;
    Env.channel_num = channel_num;
    Env.slot_num = slot_num;
    Env.CM_data = CM_cell_array;

    Current_NetState = [];  % 每种 Ntotal 从空状态开始，便于比较
    max_end_pending_all = 0;
    max_end_pending_ctrl = 0;

    for t_start = t_start0 : Env.Ntotal : Total_Sim_Slots
        [~, Next_NetState, ~] = predict_Tc_delays(t_start, offline_P, offline_T_is, offline_E, P_perf_fixed, Env, Current_NetState);
        Current_NetState = Next_NetState;

        end_pending_all = sum(Next_NetState.dur_countv > 0 & Next_NetState.Superframe_success1v == 0);
        end_pending_ctrl = sum(Next_NetState.dur_countv(Control_Idx) > 0 & Next_NetState.Superframe_success1v(Control_Idx) == 0);

        max_end_pending_all = max(max_end_pending_all, end_pending_all);
        max_end_pending_ctrl = max(max_end_pending_ctrl, end_pending_ctrl);
    end

    fprintf('Ntotal=%3d | max_end_pending_all=%d | max_end_pending_ctrl=%d\n', ...
        Ntotal, max_end_pending_all, max_end_pending_ctrl);
end

fprintf('==== Done ====\n');
