function [sv_seq, Next_NetState, Schedule_Log] = predict_Tc_delays(t_start, offline_P, offline_T_is, offline_E, P_perf, Env, Prev_NetState)
    % 输入：当前起始时间, 离线优先级, 离线周期倍数, 离线偏移, 性能流优先级, 环境结构体 
    % 输出：延迟序列，继承状态，调度日志
    During_total = Env.Ntotal;
    sv_seq = cell(1, Env.N);
    num_frames = ceil(During_total / Env.slot_num); 
    counterv_local = 0;
    Schedule_Log = [];
    % 状态变量初始化
    if nargin >= 7 && ~isempty(Prev_NetState)
        % 如果传入了上一个窗口的状态，直接继承
        dur_countv = Prev_NetState.dur_countv;
        Superframe_success1v = Prev_NetState.Superframe_success1v;
        is_first_release = Prev_NetState.is_first_release;
        hop_remain = Prev_NetState.hop_remain;
        hop_counterv = Prev_NetState.hop_counterv;
        Route_dyn = Prev_NetState.Route_dyn; 
        current_priority_active = Prev_NetState.current_priority_active;
        eligible_flag = Prev_NetState.eligible_flag;
        release_time = Prev_NetState.release_time;
    else
        % 第一次运行全零初始化
        dur_countv = zeros(1, Env.N);
        Superframe_success1v = zeros(1, Env.N);
        is_first_release = true(1, Env.N);
        hop_remain = zeros(1, Env.N);
        hop_counterv = zeros(1, Env.N);
        Route_dyn = cell(1, Env.N);
        current_priority_active = zeros(1, Env.N);
        eligible_flag = zeros(1, Env.N);
        release_time = zeros(1, Env.N);
    end
    
    for frame_idx = 1 : num_frames
        table_tit = zeros(Env.channel_num, Env.slot_num);
        table_rev = zeros(Env.channel_num, Env.slot_num);

        for slot_idx = 1 : Env.slot_num
            counterv_local = counterv_local + 1;
            t_abs = t_start + counterv_local - 1; 
            
            if counterv_local > During_total, break; end
            
            % 任务释放逻辑
            for iloop = 1:Env.N
                Ti = Env.T_i(iloop);

                eff_offset = mod(offline_E(iloop), Ti);% 作用1：偏移
                inst_offset = floor(offline_E(iloop) / Ti);% 作用2：选择第几个为稳定实例
                % 周期起点为release 时刻
                if mod(t_abs - 1, Ti) == 0
                    % 在真正的 release 边界结算上一包是否 miss
                    if ~is_first_release(iloop)
                        if ~Superframe_success1v(iloop)
                            sv_seq{iloop} = [sv_seq{iloop}, Ti + 1];
                        end
                    end
                    is_first_release(iloop) = false;
            
                    % 新包 release
                    release_time(iloop) = t_abs;
                    eligible_flag(iloop) = 0; % 刚 release，尚不可调度
                    Superframe_success1v(iloop) = 0;
                    Route_dyn{iloop} = Env.Route{iloop};
                    dur_countv(iloop) = Ti;
                    hop_remain(iloop) = Env.Route_hop(iloop);
                    hop_counterv(iloop) = 0;
            
                    % 优先级：稳定/性能实例判定
                    global_instance_idx = floor((t_abs - 1) / Ti);
            
                    if iloop <= Env.Non_control_N
                        current_priority_active(iloop) = offline_P(iloop);
                    else
                        if mod(global_instance_idx, offline_T_is(iloop)) == inst_offset
                            current_priority_active(iloop) = offline_P(iloop); % 稳定实例
                        else
                            % current_priority_active(iloop) = inf;
                            % eligible_flag(iloop) = 0;
                            % dur_countv(iloop) = 0;
                            % hop_remain(iloop) = 0;
                            % hop_counterv(iloop) = 0;
                            % Route_dyn{iloop} = [];
                            % Superframe_success1v(iloop) = 1;% 去掉性能实例
                            perf_idx = iloop - Env.Non_control_N;
                            current_priority_active(iloop) = P_perf(perf_idx); % 性能实例
                        end
                    end
                end
            
                % release 后 eff_offset 个 slot允许调度
                if ~is_first_release(iloop) && mod(t_abs - 1, Ti) == eff_offset
                    eligible_flag(iloop) = 1;
                end
            end

            
            % Kill
            for hji = 1:Env.N
                if dur_countv(hji) > 0 && Superframe_success1v(hji) == 0
                    dur_countv(hji) = dur_countv(hji) - 1;
                    % 已经超过 deadline Kill
                    % dur_countv 从 Ti 开始递减，递到 0 表示已经用完可用时间
                    if dur_countv(hji) == 0
                        eligible_flag(hji) = 0; % 不再可调度
                        hop_remain(hji) = 0;
                        hop_counterv(hji) = 0;
                        Route_dyn{hji} = [];
                    end
                end
            end

            
            % 传输调度
            active_flows = find(dur_countv > 0 & Superframe_success1v == 0 & eligible_flag == 1);
            if ~isempty(active_flows)
                [~, sort_idx] = sort(current_priority_active(active_flows));
                priority_dv = active_flows(sort_idx);

                for flow_id = priority_dv
                    if hop_remain(flow_id) > 0
                        route = Route_dyn{flow_id};
                        curr_hop = hop_counterv(flow_id) + 1;
                        src = route(curr_hop, 1); dst = route(curr_hop, 2);
                        
                        for ch = 1:Env.channel_num
                            if table_tit(ch, slot_idx) == 0
                                node_busy = any(table_tit(:, slot_idx) == src | table_rev(:, slot_idx) == src | ...
                                                table_tit(:, slot_idx) == dst | table_rev(:, slot_idx) == dst);
                                if ~node_busy
                                    table_tit(ch, slot_idx) = src; table_rev(ch, slot_idx) = dst;
                                    hop_counterv(flow_id) = hop_counterv(flow_id) + 1;
                                    hop_remain(flow_id) = hop_remain(flow_id) - 1;

                                    Schedule_Log = [Schedule_Log; t_abs, flow_id, src, dst, ch, hop_counterv(flow_id)];
                                    
                                    if hop_remain(flow_id) == 0
                                        % 传输成功
                                        Superframe_success1v(flow_id) = 1;
                                        % 计算时延
                                        delay = t_abs - release_time(flow_id) + 1;
                                        dur_countv(flow_id) = 0;
                                        eligible_flag(flow_id) = 0;
                                        % 传输完成立刻记录时延
                                        sv_seq{flow_id} = [sv_seq{flow_id}, delay];
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
    Next_NetState.dur_countv = dur_countv;
    Next_NetState.Superframe_success1v = Superframe_success1v;
    Next_NetState.is_first_release = is_first_release;
    Next_NetState.hop_remain = hop_remain;
    Next_NetState.hop_counterv = hop_counterv;
    Next_NetState.Route_dyn = Route_dyn;
    Next_NetState.current_priority_active = current_priority_active;
    Next_NetState.eligible_flag = eligible_flag;
    Next_NetState.release_time = release_time;
end