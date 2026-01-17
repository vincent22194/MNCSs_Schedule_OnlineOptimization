function [sv_seq, Next_NetState, Schedule_Log] = predict_Tc_delays(t_start, offline_P, offline_T_is, offline_E, P_perf, Env, Prev_NetState)
    % 输入：当前起始时间, 离线优先级, 离线周期倍数, 离线偏移, 性能流优先级, 环境结构体
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
        delay = zeros(1, Env.N); 
        current_priority_active = Prev_NetState.current_priority_active;
        release_time = Prev_NetState.release_time;
    else
        % 如果是第一次运行，全零初始化
        dur_countv = zeros(1, Env.N);
        Superframe_success1v = zeros(1, Env.N);
        is_first_release = true(1, Env.N);
        delay = zeros(1, Env.N);
        hop_remain = zeros(1, Env.N);
        hop_counterv = zeros(1, Env.N);
        Route_dyn = cell(1, Env.N);
        current_priority_active = zeros(1, Env.N);
        release_time = nan(1, Env.N);
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
                if mod(t_abs - 1 - offline_E(iloop), Env.T_i(iloop)) == 0
                    release_time(iloop) = t_abs;
                    if ~is_first_release(iloop)
                        if ~Superframe_success1v(iloop)
                            % 失败发生的“判定时刻”是当前release时刻 t_abs
                        delay_true_fail = t_abs - release_time(iloop);
                
                        % 你写入的是 Ti+1（代表miss），这里检查：失败判定时刻是否已经 > Ti（按你的语义）
                        if delay_true_fail <= Env.T_i(iloop)
                            fprintf('[CHECK-FAIL] flow=%d, t_rel=%d, t_fail_mark=%d, delay_true_fail=%d <= Ti=%d (maybe fail marked too early)\n', ...
                                iloop, release_time(iloop), t_abs, delay_true_fail, Env.T_i(iloop));
                        end

                            sv_seq{iloop} = [sv_seq{iloop}, Env.T_i(iloop) + 1]; 
                        end
                    end
                    is_first_release(iloop) = false;
                    
                    global_instance_idx = floor((t_abs - 1 - mod(offline_E(iloop),Env.T_i(iloop))) / Env.T_i(iloop));
                    % global_instance_idx = floor((t_abs - 1 - offline_E(iloop)) / Env.T_i(iloop));

                    if iloop <= Env.Non_control_N
                        current_priority_active(iloop) = offline_P(iloop);
                    else
                        if mod(global_instance_idx, offline_T_is(iloop)) == floor(offline_E(iloop) / Env.T_i(iloop)) 
                            current_priority_active(iloop) = offline_P(iloop); % 稳定实例
                        else
                            perf_idx = iloop - Env.Non_control_N;
                            current_priority_active(iloop) = P_perf(perf_idx); % 性能实例
                            % dur_countv(iloop) = 0; %% 假装已经成功了（丢掉所有性能流（调试
                            % Superframe_success1v(iloop) = 1; 
                            % continue;
                        end
                    end
                    
                    % 重置新包状态
                    Superframe_success1v(iloop) = 0;
                    Route_dyn{iloop} = Env.Route{iloop}; 
                    dur_countv(iloop) = Env.T_i(iloop); 
                    hop_remain(iloop) = Env.Route_hop(iloop);
                    hop_counterv(iloop) = 0;
                    delay(iloop) = 0;
                end
            end
            
            % Kill (超时截断)
            for hji = 1:Env.N
                if dur_countv(hji) > 0 && Superframe_success1v(hji) == 0
                    dur_countv(hji) = dur_countv(hji) - 1;
                    eff_offset = mod(offline_E(hji), Env.T_i(hji));
                    if (Env.T_i(hji) - dur_countv(hji)) + eff_offset > Env.T_i(hji)
                        dur_countv(hji) = 0; % 超时 Kill
                    end
                end
            end
            
            % 传输调度
            active_flows = find(dur_countv > 0 & Superframe_success1v == 0);
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
                                        current_delay = (Env.T_i(flow_id) - dur_countv(flow_id)) + mod(offline_E(flow_id), Env.T_i(flow_id));
                                        delay(flow_id) = current_delay;
                                        delay_true = t_abs - release_time(flow_id);   % 从release到成功的slot差
                                        delay_true = t_abs - release_time(flow_id);   % 从release到成功的slot差
                                        if ~isnan(delay_true)
                                            % 允许误差：看你定义的“成功发生在slot末”还是“slot内”，先用0容忍
                                            if delay_true ~= current_delay
                                                fprintf('[CHECK-SUCCESS] flow=%d, t_rel=%d, t_succ=%d, delay_true=%d, current_delay=%d, eff_offset=%d\n', ...
                                                    flow_id, release_time(flow_id), t_abs, delay_true, current_delay, mod(offline_E(flow_id), Env.T_i(flow_id)));
                                            end
                                        end
                                        dur_countv(flow_id) = 0;
                                        
                                        % 成功瞬间立即记录
                                        sv_seq{flow_id} = [sv_seq{flow_id}, current_delay];
                                        % is_stable_flag = (current_priority_active(flow_id) == offline_P(flow_id));
                                        % sv_seq{flow_id} = [sv_seq{flow_id}, [current_delay; is_stable_flag]];
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
    Next_NetState.release_time = release_time;
end