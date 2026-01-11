function [sv_seq] = predict_Tc_delays(t_start, offline_P, P_perf, offline_T_is, offline_E, Env)
    % 输入：当前起始时间, 离线优先级, 尝试的性能流优先级, 离线周期倍数, 离线偏移, 环境结构体
    
    During_total = Env.Ntotal;
    sv_seq = cell(1, Env.N);
    instance_count = zeros(1, Env.N); 
    
    % 状态变量初始化
    dur_countv = zeros(1, Env.N);
    Superframe_success1v = zeros(1, Env.N);
    is_first_release = true(1, Env.N);
    delay = zeros(1, Env.N);
    hop_remain = zeros(1, Env.N);
    hop_counterv = zeros(1, Env.N);
    Route_dyn = cell(1, Env.N);
    current_priority_active = zeros(1, Env.N);
    
    num_frames = ceil(During_total / Env.slot_num);
    counterv_local = 0;

    for frame_idx = 1 : num_frames
        table_tit = zeros(Env.channel_num, Env.slot_num);
        table_rev = zeros(Env.channel_num, Env.slot_num);
        
        for slot_idx = 1 : Env.slot_num
            counterv_local = counterv_local + 1;
            t_abs = t_start + counterv_local - 1; % 绝对时间线，用于判定释放
            
            if counterv_local > During_total, break; end
            
            % 任务释放逻辑
            for iloop = 1:Env.N
                if mod(t_abs - 1 - offline_E(iloop), Env.T_i(iloop)) == 0
                    % 记录上一实例结果
                    if ~is_first_release(iloop)
                        if ~Superframe_success1v(iloop)
                            sv_seq{iloop} = [sv_seq{iloop}, Env.T_i(iloop) + 1]; % 丢包延迟
                        else
                            sv_seq{iloop} = [sv_seq{iloop}, delay(iloop)];
                        end
                    end
                    
                    is_first_release(iloop) = false;
                    instance_count(iloop) = instance_count(iloop) + 1;
                    
                    % 优先级切换逻辑
                    % 判断本实例是稳定实例还是性能实例
                    if mod(instance_count(iloop)-1, offline_T_is(iloop)) == 0
                        current_priority_active(iloop) = offline_P(iloop);
                    else
                        % 性能实例使用决策变量中的优先级
                        current_priority_active(iloop) = P_perf(iloop - Env.Non_control_N);
                    end
                    
                    Superframe_success1v(iloop) = 0;
                    Route_dyn{iloop} = Env.Route{iloop}; % 路由加载
                    dur_countv(iloop) = Env.T_i(iloop); 
                    hop_remain(iloop) = Env.Route_hop(iloop);
                    hop_counterv(iloop) = 0;
                    delay(iloop) = 0;
                end
            end
            
            % Kill 策略
            for hji = 1:Env.N
                if dur_countv(hji) > 0 && Superframe_success1v(hji) == 0
                    dur_countv(hji) = dur_countv(hji) - 1;
                    eff_offset = mod(offline_E(hji), Env.T_i(hji));
                    % 截止期错过检查
                    if (Env.T_i(hji) - dur_countv(hji)) + eff_offset > Env.T_i(hji)
                        dur_countv(hji) = 0; 
                    end
                end
            end
            
            % TDMA 多信道资源调度
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
                                % 冲突检测 
                                node_busy = any(table_tit(:, slot_idx) == src | table_rev(:, slot_idx) == src | ...
                                                table_tit(:, slot_idx) == dst | table_rev(:, slot_idx) == dst);
                                if ~node_busy
                                    table_tit(ch, slot_idx) = src; table_rev(ch, slot_idx) = dst;
                                    hop_counterv(flow_id) = hop_counterv(flow_id) + 1;
                                    hop_remain(flow_id) = hop_remain(flow_id) - 1;
                                    if hop_remain(flow_id) == 0
                                        Superframe_success1v(flow_id) = 1;
                                        % 计算时延
                                        delay(flow_id) = (Env.T_i(flow_id) - dur_countv(flow_id)) + mod(offline_E(flow_id), Env.T_i(flow_id));
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
end