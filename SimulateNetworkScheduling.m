function [sv_seq] = SimulateNetworkScheduling(Route, currentPeriod, currentEgili, T_i, current_priority, channel_num, slot_num, fixed_duration)
% SimulateNetworkScheduling 时隙级多信道冲突避免调度仿真
%
% 输入参数:
%   Route           : cell数组 (1xN), 每个单元包含 [src, dst; src, dst...] 的路由矩阵
%   currentPeriod   : 向量 (1xN), 每个流的周期 (单位: 时隙)
%   currentEgili    : 向量 (1xN), 每个流的释放偏移量 (单位: 时隙)
%   T_i             : 向量 (1xN), 每个流的截止期 (单位: 时隙)
%   current_priority: 向量 (1xN), 流ID的排列，按优先级从高到低排序
%   channel_num     : 标量, 可用信道数量
%   slot_num        : 标量, 每帧的时隙数
%   fixed_duration  : 标量, 强制指定仿真总时隙数
%
% 输出参数:
%   sv_seq          : cell数组 (1xN), 每个单元包含该流每次传输的延迟序列 (超时记为 T_i+1)

    %% 1. 初始化参数
    N = length(Route); 
    Ntotal = N;
    
    % 计算每一跳的步数
    Route_hop = zeros(1, N);
    for i = 1:N
        Route_hop(i) = size(Route{i}, 1);
    end

    % 初始化输出数据结构
    sv_seq = cell(1, Ntotal);
    
    % 运行时状态变量
    Route_dyn = cell(1, Ntotal);
    for iiu = 1:Ntotal
        sv_seq{iiu} = [];
        Route_dyn{iiu} = [];
    end
    delay = zeros(1, Ntotal);           
    dur_countv = zeros(1, Ntotal);      
    counterv = 0;                       
    Superframe_success1v = zeros(1, N); 
    is_first_release = true(1,N);       
    
    hop_counterv = zeros(1, N);         
    hop_remain = zeros(1, N);           
    
    % 调度表
    schedule_csp_loopv = zeros(channel_num, slot_num); 
    schedule_table_titv = zeros(channel_num, slot_num); 
    schedule_table_revv = zeros(channel_num, slot_num); 
    
    % 计算总帧数
    num_frames = ceil(fixed_duration / slot_num);

    %% 2. 调度主循环
    for frame_idx = 1 : num_frames
        
        schedule_table_titv(:) = 0;
        schedule_table_revv(:) = 0;
        schedule_csp_loopv(:) = 0;
        
        for slot_idx = 1 : slot_num
            
            counterv = counterv + 1; 
            
            if counterv > fixed_duration
                break;
            end
            
            %% A. 任务释放与截止期检查
            for iloop = 1:N
                if mod(counterv - 1 - currentEgili(iloop), currentPeriod(iloop)) == 0
                    
                    % 1. 结算上一个任务实例
                    if ~is_first_release(iloop)
                        % 仅仅更新延迟序列
                        if ~Superframe_success1v(iloop)
                            % 没传完，强制标记为超时 (T_i + 1)
                            actual_delay = T_i(iloop) + 1;
                            sv_seq{iloop} = [sv_seq{iloop}, actual_delay];
                        else
                            final_delay = delay(iloop);
                            sv_seq{iloop} = [sv_seq{iloop}, final_delay];
                        end
                    end
                
                    % 2. 释放新任务实例
                    is_first_release(iloop) = false;
                    Superframe_success1v(iloop) = 0;      
                    Route_dyn{iloop} = Route{iloop};      
                    dur_countv(iloop) = currentPeriod(iloop); 
                    hop_remain(iloop) = Route_hop(iloop); 
                    hop_counterv(iloop) = 0;              
                end
            end
            
            %% B. 时间流逝更新
            for hji = 1:N
                if dur_countv(hji) > 0 && Superframe_success1v(hji) == 0 
                    dur_countv(hji) = dur_countv(hji) - 1;
                    phy_elapsed = (currentPeriod(hji) - dur_countv(hji)); 
                    if phy_elapsed > T_i(hji)
                        dur_countv(hji) = 0; 
                    end
                end
            end

            %% C. 资源调度
            active_flows = find(dur_countv(1:N) > 0 & Superframe_success1v(1:N) == 0);
            
            if ~isempty(active_flows)
                priority_dv = intersect(current_priority, active_flows, 'stable');
                
                for idx = 1:length(priority_dv)
                    flow_id = priority_dv(idx);
                    route = Route_dyn{flow_id};
                    
                    if hop_remain(flow_id) > 0
                        current_hop_idx = hop_counterv(flow_id) + 1;
                        src_node = route(current_hop_idx, 1);
                        dst_node = route(current_hop_idx, 2);
                        
                        for ch = 1:channel_num
                            if schedule_table_titv(ch, slot_idx) == 0
                                node_busy = 0;
                                for other_ch = 1:channel_num
                                    if other_ch ~= ch
                                        occ_tx = schedule_table_titv(other_ch, slot_idx);
                                        occ_rx = schedule_table_revv(other_ch, slot_idx);
                                        if occ_tx == src_node || occ_rx == src_node || ...
                                           occ_tx == dst_node || occ_rx == dst_node
                                            node_busy = 1;
                                            break;
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
                                        delay(flow_id) = currentPeriod(flow_id) - dur_countv(flow_id);
                                        dur_countv(flow_id) = 0; 
                                    end
                                    break; 
                                end
                            end
                        end % end channel loop 
                    end
                end % end priority_dv 
            end
        end % end slot loop
    end %end frame loop
end