function [Phi, G_star, G_k] = get_mats(A_i, B_i, d_i_k_slot, Ti_slot)
    % 传入的参数以时隙计，本函数一个时隙为10ms
    T_i = Ti_slot * 0.01;
    d_i = d_i_k_slot * 0.01;
    
    Phi = expm(A_i * T_i);
    
    func = @(t) expm(A_i * t) * B_i;

    if d_i_k_slot <= Ti_slot && d_i_k_slot > 0
        G_star = integral(func, T_i - d_i, T_i, 'ArrayValued', true);
        G_k    = integral(func, 0, T_i - d_i, 'ArrayValued', true);
    else
        G_star = integral(func, 0, T_i, 'ArrayValued', true);
        G_k    = zeros(size(B_i)); 
    end
end