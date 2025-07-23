function depth_measurements = add_depth_measurements(true_data, map_data, params)
    N_pf = length(params.time_pf);
    depth_measurements = zeros(1, N_pf);
    true_indices = 1:params.ratio:length(true_data.x);
    true_x_sub = true_data.x(true_indices);
    true_y_sub = true_data.y(true_indices);
    n_points = min(N_pf, length(true_x_sub));
    true_x_sub = true_x_sub(1:n_points);
    true_y_sub = true_y_sub(1:n_points);
    for i = 1:n_points
        true_depth = interp2(map_data.xVec, map_data.yVec, double(map_data.depth), ...
                             true_x_sub(i), true_y_sub(i), 'linear', 0);
        depth_measurements(i) = true_depth + params.sigma_pos * randn();
    end
end 