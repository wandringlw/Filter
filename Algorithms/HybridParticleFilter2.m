function [est_data, dr_data, exec_time] = HybridParticleFilter2(params, true_data, meas_data, map_data)
    %HYBRIDPARTICLEFILTER2 混合粒子滤波 (固定粒子跟随估计位置)
    %   使用全局卡尔曼滤波器估计[v,φ]，并结合固定与随机粒子估计[x,y]。
    %   固定粒子网格的中心跟随上一时刻的最终估计位置。
    
    %% 1. 初始化
    tic;
    
    % 提取参数
    N_total = params.N_total;
    N_pf = params.N_pf;
    dt_kf = params.dt_kf;
    dt_pf = params.dt_pf;
    ratio = params.ratio;
    N_fixed = params.N_fixed;
    N_random = params.N_random;
    Nparticles_total = N_fixed + N_random;
    depthGatingThreshold = params.depthGatingThreshold;
    Ne_eff_threshold = params.Ne_eff_threshold; % 直接使用传入的阈值
    
    % 提取地图数据
    depth_data = map_data.depth;
    xVecMap = map_data.xVec;
    yVecMap = map_data.yVec;
    
    fprintf('混合PF2 初始化完成:\n');
    fprintf('- 固定粒子: %d, 随机粒子: %d, 总计: %d\n', N_fixed, N_random, Nparticles_total);
    fprintf('- 固定粒子策略: 低频更新后跟随估计位置\n');
    fprintf('- 时间步数: 高频%d, 低频%d\n', N_total, N_pf);
    
    %% 2. 初始化全局卡尔曼滤波器 (处理速度v 和 航向phi)
    kf_state = [params.init_est_v; params.init_est_phi];
    kf_cov = diag([params.init_v_std^2, params.init_phi_std^2]);
    F = eye(2);
    Q = diag([params.q_v^2, params.q_phi^2]);
    H = eye(2);
    R = diag([params.r_v^2, params.r_phi^2]);
    
    %% 3. 初始化混合粒子集合 (处理位置x 和 y)
    particles = struct('x', [], 'y', [], 'w', [], 'isFixed', []);
    fixed_offsets = params.fixed_offsets;
    
    for i = 1:Nparticles_total
        if i <= N_fixed
            % 固定粒子：基于初始估计位置的网格
            [gridX, gridY] = ind2sub(sqrt(N_fixed) * [1 1], i);
            particles(i).x = params.init_est_x + fixed_offsets(gridX);
            particles(i).y = params.init_est_y + fixed_offsets(gridY);
            particles(i).isFixed = true;
        else
            % 随机粒子：基于初始估计位置的随机高斯分布
            particles(i).x = params.init_est_x + params.init_pos_std * randn();
            particles(i).y = params.init_est_y + params.init_pos_std * randn();
            particles(i).isFixed = false;
        end
        particles(i).w = 1 / Nparticles_total; % 初始权重均分
    end
    
    %% 4. 预分配结果数组
    est_x = zeros(1, N_pf); est_y = zeros(1, N_pf);
    est_v = zeros(1, N_pf); est_phi = zeros(1, N_pf);
    weight_variance = zeros(1, N_pf);
    dr_x = zeros(1, N_total); dr_y = zeros(1, N_total);
    
    % 初始状态设置
    est_x(1) = params.init_est_x; est_y(1) = params.init_est_y;
    est_v(1) = kf_state(1); est_phi(1) = kf_state(2);
    dr_x(1) = params.init_est_x; dr_y(1) = params.init_est_y;
    
    %% 5. 可视化初始化
    if params.Hybrid_showVisualization
        fig_hybrid2 = figure('Name', '混合粒子滤波器 2', 'Position', [150, 150, 800, 600]);
    end
    
    %% 6. 主循环
    pf_idx = 1;
    
    for k = 2:N_total
        % --- 6a. 高频更新：KF & 航位推算 ---
        % KF预测
        kf_state = F * kf_state;
        kf_cov = F * kf_cov * F' + Q;
        kf_state(2) = wrapToPi(kf_state(2));
    
        % KF更新 (使用IMU测量)
        z_imu = [meas_data.v_imu(k); wrapToPi(meas_data.phi_imu(k))];
        innovation = z_imu - H * kf_state;
        innovation(2) = wrapToPi(innovation(2)); % 航向残差归一化
        S = H * kf_cov * H' + R;
        K = kf_cov * H' / S;
        kf_state = kf_state + K * innovation;
        kf_state(2) = wrapToPi(kf_state(2)); % 确保航向在[-pi, pi]
        kf_cov = (eye(2) - K * H) * kf_cov;
    
        % 航位推算
        dr_x(k) = dr_x(k-1) + kf_state(1) * cos(kf_state(2)) * dt_kf;
        dr_y(k) = dr_y(k-1) + kf_state(1) * sin(kf_state(2)) * dt_kf;
    
        % --- 6b. 高频更新：粒子传播 ---
        v_est = kf_state(1);
        phi_est = kf_state(2);
        dx_center = v_est * cos(phi_est) * dt_kf;
        dy_center = v_est * sin(phi_est) * dt_kf;
    
        for i = 1:Nparticles_total
            % 所有粒子都先进行平移
            particles(i).x = particles(i).x + dx_center;
            particles(i).y = particles(i).y + dy_center;
            
            % 只有随机粒子额外增加过程噪声
            if ~particles(i).isFixed
                particles(i).x = particles(i).x + params.sigma_x * randn();
                particles(i).y = particles(i).y + params.sigma_y * randn();
            end
        end
    
        % --- 6c. 低频更新：当接收到深度观测时 ---
        if mod(k - 1, ratio) == 0
            pf_idx = pf_idx + 1;
            if pf_idx > N_pf, break; end
            
            % i. 权重更新
            z_depth = meas_data.depth(pf_idx);
            px_all = [particles.x];
            py_all = [particles.y];
            
            z_pred_all = interp2(xVecMap, yVecMap, double(depth_data), double(px_all), double(py_all), 'linear', 0);
            
            depth_diff = abs(z_depth - z_pred_all);
            likelihoods = exp(-depth_diff.^2 / (2 * params.sigma_pos^2)); % 使用高斯似然
            likelihoods(depth_diff > depthGatingThreshold) = 1e-9; % 深度门限，惩罚离谱粒子
            
            current_weights = [particles.w] .* likelihoods; % 更新权重
            w_sum = sum(current_weights);
            
            if w_sum < eps
                current_weights = ones(1, Nparticles_total) / Nparticles_total; % 权重退化处理
                fprintf('警告: 混合PF2步骤%d权重退化\n', pf_idx);
            else
                current_weights = current_weights / w_sum; % 归一化
            end
            
            % ii. 重采样
            Neff_global = 1 / sum(current_weights.^2);
            
            if Neff_global < Ne_eff_threshold
                if mod(pf_idx, 10) == 0
                     fprintf('  步骤 %d: 全局有效粒子数 %.1f, 执行重采样\n', pf_idx, Neff_global);
                end
    
                parent_indices = datasample(1:Nparticles_total, N_random, 'Weights', current_weights);
                new_random_particles = particles(parent_indices);
                for i = 1:N_random
                    particles(N_fixed + i) = new_random_particles(i);
                    particles(N_fixed + i).isFixed = false;
                end
                % 分层权重重置：随机粒子权重稍高，固定粒子权重稍低（确保归一化）
                total_weight = N_fixed * 0.8 + N_random * 1.2;  % 计算权重总和
                for i = 1:N_fixed
                    particles(i).w = 0.8 / total_weight;  % 固定粒子权重稍低
                end
                for i = N_fixed+1:Nparticles_total
                    particles(i).w = 1.2 / total_weight;  % 随机粒子权重稍高
                end
            else
                % 未重采样时，直接分配权重
                for i = 1:Nparticles_total, particles(i).w = current_weights(i); end
            end
            
            % iii. 状态估计
            final_weights = [particles.w];
            est_x(pf_idx) = sum([particles.x] .* final_weights);
            est_y(pf_idx) = sum([particles.y] .* final_weights);
            est_v(pf_idx) = kf_state(1);
            if pf_idx > 1
                est_phi(pf_idx) = est_phi(pf_idx-1) + wrapToPi(kf_state(2) - est_phi(pf_idx-1));
            else
                est_phi(pf_idx) = kf_state(2);
            end
            
            % iv. 固定粒子重新布局
            % 在得到本轮估计后，将固定粒子重新布局到该位置周围
            for i = 1:N_fixed
                [gridX, gridY] = ind2sub(sqrt(N_fixed) * [1 1], i);
                particles(i).x = est_x(pf_idx) + fixed_offsets(gridX);
                particles(i).y = est_y(pf_idx) + fixed_offsets(gridY);
            end
            
            % v. 可视化更新
            if params.Hybrid_showVisualization && mod(pf_idx, 5) == 0
                figure(fig_hybrid2); clf;
                
                % 设置显示范围
                zoomSize = params.zoomSize;
                center_x = est_x(pf_idx);
                center_y = est_y(pf_idx);
                
                map_x_min = min(xVecMap); map_x_max = max(xVecMap);
                map_y_min = min(yVecMap); map_y_max = max(yVecMap);
                
                x_min_disp = max(center_x - zoomSize/2, map_x_min);
                x_max_disp = min(center_x + zoomSize/2, map_x_max);
                y_min_disp = max(center_y - zoomSize/2, map_y_min);
                y_max_disp = min(center_y + zoomSize/2, map_y_max);
                
                % 显示地图背景
                imagesc(xVecMap, yVecMap, depth_data);
                set(gca,'YDir','normal'); colormap(jet); colorbar; hold on;
                xlim([x_min_disp, x_max_disp]); ylim([y_min_disp, y_max_disp]);
                
                % 分类显示粒子
                idxF = [particles.isFixed];
                particles_x_all = [particles.x];
                particles_y_all = [particles.y];
                
                % 固定粒子：蓝色
                scatter(particles_x_all(idxF), particles_y_all(idxF), 30, 'b', 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);
                
                % 随机粒子：白色
                scatter(particles_x_all(~idxF), particles_y_all(~idxF), 30, 'w', 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);
                
                % 绘制轨迹
                current_k_high_freq = min(k, length(true_data.x));
                plot(est_x(1:pf_idx), est_y(1:pf_idx), 'r-', 'LineWidth', 2, 'DisplayName', '混合粒子滤波器2估计');
                plot(dr_x(1:k), dr_y(1:k), 'c--', 'LineWidth', 1.5, 'DisplayName', '航位推算');
                plot(true_data.x(1:current_k_high_freq), true_data.y(1:current_k_high_freq), 'k-', 'LineWidth', 2, 'DisplayName', '真实轨迹');
                
                % 标记当前位置
                plot(true_data.x(current_k_high_freq), true_data.y(current_k_high_freq), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
                plot(est_x(pf_idx), est_y(pf_idx), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
                
                title(sprintf('混合粒子滤波器2 - 时间: %.1fs', pf_idx*dt_pf));
                xlabel('X (m)'); ylabel('Y (m)');
                legend('Location', 'northwest');
                drawnow;
            end
            
            % 进度显示
            if mod(pf_idx, 10) == 0
                fprintf('  混合PF2进度: %d/%d (%.1f%%)\n', pf_idx, N_pf, pf_idx/N_pf*100);
            end
        end
    end
    
    %% 7. 整理输出结果
    est_data = struct();
    est_data.x = est_x(1:pf_idx);
    est_data.y = est_y(1:pf_idx);
    est_data.v = est_v(1:pf_idx);
    est_data.phi = est_phi(1:pf_idx);
    est_data.time = params.time_pf(1:pf_idx);
    est_data.weight_variance = weight_variance(1:pf_idx);
    
    dr_indices = 1:ratio:(k - mod(k-1, ratio));
    dr_data.x = dr_x(dr_indices(1:pf_idx));
    dr_data.y = dr_y(dr_indices(1:pf_idx));
    
    exec_time = toc;
    fprintf('混合粒子滤波器2 执行完成! 总时间: %.3f秒\n', exec_time);
    
    end