function [est_data, dr_data, exec_time] = HybridParticleFilter3(params, true_data, meas_data, map_data)
    %HYBRIDPARTICLEFILTER3 自适应混合粒子滤波算法
    %   固定粒子的策略根据时间ts进行切换：
    %   - 策略1 (t < ts): 固定粒子跟随航位推算(DR)位置。
    %   - 策略2 (t >= ts): 固定粒子在低频更新后，重新布局到新的估计位置周围。
    %   算法流程经过整合与优化，确保理论正确性和鲁棒性。
    
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
    Ne_eff_threshold = params.Ne_eff_threshold;
    ts = params.ts; % 策略切换时间
    
    % 提取地图数据
    depth_data = map_data.depth;
    xVecMap = map_data.xVec;
    yVecMap = map_data.yVec;
    
    fprintf('自适应混合PF初始化完成:\n');
    fprintf('- 固定粒子: %d, 随机粒子: %d, 总计: %d个\n', N_fixed, N_random, Nparticles_total);
    fprintf('- 固定粒子策略切换时间: %.1fs\n', ts);
    fprintf('- 策略1 (t<%.1fs): 跟随DR位置\n', ts);
    fprintf('- 策略2 (t>=%.1fs): 跟随估计位置\n', ts);
    fprintf('- 时间步数: 高频%d, 低频%d\n', N_total, N_pf);
    
    %% 2. 初始化全局卡尔曼滤波器 (处理v, phi)
    kf_state = [params.init_est_v; params.init_est_phi];
    kf_cov = diag([params.init_v_std^2, params.init_phi_std^2]);
    F = eye(2);
    Q = diag([params.q_v^2, params.q_phi^2]);
    H = eye(2);
    R = diag([params.r_v^2, params.r_phi^2]);
    
    %% 3. 初始化混合粒子集合 (处理x, y)
    particles = struct('x', [], 'y', [], 'w', [], 'isFixed', []);
    fixed_offsets = params.fixed_offsets;
    if length(fixed_offsets)^2 < N_fixed
        error('提供的 fixed_offsets 数量不足以生成 N_fixed 个固定粒子。');
    end
    
    for i = 1:Nparticles_total
        if i <= N_fixed
            [gridX, gridY] = ind2sub(sqrt(N_fixed) * [1 1], i);
            particles(i).x = params.init_est_x + fixed_offsets(gridX);
            particles(i).y = params.init_est_y + fixed_offsets(gridY);
            particles(i).isFixed = true;
        else
            particles(i).x = params.init_est_x + params.init_pos_std * randn();
            particles(i).y = params.init_est_y + params.init_pos_std * randn();
            particles(i).isFixed = false;
        end
        particles(i).w = 1 / Nparticles_total;
    end
    
    %% 4. 预分配结果数组与状态变量
    est_x = zeros(1, N_pf); est_y = zeros(1, N_pf);
    est_v = zeros(1, N_pf); est_phi = zeros(1, N_pf);
    dr_x = zeros(1, N_total); dr_y = zeros(1, N_total);
    strategy_switched_flag = false; % 策略切换标志
    
    % 初始状态设置
    est_x(1) = params.init_est_x; est_y(1) = params.init_est_y;
    est_v(1) = kf_state(1); est_phi(1) = kf_state(2);
    dr_x(1) = params.init_est_x; dr_y(1) = params.init_est_y;
    
    %% 5. 可视化初始化
    if params.Hybrid_showVisualization
        fig_hybrid3 = figure('Name', '自适应混合粒子滤波器', 'Position', [200, 200, 800, 600]);
    end
    
    %% 6. 主循环
    pf_idx = 1;
    
    for k = 2:N_total
        current_time = (k - 1) * dt_kf;
    
        % --- 6a. 高频更新：KF & 航位推算 ---
        % KF 预测与更新
        kf_state = F * kf_state;
        kf_cov = F * kf_cov * F' + Q;
        z_imu = [meas_data.v_imu(k); wrapToPi(meas_data.phi_imu(k))];
        innovation = z_imu - H * kf_state;
        innovation(2) = wrapToPi(innovation(2));
        S = H * kf_cov * H' + R;
        K = kf_cov * H' / S;
        kf_state = kf_state + K * innovation;
        kf_state(2) = wrapToPi(kf_state(2));
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
            if particles(i).isFixed
                % 策略1: 固定粒子跟随DR位置
                if current_time < ts
                    % 将固定粒子“钉”在当前DR位置的网格上
                    [gridX, gridY] = ind2sub(sqrt(N_fixed) * [1 1], i);
                    particles(i).x = dr_x(k) + fixed_offsets(gridX);
                    particles(i).y = dr_y(k) + fixed_offsets(gridY);
                else % 策略2: 固定粒子跟随估计位置（此时仅做平移）
                    particles(i).x = particles(i).x + dx_center;
                    particles(i).y = particles(i).y + dy_center;
                end
            else % 随机粒子
                particles(i).x = particles(i).x + dx_center + params.sigma_x * randn();
                particles(i).y = particles(i).y + dy_center + params.sigma_y * randn();
            end
        end
        
        % --- 6c. 低频更新：当接收到深度观测时 ---
        if mod(k - 1, ratio) == 0
            pf_idx = pf_idx + 1;
            if pf_idx > N_pf, break; end
            
            % i. 权重更新
            z_depth = meas_data.depth(pf_idx);
            px_all = [particles.x]; py_all = [particles.y];
            z_pred_all = interp2(xVecMap, yVecMap, double(depth_data), double(px_all), double(py_all), 'linear', 0);
            depth_diff = abs(z_depth - z_pred_all);
            likelihoods = exp(-depth_diff.^2 / (2 * params.sigma_pos^2));
            likelihoods(depth_diff > depthGatingThreshold) = 1e-9;
            current_weights = [particles.w] .* likelihoods;
            w_sum = sum(current_weights);
            if w_sum < eps
                current_weights = ones(1, Nparticles_total) / Nparticles_total;
                fprintf('警告: 自适应PF步骤%d权重退化\n', pf_idx);
            else
                current_weights = current_weights / w_sum;
            end
            
            % ii. 重采样 (全局协作模式)
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
            if pf_idx > 1, est_phi(pf_idx) = est_phi(pf_idx-1) + wrapToPi(kf_state(2) - est_phi(pf_idx-1));
            else, est_phi(pf_idx) = kf_state(2); end
            
            % iv. 【核心自适应逻辑】固定粒子重新布局
            if current_time >= ts
                % 策略切换事件
                if ~strategy_switched_flag
                    fprintf('*** 时间 %.1fs: 固定粒子策略切换！***\n', current_time);
                    fprintf('    从策略1（跟随DR）切换到策略2（跟随估计位置）\n');
                    strategy_switched_flag = true;
                end
                % 策略2: 在得到新估计后，将固定粒子布局到估计位置周围
                for i = 1:N_fixed
                    [gridX, gridY] = ind2sub(sqrt(N_fixed) * [1 1], i);
                    particles(i).x = est_x(pf_idx) + fixed_offsets(gridX);
                    particles(i).y = est_y(pf_idx) + fixed_offsets(gridY);
                end
            end
            
            % v. 可视化更新
            if params.Hybrid_showVisualization && mod(pf_idx, 5) == 0
                figure(fig_hybrid3); clf;
                zoomSize = params.zoomSize; center_x = est_x(pf_idx); center_y = est_y(pf_idx);
                map_x_min = min(xVecMap); map_x_max = max(xVecMap); map_y_min = min(yVecMap); map_y_max = max(yVecMap);
                x_min_disp = max(center_x - zoomSize/2, map_x_min); x_max_disp = min(center_x + zoomSize/2, map_x_max);
                y_min_disp = max(center_y - zoomSize/2, map_y_min); y_max_disp = min(center_y + zoomSize/2, map_y_max);
                imagesc(xVecMap, yVecMap, depth_data); set(gca,'YDir','normal'); colormap(jet); colorbar; hold on;
                xlim([x_min_disp, x_max_disp]); ylim([y_min_disp, y_max_disp]);
                
                idxF = [particles.isFixed]; particles_x_all = [particles.x]; particles_y_all = [particles.y];
                if current_time < ts
                    scatter(particles_x_all(idxF), particles_y_all(idxF), 30, 'g', 'filled', 'MarkerEdgeColor', 'k'); % 策略1: 绿色
                    strategy_name = '策略1(DR)';
                else
                    scatter(particles_x_all(idxF), particles_y_all(idxF), 30, 'b', 'filled', 'MarkerEdgeColor', 'k'); % 策略2: 蓝色
                    strategy_name = '策略2(估计)';
                end
                scatter(particles_x_all(~idxF), particles_y_all(~idxF), 30, 'w', 'filled', 'MarkerEdgeColor', 'k'); % 随机粒子: 白色
                
                current_k_high_freq = min(k, length(true_data.x));
                plot(est_x(1:pf_idx), est_y(1:pf_idx), 'r-', 'LineWidth', 2, 'DisplayName', '自适应PF估计');
                plot(dr_x(1:k), dr_y(1:k), 'c--', 'LineWidth', 1.5, 'DisplayName', '航位推算');
                plot(true_data.x(1:current_k_high_freq), true_data.y(1:current_k_high_freq), 'k-', 'LineWidth', 2, 'DisplayName', '真实轨迹');
                plot(true_data.x(current_k_high_freq), true_data.y(current_k_high_freq), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
                plot(est_x(pf_idx), est_y(pf_idx), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
                
                title(sprintf('自适应混合PF - %s (时间: %.1fs)', strategy_name, pf_idx*dt_pf));
                xlabel('X (m)'); ylabel('Y (m)'); legend('Location', 'northwest'); drawnow;
            end
            
            % vi. 进度显示
            if mod(pf_idx, 10) == 0
                if current_time < ts, strategy_info = '策略1(DR)'; else, strategy_info = '策略2(估计)'; end
                fprintf('  自适应混合PF进度: %d/%d (%.1f%%) - %s\n', pf_idx, N_pf, pf_idx/N_pf*100, strategy_info);
            end
        end
    end
    
    %% 7. 整理输出结果
    est_data = struct();
    est_data.x = est_x(1:pf_idx); est_data.y = est_y(1:pf_idx);
    est_data.v = est_v(1:pf_idx); est_data.phi = est_phi(1:pf_idx);
    est_data.time = params.time_pf(1:pf_idx);
    est_data.weight_variance = zeros(1,pf_idx); % 暂不计算
    
    dr_indices = 1:ratio:(k - mod(k-1, ratio));
    dr_data.x = dr_x(dr_indices(1:pf_idx));
    dr_data.y = dr_y(dr_indices(1:pf_idx));
    
    exec_time = toc;
    fprintf('自适应混合粒子滤波器算法执行完成! 总时间: %.3f秒\n', exec_time);
    if strategy_switched_flag, fprintf('策略已在 %.1fs 切换\n', ts); end
    
    end