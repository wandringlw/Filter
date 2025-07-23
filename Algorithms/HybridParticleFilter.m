function [est_data, dr_data, exec_time] = HybridParticleFilter(params, true_data, meas_data, map_data)
%HYBRIDPARTICLEFILTER 基于新模型的混合粒子滤波算法
%   使用全局卡尔曼滤波器估计[v,φ] + 固定网格粒子+随机粒子估计[x,y]
%   在解耦模式下运行，适配新模型的状态空间
%
%   输入:
%   - params: 新模型参数结构体
%   - true_data: 真实轨迹数据
%   - meas_data: 测量数据(包含v_imu, phi_imu, depth)
%   - map_data: 地图数据
%
%   输出:
%   - est_data: 混合PF估计结果
%   - dr_data: 航位推算结果
%   - exec_time: 算法执行时间

%% 初始化
tic;

% 提取基本参数
N_total = params.N_total;
N_pf = params.N_pf;
dt_kf = params.dt_kf;
dt_pf = params.dt_pf;
ratio = params.ratio;

% 混合PF特有参数
N_fixed = params.N_fixed;
N_random = params.N_random;
Nparticles_total = N_fixed + N_random;
depthGatingThreshold = params.depthGatingThreshold;
Ne_eff_threshold = params.Ne_eff_threshold;

% 提取地图数据
depth_data = map_data.depth;
xVecMap = map_data.xVec;
yVecMap = map_data.yVec;

fprintf('混合PF初始化完成:\n');
fprintf('- 固定粒子: %d个, 随机粒子: %d个, 总计: %d个\n', N_fixed, N_random, Nparticles_total);
fprintf('- 时间步数: 高频%d, 低频%d\n', N_total, N_pf);

%% 初始化全局卡尔曼滤波器 (处理速度和航向)
% 所有粒子共享同一个KF估计的速度和航向
kf_state = [params.init_est_v; params.init_est_phi];
kf_cov = diag([params.init_v_std^2, params.init_phi_std^2]);

% KF系统矩阵 (解耦模式：独立随机游走)
F = eye(2);                                    % 状态转移矩阵
Q = diag([params.q_v^2, params.q_phi^2]);    % 过程噪声协方差
H = eye(2);                                    % 测量矩阵
R = diag([params.r_v^2, params.r_phi^2]);    % 测量噪声协方差

%% 初始化混合粒子集合
particles = struct('x', [], 'y', [], 'w', [], 'isFixed', []);

% 创建固定网格粒子
fixed_offsets = params.fixed_offsets;
[Xf_init_grid, Yf_init_grid] = meshgrid(params.init_est_x + fixed_offsets, ...
                                        params.init_est_y + fixed_offsets);
Xf_init_flat = Xf_init_grid(:);
Yf_init_flat = Yf_init_grid(:);

% 初始化所有粒子
for i = 1:Nparticles_total
    if i <= N_fixed
        % 固定粒子：网格位置
        particles(i).x = Xf_init_flat(i);
        particles(i).y = Yf_init_flat(i);
        particles(i).isFixed = true;
    else
        % 随机粒子：随机位置
        particles(i).x = params.init_est_x + params.init_pos_std * randn();
        particles(i).y = params.init_est_y + params.init_pos_std * randn();
        particles(i).isFixed = false;
    end
    particles(i).w = 1/Nparticles_total;  % 初始权重均分
end

%% 预分配结果数组
est_x = zeros(1, N_pf);
est_y = zeros(1, N_pf);
est_v = zeros(1, N_pf);
est_phi = zeros(1, N_pf);
weight_variance = zeros(1, N_pf);  % 粒子权重方差（置信度指标）

dr_x = zeros(1, N_total);
dr_y = zeros(1, N_total);

%% 初始状态设置
est_x(1) = params.init_est_x;
est_y(1) = params.init_est_y;
est_v(1) = kf_state(1);
est_phi(1) = kf_state(2);

dr_x(1) = params.init_est_x;
dr_y(1) = params.init_est_y;

%% 可视化初始化
if params.Hybrid_showVisualization
    fig_hybrid = figure('Name', '混合粒子滤波器 (Hybrid PF) 仿真', 'Position', [100, 100, 800, 600]);
end

%% 主循环
pf_idx = 1; % 低频更新索引

for k = 2:N_total
    %% 高频更新：全局卡尔曼滤波器 (每个时间步)
    % 1. KF预测
    kf_state = F * kf_state;  % 状态预测 (解耦模式：无控制输入)
    kf_cov = F * kf_cov * F' + Q;  % 协方差预测
    
    % 航向角归一化
    kf_state(2) = wrapToPi(kf_state(2));
    
    % 2. KF更新 (IMU测量)
    z_imu = [meas_data.v_imu(k); wrapToPi(meas_data.phi_imu(k))];  % 确保测量航向也被归一化
    
    % 计算卡尔曼增益
    S = H * kf_cov * H' + R;
    K = kf_cov * H' / S;
    
    % 测量残差（考虑航向角的周期性）
    innovation = z_imu - H * kf_state;
    innovation(2) = wrapToPi(innovation(2));  % 航向残差归一化
    
    % 状态和协方差更新
    kf_state_update = kf_state + K * innovation;
    kf_state(1) = kf_state_update(1);  % 速度更新
    kf_state(2) = wrapToPi(kf_state_update(2));  % 航向更新（确保在[-pi, pi]范围内）
    kf_cov = (eye(2) - K * H) * kf_cov;
    
    %% 航位推算更新 (每个时间步)
    dr_x(k) = dr_x(k-1) + kf_state(1) * cos(kf_state(2)) * dt_kf;
    dr_y(k) = dr_y(k-1) + kf_state(1) * sin(kf_state(2)) * dt_kf;
    
    %% 粒子位置传播 (每个时间步)
    % 所有粒子使用相同的KF估计的速度和航向传播
    v_est = kf_state(1);
    phi_est = kf_state(2);
    
    % 计算中心位移
    dx_center = v_est * cos(phi_est) * dt_kf;
    dy_center = v_est * sin(phi_est) * dt_kf;
    
    for i = 1:Nparticles_total
        if particles(i).isFixed
            % 固定粒子：只做平移，不加噪声
            particles(i).x = particles(i).x + dx_center;
            particles(i).y = particles(i).y + dy_center;
        else
            % 随机粒子：平移 + 随机噪声
            particles(i).x = particles(i).x + dx_center + params.sigma_x * randn();
            particles(i).y = particles(i).y + dy_center + params.sigma_y * randn();
        end
    end
    
    %% 低频更新：深度测量 (每ratio步)
    if mod(k-1, ratio) == 0
        pf_idx = pf_idx + 1;
        if pf_idx > N_pf, break; end
        
        z_depth = meas_data.depth(pf_idx);
        
        %% 权重更新
        px_all = [particles.x]; py_all = [particles.y];
        z_pred_all = interp2(xVecMap, yVecMap, double(depth_data), double(px_all), double(py_all), 'linear', 0);
        depth_diff = abs(z_depth - z_pred_all);
        likelihoods = exp(-depth_diff.^2 / (2 * params.sigma_pos^2));
        likelihoods(depth_diff > depthGatingThreshold) = 1e-9;
        current_weights = [particles.w] .* likelihoods;
        w_sum = sum(current_weights);
        if w_sum < eps
            current_weights = ones(1, Nparticles_total) / Nparticles_total;
            fprintf('警告: 混合PF步骤%d权重退化\n', pf_idx);
        else
            current_weights = current_weights / w_sum;
        end
        
        %% 重采样
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

        %% 状态估计
        final_weights = [particles.w];
        est_x(pf_idx) = sum([particles.x] .* final_weights);
        est_y(pf_idx) = sum([particles.y] .* final_weights);
        est_v(pf_idx) = kf_state(1);
        if pf_idx > 1, est_phi(pf_idx) = est_phi(pf_idx-1) + wrapToPi(kf_state(2) - est_phi(pf_idx-1));
        else, est_phi(pf_idx) = kf_state(2); end
        weight_variance(pf_idx) = var(final_weights);
        
        %% 可视化更新
        if params.Hybrid_showVisualization && mod(pf_idx, 5) == 0
            figure(fig_hybrid); clf;
            zoomSize = params.zoomSize; center_x = est_x(pf_idx); center_y = est_y(pf_idx);
            map_x_min = min(xVecMap); map_x_max = max(xVecMap); map_y_min = min(yVecMap); map_y_max = max(yVecMap);
            x_min_disp = max(center_x - zoomSize/2, map_x_min); x_max_disp = min(center_x + zoomSize/2, map_x_max);
            y_min_disp = max(center_y - zoomSize/2, map_y_min); y_max_disp = min(center_y + zoomSize/2, map_y_max);
            imagesc(xVecMap, yVecMap, depth_data); set(gca,'YDir','normal'); colormap(jet); colorbar; hold on;
            xlim([x_min_disp, x_max_disp]); ylim([y_min_disp, y_max_disp]);
            idxF = [particles.isFixed]; particles_x = [particles.x]; particles_y = [particles.y];
            scatter(particles_x(idxF), particles_y(idxF), 30, 'g', 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);
            scatter(particles_x(~idxF), particles_y(~idxF), 30, 'w', 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);
            current_k = min(k, length(true_data.x));
            plot(est_x(1:pf_idx), est_y(1:pf_idx), 'r-', 'LineWidth', 2, 'DisplayName', '混合粒子滤波器估计');
            plot(dr_x(1:k), dr_y(1:k), 'b--', 'LineWidth', 1.5, 'DisplayName', '航位推算');
            plot(true_data.x(1:current_k), true_data.y(1:current_k), 'k-', 'LineWidth', 2, 'DisplayName', '真实轨迹');
            plot(true_data.x(current_k), true_data.y(current_k), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
            plot(est_x(pf_idx), est_y(pf_idx), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
            title(sprintf('混合粒子滤波器粒子分布 (时间: %.1fs)', pf_idx*dt_pf)); xlabel('X (m)'); ylabel('Y (m)'); drawnow;
        end
        
        % 进度显示
        if mod(pf_idx, 10) == 0
            fprintf('  混合PF进度: %d/%d (%.1f%%)\n', pf_idx, N_pf, pf_idx/N_pf*100);
        end
    end
end

%% 整理输出结果
est_data = struct();
est_data.x = est_x(1:pf_idx);
est_data.y = est_y(1:pf_idx);
est_data.v = est_v(1:pf_idx);
est_data.phi = est_phi(1:pf_idx);
est_data.time = params.time_pf(1:pf_idx);
est_data.weight_variance = weight_variance(1:pf_idx); % 添加权重方差数据

dr_data = struct();
dr_data.x = dr_x(1:pf_idx);
dr_data.y = dr_y(1:pf_idx);

exec_time = toc;

fprintf('混合粒子滤波器算法执行完成! 总时间: %.3f秒\n', exec_time);

end
