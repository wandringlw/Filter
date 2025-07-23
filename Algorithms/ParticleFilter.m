function [est_data, dr_data, exec_time] = ParticleFilter(params, true_data, meas_data, map_data)
%PARTICLEFILTER 基于新模型的纯粒子滤波算法
%   使用全局卡尔曼滤波器估计[v,φ] + 所有粒子共享KF结果估计[x,y]
%   在解耦模式下运行，适配新模型的状态空间
%
%   输入:
%   - params: 新模型参数结构体
%   - true_data: 真实轨迹数据
%   - meas_data: 测量数据(包含v_imu, phi_imu, depth)
%   - map_data: 地图数据
%
%   输出:
%   - est_data: 纯PF估计结果
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
N_particles = params.N_particles;

% 纯PF特有参数
depthGatingThreshold = params.depthGatingThreshold;
Ne_eff_threshold = params.Ne_eff_threshold;

% 提取地图数据
depth_data = map_data.depth;
xVecMap = map_data.xVec;
yVecMap = map_data.yVec;

fprintf('纯PF初始化完成:\n');
fprintf('- 粒子数: %d, 重采样阈值: %d\n', N_particles, Ne_eff_threshold);
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

%% 初始化粒子集合 (仅位置状态)
particles = struct('x', [], 'y', [], 'w', []);
for i = 1:N_particles
    particles(i).x = params.init_est_x + params.init_pos_std * randn();
    particles(i).y = params.init_est_y + params.init_pos_std * randn();
    particles(i).w = 1/N_particles;
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
if params.Pure_showVisualization
    fig_pure = figure('Name', '粒子滤波器 (PF) 仿真', 'Position', [100, 100, 800, 600]);
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
    z_imu = [meas_data.v_imu(k); meas_data.phi_imu(k)];
    
    % 计算卡尔曼增益
    S = H * kf_cov * H' + R;
    K = kf_cov * H' / S;
    
    % 测量残差
    innovation = z_imu - H * kf_state;
    innovation(2) = wrapToPi(innovation(2));  % 航向残差归一化
    
    % 状态和协方差更新
    kf_state = kf_state + K * innovation;
    kf_cov = (eye(2) - K * H) * kf_cov;
    
    % 航向角归一化
    kf_state(2) = wrapToPi(kf_state(2));
    
    %% 航位推算更新 (每个时间步)
    dr_x(k) = dr_x(k-1) + kf_state(1) * cos(kf_state(2)) * dt_kf;
    dr_y(k) = dr_y(k-1) + kf_state(1) * sin(kf_state(2)) * dt_kf;
    
    %% 粒子位置传播 (每个时间步)
    % 所有粒子使用相同的KF估计的速度和航向传播
    v_est = kf_state(1);
    phi_est = kf_state(2);
    
    for i = 1:N_particles
        % 位置传播 (解耦模式：仅运动学传播，无水流影响)
        particles(i).x = particles(i).x + v_est * cos(phi_est) * dt_kf + params.sigma_x * randn();
        particles(i).y = particles(i).y + v_est * sin(phi_est) * dt_kf + params.sigma_y * randn();
    end
    
    %% 低频更新：深度测量 (每ratio步)
    if mod(k-1, ratio) == 0
        pf_idx = pf_idx + 1;
        if pf_idx > N_pf, break; end
        
        z_depth = meas_data.depth(pf_idx);
        
        %% 权重更新
        % 批量处理所有粒子的深度预测
        px_all = [particles.x];
        py_all = [particles.y];
        w_all = [particles.w];
        
        % 从地图获取所有粒子位置的深度值
        z_pred_all = interp2(xVecMap, yVecMap, double(depth_data), ...
                           double(px_all), double(py_all), 'linear', 0);
        
        % 计算深度差异和门限
        depth_diff = abs(z_depth - z_pred_all);
        gating_mask = depth_diff > depthGatingThreshold;
        
        % 计算似然
        likelihoods = exp(-(z_depth - z_pred_all).^2 / (2*params.sigma_pos^2)) / ...
                     sqrt(2*pi*params.sigma_pos^2);
        
        % 应用门限（将超出门限的粒子似然设为很小值）
        likelihoods(gating_mask) = 1e-6;
        
        % 更新权重
        w_new = w_all .* likelihoods;
        w_sum = sum(w_new);
        
        if w_sum < eps
            % 权重退化处理
            w_new = ones(1, N_particles) / N_particles;
            fprintf('警告: 纯PF步骤%d权重退化\n', pf_idx);
        else
            w_new = w_new / w_sum;
        end
        
        % 将权重分配回粒子
        for i = 1:N_particles
            particles(i).w = w_new(i);
        end
        
        %% 重采样
        Neff = 1 / sum(w_new.^2);
        
        if Neff < Ne_eff_threshold
            % 系统重采样
            idx_resample = systematic_resample(w_new);
            new_particles = particles;
            
            for i = 1:N_particles
                new_particles(i) = particles(idx_resample(i));
                new_particles(i).w = 1/N_particles;
            end
            
            particles = new_particles;
            
            if mod(pf_idx, 10) == 0
                fprintf('  步骤 %d: 有效粒子数 %.1f, 执行重采样\n', pf_idx, Neff);
            end
        end
        
        %% 状态估计
        % 位置估计 (加权平均)
        est_x(pf_idx) = sum([particles.x] .* [particles.w]);
        est_y(pf_idx) = sum([particles.y] .* [particles.w]);
        
        % 速度航向估计 (使用KF结果) - 添加航向角连续性处理
        est_v(pf_idx) = kf_state(1);
        if pf_idx > 1
            % 确保航向角连续性，避免±π跳跃
            est_phi(pf_idx) = est_phi(pf_idx-1) + wrapToPi(kf_state(2) - est_phi(pf_idx-1));
        else
            est_phi(pf_idx) = kf_state(2);
        end
        
        %% 权重方差计算
        weight_variance(pf_idx) = var(w_new);
        
        %% 可视化更新
        if params.Pure_showVisualization && mod(pf_idx, 5) == 0
            figure(fig_pure); clf;
            
            % 设置显示范围
            zoomSize = params.zoomSize;
            center_x = est_x(pf_idx);
            center_y = est_y(pf_idx);
            
            % 确保显示窗口在地图范围内
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
            
            % 只绘制在显示范围内的粒子
            particles_x = [particles.x];
            particles_y = [particles.y];
            in_view = particles_x >= x_min_disp & particles_x <= x_max_disp & ...
                     particles_y >= y_min_disp & particles_y <= y_max_disp;
            
            if any(in_view)
                scatter(particles_x(in_view), particles_y(in_view), 30, 'w', 'filled', 'MarkerFaceAlpha', 0.8);
            end
            
            % 绘制轨迹
            current_k = min(k, length(true_data.x));
            plot(est_x(1:pf_idx), est_y(1:pf_idx), 'r-', 'LineWidth', 2, 'DisplayName', '粒子滤波器估计');
            plot(dr_x(1:k), dr_y(1:k), 'b--', 'LineWidth', 1.5, 'DisplayName', '航位推算');
            plot(true_data.x(1:current_k), true_data.y(1:current_k), 'k-', 'LineWidth', 2, 'DisplayName', '真实轨迹');
            
            % 标记当前位置
            plot(true_data.x(current_k), true_data.y(current_k), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
            plot(est_x(pf_idx), est_y(pf_idx), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
            
            title(sprintf('粒子滤波器粒子分布 (时间: %.1fs)', pf_idx*dt_pf));
            xlabel('X (m)'); ylabel('Y (m)');
            drawnow;
        end
        
        % 进度显示
        if mod(pf_idx, 10) == 0
            fprintf('  纯PF进度: %d/%d (%.1f%%)\n', pf_idx, N_pf, pf_idx/N_pf*100);
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
% 提取航位推算数据对应低频时间点
dr_indices = 1:params.ratio:k;
dr_data.x = dr_x(dr_indices(1:pf_idx));
dr_data.y = dr_y(dr_indices(1:pf_idx));

exec_time = toc;

fprintf('粒子滤波器算法执行完成! 总时间: %.3f秒\n', exec_time);

end

%% 辅助函数

function indices = systematic_resample(weights)
%SYSTEMATIC_RESAMPLE 系统重采样
    N = length(weights);
    positions = ((0:N-1) + rand(1))/N;
    cumulative_sum = cumsum(weights);
    indices = zeros(1, N);
    
    i = 1; j = 1;
    while i <= N && j <= N
        if positions(i) < cumulative_sum(j)
            indices(i) = j;
            i = i + 1;
        else
            j = j + 1;
        end
    end
    
    % 处理边界情况
    while i <= N
        indices(i) = N;
        i = i + 1;
    end
end 