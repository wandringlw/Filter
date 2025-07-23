function [est_data, exec_time] = RaoBlackwellizedParticleFilter(params, true_data, meas_data, map_data)
%RAOBLACKWELLIZEDPARTICLEFILTER 新模型下的Rao-Blackwellized粒子滤波器
%   实现基于新运动学模型的RBPF算法
%
%   输入:
%   - params: 算法参数结构体
%   - true_data: 真实轨迹数据 (用于控制输入)
%   - meas_data: 测量数据结构体
%   - map_data: 地图数据 (用于深度测量更新和可视化)
%
%   输出:
%   - est_data: 估计结果结构体
%   - exec_time: 算法执行时间

%% 算法初始化
tic; % 开始计时

N_particles = params.N_particles;
N_total = params.N_total;
N_pf = length(params.time_pf);
Ne_eff_threshold = params.Ne_eff_threshold;

%% 初始化粒子集合
% 使用结构体数组存储粒子
particles(N_particles) = struct();

for i = 1:N_particles
    % 非线性状态初始化 (位置) [x; y]
    particles(i).pos = [params.init_est_x; params.init_est_y] + ...
                       params.init_pos_std * randn(2, 1);
    
    % 粒子权重
    particles(i).weight = 1/N_particles;
    
    % 每个粒子的卡尔曼滤波器 (处理速度和航向)
    particles(i).kf = struct();
    particles(i).kf.state = [params.init_est_v; params.init_est_phi] + ...
                           [params.init_v_std; params.init_phi_std] .* randn(2, 1);
    particles(i).kf.cov = diag([params.init_v_std^2, params.init_phi_std^2]);
end


%% 预分配输出数组
est_x = zeros(1, N_pf);
est_y = zeros(1, N_pf);
est_v = zeros(1, N_pf);
est_phi = zeros(1, N_pf);

% 初始估计
est_x(1) = params.init_est_x;
est_y(1) = params.init_est_y;
est_v(1) = params.init_est_v;
est_phi(1) = params.init_est_phi;

%% 提取地图数据
depth_data = map_data.depth;
xVecMap = map_data.xVec;
yVecMap = map_data.yVec;

%% 可视化初始化
if params.visualization.show_real_time
    fig_realtime = figure('Name', '新模型RBPF粒子跟踪');
    if params.visualization.save_video
        vidObj = VideoWriter(params.visualization.video_filename, 'MPEG-4');
        vidObj.FrameRate = 10; 
        vidObj.Quality = 95; 
        open(vidObj);
    end
end

%% RBPF主循环
pf_idx = 1; % 低频更新索引

% 预先计算状态转移矩阵和控制矩阵
F = [1 - params.k_d * params.dt_kf, 0;
     0, 1];
B = [params.dt_kf; 
     params.dt_kf / params.T_r];
H = eye(2); % 观测矩阵 (直接观测速度和航向)

for k = 2:N_total
    %% 获取当前时间步的控制输入
    if isfield(true_data, 'control_a_c') && isfield(true_data, 'control_delta_r')
        u_k = [true_data.control_a_c(k); true_data.control_delta_r(k)];
    else
        % 如果没有控制输入，使用默认值
        time_current = params.time_kf(k);
        u_k = [params.control.throttle_constant; 
               params.control.steering_amplitude * sin(2*pi * time_current / params.control.steering_period)];
    end
    
    %% 1. 预测步骤 (Prediction)
    for i = 1:N_particles
        %% 1a. 条件线性状态预测 (KF预测 - 速度和航向)
        % 基础KF预测
        state_pred = F * particles(i).kf.state + B .* u_k;
        
        % 深度耦合模型：加入水流切变效应
        if isfield(params, 'coupling') && (params.coupling.C_v ~= 0 || params.coupling.C_phi ~= 0)
            % 计算当前粒子位置的水流场梯度（散度和旋度）
            [~, ~, divergence, vorticity] = simulate_current_field(particles(i).pos(1), particles(i).pos(2), params);
            
            % 速度切变效应：散度影响 (a_v_shear = C_v * divergence)
            v_shear_effect = params.coupling.C_v * divergence;
            
            % 航向切变效应：旋度影响 (a_phi_shear = C_phi * vorticity)
            phi_shear_effect = params.coupling.C_phi * vorticity;
            
            % 将切变效应作为额外的输入项加入状态预测
            shear_input = [v_shear_effect; phi_shear_effect] * params.dt_kf;
            state_pred = state_pred + shear_input;
        end
        
        % 更新粒子的KF状态和协方差
        particles(i).kf.state = state_pred;
        particles(i).kf.cov = F * particles(i).kf.cov * F' + params.Q_vel;
        
        % 航向角归一化
        particles(i).kf.state(2) = wrapToPi(particles(i).kf.state(2));
        
        %% 1b. 非线性状态预测 (位置传播)
        % 使用KF后验速度和航向进行位置预测
        v_est = particles(i).kf.state(1);
        phi_est = particles(i).kf.state(2);
        
        % 根据模型类型选择位置传播方式
        if isfield(params, 'coupling') && (params.coupling.C_v ~= 0 || params.coupling.C_phi ~= 0)
            % 耦合模式：考虑水流影响
            [u_c, v_c] = simulate_current_field(particles(i).pos(1), particles(i).pos(2), params);
            pos_velocity = [v_est * cos(phi_est) + u_c;
                           v_est * sin(phi_est) + v_c];
        else
            % 解耦模式：仅运动学传播，无水流影响（与PF保持一致）
            pos_velocity = [v_est * cos(phi_est);
                           v_est * sin(phi_est)];
        end
        
        particles(i).pos = particles(i).pos + pos_velocity * params.dt_kf + ...
                          sqrt(diag(params.Q_pos)) .* randn(2, 1);
    end
    
    %% 2. 高频更新步骤 (IMU测量 - 每个时间步)
    z_imu = [meas_data.v_imu(k); meas_data.phi_imu(k)];
    
    for i = 1:N_particles
        %% KF更新步骤 (速度和航向测量)
        % 计算卡尔曼增益
        S = H * particles(i).kf.cov * H' + params.R_imu;
        K = particles(i).kf.cov * H' / S;
        
        % 测量残差
        innovation = z_imu - H * particles(i).kf.state;
        % 航向残差需要角度归一化
        innovation(2) = wrapToPi(innovation(2));
        
        % 状态和协方差更新
        particles(i).kf.state = particles(i).kf.state + K * innovation;
        particles(i).kf.cov = (eye(2) - K * H) * particles(i).kf.cov;
        
        % 航向角归一化
        particles(i).kf.state(2) = wrapToPi(particles(i).kf.state(2));
    end
    
    %% 3. 低频更新步骤 (位置测量)
    if mod(k-1, params.ratio) == 0
        pf_idx = pf_idx + 1;
        if pf_idx > N_pf, break; end
        
        % 获取深度测量值
        z_depth = meas_data.depth(pf_idx);
        
        %% 位置测量更新权重 (使用深度测量和地图匹配)
        % 提取所有粒子的位置
        particle_x = zeros(1, N_particles);
        particle_y = zeros(1, N_particles);
        for i = 1:N_particles
            particle_x(i) = particles(i).pos(1);
            particle_y(i) = particles(i).pos(2);
        end
        
        % 从地图获取所有粒子位置的预测深度值
        z_pred_all = interp2(xVecMap, yVecMap, double(depth_data), ...
                           double(particle_x), double(particle_y), 'linear', 0);
        
        % 计算深度差异和门限
        depth_diff = abs(z_depth - z_pred_all);
        gating_mask = depth_diff > params.depthGatingThreshold;
        
        % 批量计算所有粒子的似然
        likelihoods = exp(-(z_depth - z_pred_all).^2 / (2*params.sigma_pos^2)) / ...
                     sqrt(2*pi*params.sigma_pos^2);
        
        % 应用门限（将超出门限的粒子似然设为很小值）
        likelihoods(gating_mask) = 1e-6;
        
        % 更新权重
        weights = [particles.weight];
        w_new = weights .* likelihoods;
        w_sum = sum(w_new);
        
        if w_sum < eps
            % 权重退化处理
            w_new = ones(1, N_particles) / N_particles;
        else
            w_new = w_new / w_sum;
        end
        
        % 将权重分配回粒子
        for i = 1:N_particles
            particles(i).weight = w_new(i);
        end
        
        %% 状态估计
        [est_x(pf_idx), est_y(pf_idx), est_v(pf_idx), phi_raw] = ...
            compute_state_estimate(particles);
        
        % 添加航向角连续性处理
        if pf_idx > 1
            est_phi(pf_idx) = est_phi(pf_idx-1) + wrapToPi(phi_raw - est_phi(pf_idx-1));
        else
            est_phi(pf_idx) = phi_raw;
        end
        
        %% 重采样
        weights = [particles.weight];
        N_eff = 1 / sum(weights.^2);
        
        if N_eff < Ne_eff_threshold
            % 系统重采样
            idx = systematic_resample(weights);
            particles = particles(idx);
            % 重置权重
            for i = 1:N_particles
                particles(i).weight = 1/N_particles;
            end
        end
        
        %% 实时可视化更新
        if params.visualization.show_real_time && mod(pf_idx-1, params.visualization.update_interval) == 0
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
            
            % 清除画布并重新绘制
            figure(fig_realtime); clf;
            
            % 显示地图背景
            imagesc(xVecMap, yVecMap, depth_data);
            set(gca,'YDir','normal'); colormap(jet); colorbar; hold on;
            xlim([x_min_disp, x_max_disp]); ylim([y_min_disp, y_max_disp]);
            
            % 添加水流场箭头显示
            arrow_step = 120;
            [x_arrow_global, y_arrow_global] = meshgrid(map_x_min:arrow_step:map_x_max, map_y_min:arrow_step:map_y_max);
            [u_arrow_global, v_arrow_global] = simulate_current_field(x_arrow_global, y_arrow_global, params);
            
            % 绘制全局水流箭头
            quiver(x_arrow_global, y_arrow_global, u_arrow_global, v_arrow_global, 0.6, 'w', 'LineWidth', 1.2);
            
            % 只绘制在显示范围内的粒子
            in_view = particle_x >= x_min_disp & particle_x <= x_max_disp & ...
                      particle_y >= y_min_disp & particle_y <= y_max_disp;
            
            if any(in_view)
                scatter(particle_x(in_view), particle_y(in_view), 30, 'r', 'filled', 'MarkerFaceAlpha', 0.8);
            end
            
            % 绘制估计轨迹
            plot(est_x(1:pf_idx), est_y(1:pf_idx), 'r-', 'LineWidth', 3);
            
            % 绘制真实轨迹
            current_k = min((pf_idx-1)*params.ratio + 1, length(true_data.x));
            plot(true_data.x(1:current_k), true_data.y(1:current_k), 'k-', 'LineWidth', 2);
            
            % 标记当前真实位置和估计位置
            plot(true_data.x(current_k), true_data.y(current_k), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
            plot(est_x(pf_idx), est_y(pf_idx), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
            
            % 设置标题和标签
            current_time = params.time_pf(pf_idx);
            title(sprintf('新模型RBPF 粒子分布 (Time = %.2f s)', current_time));
            xlabel('x (m)'); ylabel('y (m)'); 
            drawnow;
            
            % 保存视频帧
            if params.visualization.save_video && exist('vidObj', 'var')
                frame = getframe(fig_realtime); 
                writeVideo(vidObj, frame);
            end
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

% 添加有效粒子数记录（用于可视化）
if exist('N_eff', 'var')
    est_data.N_eff = N_eff * ones(size(est_data.time));
else
    est_data.N_eff = [];
end

exec_time = toc;

%% 清理可视化资源
if exist('vidObj','var') && params.visualization.show_real_time && params.visualization.save_video
    close(vidObj); 
end

end

%% 辅助函数

function [est_x, est_y, est_v, est_phi] = compute_state_estimate(particles)
%COMPUTE_STATE_ESTIMATE 计算加权状态估计
    weights = [particles.weight];
    
    % 位置估计 (加权平均)
    positions = zeros(length(particles), 2);
    for i = 1:length(particles)
        positions(i, :) = particles(i).pos';
    end
    est_x = sum(weights .* positions(:,1)');
    est_y = sum(weights .* positions(:,2)');
    
    % 速度和航向估计
    velocities = zeros(length(particles), 1);
    headings = zeros(length(particles), 1);
    
    for i = 1:length(particles)
        velocities(i) = particles(i).kf.state(1);
        headings(i) = particles(i).kf.state(2);
    end
    
    % 速度加权平均
    est_v = sum(weights .* velocities');
    
    % 航向角加权平均 (使用圆周平均)
    cos_sum = sum(weights .* cos(headings'));
    sin_sum = sum(weights .* sin(headings'));
    est_phi = atan2(sin_sum, cos_sum);
end

function idx = systematic_resample(weights)
%SYSTEMATIC_RESAMPLE 系统重采样
    N = length(weights);
    positions = ((0:N-1) + rand()) / N;
    idx = zeros(1, N);
    cumsum_w = cumsum(weights);
    i = 1; j = 1;
    
    while i <= N && j <= N
        if positions(i) < cumsum_w(j)
            idx(i) = j;
            i = i + 1;
        else
            j = j + 1;
        end
    end
    
    % 处理边界情况
    while i <= N
        idx(i) = N;
        i = i + 1;
    end
end 