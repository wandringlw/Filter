function [est_data, dr_data, exec_time] = PointMassFilter(params, true_data, meas_data, map_data)
%POINTMASSFILTER 基于新模型的点质量滤波算法
%   使用卡尔曼滤波器估计[v,φ] + 点质量滤波器估计[x,y]
%   在解耦模式下运行，适配新模型的状态空间
%
%   输入:
%   - params: 新模型参数结构体
%   - true_data: 真实轨迹数据
%   - meas_data: 测量数据(包含v_imu, phi_imu, depth)
%   - map_data: 地图数据
%
%   输出:
%   - est_data: PMF估计结果
%   - dr_data: 航位推算结果
%   - exec_time: 算法执行时间

%% 初始化
tic;

% 提取基本参数
N_total = params.N_total;
N_pf = params.N_pf;
% dt_kf = params.dt_kf;  % PMF不使用高频更新
dt_pf = params.dt_pf;
ratio = params.ratio;

% PMF特有参数
Ngrid = params.Ngrid;
dx_pmf = params.dx_pmf;
sigma_z = params.sigma_pos;  % 使用新模型的位置测量噪声

% 提取地图数据
depth_data = map_data.depth;
xVecMap = map_data.xVec;
yVecMap = map_data.yVec;

fprintf('PMF初始化完成:\n');
fprintf('- 网格: %dx%d, 分辨率: %.1f m\n', Ngrid, Ngrid, dx_pmf);
fprintf('- 时间步数: 高频%d, 低频%d\n', N_total, N_pf);

%% 初始化PMF网格
x_range = linspace(params.init_est_x - floor(Ngrid/2)*dx_pmf, ...
                   params.init_est_x + floor(Ngrid/2)*dx_pmf, Ngrid);
y_range = linspace(params.init_est_y - floor(Ngrid/2)*dx_pmf, ...
                   params.init_est_y + floor(Ngrid/2)*dx_pmf, Ngrid);
[Xgrid, Ygrid] = meshgrid(x_range, y_range);
Xgrid = Xgrid';
Ygrid = Ygrid';

% 初始PMF分布(高斯分布)
sigma_init = params.init_pos_std;
dist2 = (Xgrid - params.init_est_x).^2 + (Ygrid - params.init_est_y).^2;
pmf = exp(-dist2/(2*sigma_init^2));
pmf = pmf / sum(pmf(:));

%% 初始化卡尔曼滤波器 (处理速度和航向)
% 状态向量 [v, phi]
kf_state = [params.init_est_v; params.init_est_phi];
kf_cov = diag([params.init_v_std^2, params.init_phi_std^2]);

% KF系统矩阵 (解耦模式：独立随机游走)
F = eye(2);                                    % 状态转移矩阵
Q = diag([params.q_v^2, params.q_phi^2]);    % 过程噪声协方差
H = eye(2);                                    % 测量矩阵
R = diag([params.r_v^2, params.r_phi^2]);    % 测量噪声协方差

%% 预分配结果数组
est_x = zeros(1, N_pf);
est_y = zeros(1, N_pf);
est_v = zeros(1, N_pf);
est_phi = zeros(1, N_pf);

dr_x = zeros(1, N_pf);
dr_y = zeros(1, N_pf);

%% 初始状态设置
est_x(1) = params.init_est_x;
est_y(1) = params.init_est_y;
est_v(1) = kf_state(1);
est_phi(1) = kf_state(2);

dr_x(1) = params.init_est_x;
dr_y(1) = params.init_est_y;

%% 可视化初始化
if params.PMF_showVisualization
    fig_pmf = figure('Name', '点质量滤波器 (PMF) 仿真', 'Position', [100, 100, 1200, 600]);
    
    % 左侧：轨迹图
    subplot(1, 2, 1);
    hold on;
    plot(true_data.x, true_data.y, 'k-', 'LineWidth', 1.5, 'DisplayName', '真实轨迹');
    h_est_traj = plot(est_x(1), est_y(1), 'r-', 'LineWidth', 2, 'DisplayName', '点质量滤波器估计');
    h_dr_traj = plot(dr_x(1), dr_y(1), 'b--', 'LineWidth', 1.5, 'DisplayName', '航位推算');
    h_true_pos = plot(true_data.x(1), true_data.y(1), 'ko', 'MarkerSize', 8, 'DisplayName', '当前真实位置');
    h_est_pos = plot(est_x(1), est_y(1), 'ro', 'MarkerSize', 8, 'DisplayName', '当前估计位置');
    title('轨迹对比');
    xlabel('X (m)'); ylabel('Y (m)');
    legend('Location', 'best');
    grid on; axis equal;
    
    % 右侧：PMF分布
    subplot(1, 2, 2);
    h_pmf_surf = surf(Xgrid, Ygrid, pmf, 'EdgeColor', 'none');
    colormap(jet); colorbar;
    title('点质量滤波器概率分布');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('概率');
    view(3); grid on;
    
    drawnow;
end

%% 主循环
pf_idx = 1;
for k = 2:N_total
    %% 高频更新：卡尔曼滤波器 (每个时间步)
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
    
    %% 低频更新：PMF (每ratio步)
    if mod(k-1, ratio) == 0
        pf_idx = pf_idx + 1;
        if pf_idx > N_pf, break; end
        
        %% PMF预测：使用KF估计传播网格
        v_est = kf_state(1);
        phi_est = kf_state(2);
        
        % 计算位移 (解耦模式：仅运动学传播，无水流影响)
        deltaX = v_est * dt_pf * cos(phi_est);
        deltaY = v_est * dt_pf * sin(phi_est);
        
        % 传播网格中心
        Xgrid = Xgrid + deltaX;
        Ygrid = Ygrid + deltaY;
        
        % 添加过程噪声 (高斯卷积)
        kernel_size = ceil(3*params.sigma_x/dx_pmf);
        kernel_size = max(1, min(kernel_size, 15));
        
        vec = -kernel_size:kernel_size;
        [Xk, Yk] = meshgrid(vec, vec);
        dist2 = (Xk.^2 + Yk.^2) * (dx_pmf^2);
        gaussKernel = exp(-dist2/(2*params.sigma_x^2));
        gaussKernel = gaussKernel / sum(gaussKernel(:));
        
        pmf_pred = conv2(pmf, gaussKernel, 'same');
        
        %% PMF更新：深度测量
        z_depth = meas_data.depth(pf_idx);
        
        % 从地图获取深度值
        depth_grid = interp2(xVecMap, yVecMap, double(depth_data), ...
                           double(Xgrid), double(Ygrid), 'linear', 0);
        
        % 计算似然
        diffMat = (z_depth - depth_grid);
        pz = exp(-diffMat.^2/(2*sigma_z^2));
        
        % 贝叶斯更新
        pmf_update = pmf_pred .* pz;
        
        % 归一化
        sum_pmf = sum(pmf_update(:));
        if sum_pmf < 1e-10
            pmf = pmf_pred;  % 退化处理
            fprintf('警告: PMF步骤%d权重退化\n', pf_idx);
        else
            pmf = pmf_update / sum_pmf;
        end
        
        %% 状态估计
        % PMF位置估计 (加权平均)
        est_x(pf_idx) = sum(sum(Xgrid .* pmf));
        est_y(pf_idx) = sum(sum(Ygrid .* pmf));
        
        % KF速度航向估计 - 添加航向角连续性处理
        est_v(pf_idx) = kf_state(1);
        if pf_idx > 1
            % 确保航向角连续性，避免±π跳跃
            est_phi(pf_idx) = est_phi(pf_idx-1) + wrapToPi(kf_state(2) - est_phi(pf_idx-1));
        else
            est_phi(pf_idx) = kf_state(2);
        end
        
        % 航位推算更新
        dr_x(pf_idx) = dr_x(pf_idx-1) + v_est * dt_pf * cos(phi_est);
        dr_y(pf_idx) = dr_y(pf_idx-1) + v_est * dt_pf * sin(phi_est);
        
        %% 可视化更新
        if params.PMF_showVisualization && mod(pf_idx, 5) == 0
            figure(fig_pmf);
            
            % 更新轨迹
            subplot(1, 2, 1);
            current_k = min(k, length(true_data.x));
            set(h_true_pos, 'XData', true_data.x(current_k), 'YData', true_data.y(current_k));
            set(h_est_pos, 'XData', est_x(pf_idx), 'YData', est_y(pf_idx));
            set(h_est_traj, 'XData', est_x(1:pf_idx), 'YData', est_y(1:pf_idx));
            set(h_dr_traj, 'XData', dr_x(1:pf_idx), 'YData', dr_y(1:pf_idx));
            
            % 更新PMF分布
            subplot(1, 2, 2);
            set(h_pmf_surf, 'XData', Xgrid, 'YData', Ygrid, 'ZData', pmf, 'CData', pmf);
            title(sprintf('点质量滤波器概率分布 (时间: %.1fs)', pf_idx*dt_pf));
            
            drawnow;
        end
        
        % 进度显示
        if mod(pf_idx, 10) == 0
            fprintf('  PMF进度: %d/%d (%.1f%%)\n', pf_idx, N_pf, pf_idx/N_pf*100);
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

dr_data = struct();
dr_data.x = dr_x(1:pf_idx);
dr_data.y = dr_y(1:pf_idx);

exec_time = toc;

fprintf('点质量滤波器算法执行完成! 总时间: %.3f秒\n', exec_time);

end 