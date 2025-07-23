function params = config_params(gui_params)
%CONFIG_PARAMS 滤波器平台统一配置参数
%   定义所有算法的配置参数
%
%   用法:
%   params = config_params(gui_params)  % 使用GUI参数更新
%
%   输入:
%   - gui_params: GUI参数结构体
%
%   输出:
%   - params: 配置参数结构体

% 获取算法类型和模型类型
filter_type = gui_params.filter_type;
model_type = gui_params.model_type;  % 从GUI获取模型类型

%% 时间和仿真参数
params.T = 1000;              % 总仿真时间 [s]
params.dt_kf = 0.1;          % 高频更新步长 [s]
params.ratio = 10;           % 高低频比例
params.dt_pf = params.dt_kf * params.ratio;  % 低频更新步长 [s]

% 计算时间步数
params.N_total = round(params.T/params.dt_kf) + 1;
params.N_pf = floor((params.N_total-1)/params.ratio) + 1;
params.time_kf = 0:params.dt_kf:params.T;
params.time_pf = 0:params.dt_pf:params.T;

%% 初始化标准差参数
params.init_pos_std = 50.0;   % 初始位置标准差 [m]
params.init_v_std = 2;     % 初始速度标准差 [m/s]
params.init_phi_std = 0.2;   % 初始航向标准差 [rad]

%% 模型参数
switch lower(model_type)
    case 'coupled'
        % 耦合模型参数
        params.k_d = 0.05;                    % 阻力系数
        params.T_r = 10;                    % 转向时间常数
        params.current_field.vortex_strength = 1;  % 涡旋强度
        params.current_field.vortex_center = [4845, 4845];  % 涡旋中心
        params.current_field.decay_radius = 1500.0;  % 衰减半径
        params.current_field.background_current = [0.2, 0.1];  % 背景水流
        params.control.throttle_constant = 0.05;     % 油门控制
        params.control.steering_amplitude = deg2rad(10);  % 转向幅度
        params.control.steering_period = 10.0;    % 转向周期

        % 水流切变耦合系数
        params.coupling.C_v = 0.02;           % 速度-散度耦合系数 [1/s]
        params.coupling.C_phi = 0.01;         % 航向-旋度耦合系数 [1/s]
        params.coupling.gradient_delta = 1.0; % 梯度计算的数值差分步长 [m]
        
    case 'decoupled'
        % 解耦模型参数
        params.k_d = 0.0;                     % 无阻力
        params.T_r = 10.0;                    % 保持转向时间常数
        params.current_field.vortex_strength = 0.0;  % 无涡旋
        params.current_field.vortex_center = [4845, 4845];
        params.current_field.decay_radius = 1500.0;
        params.current_field.background_current = [0.0, 0.0];  % 无背景水流
        params.control.throttle_constant = 0.0;      % 无控制输入
        params.control.steering_amplitude = 0.0;
        params.control.steering_period = 20.0;      
        % 解耦模式：无耦合效应
        params.coupling.C_v = 0.0;           % 无速度-散度耦合
        params.coupling.C_phi = 0.0;         % 无航向-旋度耦合
        params.coupling.gradient_delta = 1.0; % 梯度计算的数值差分步长 [m]
end


%% 初始状态
params.init_true_x = 3000;   % 真实初始x位置 [m]
params.init_true_y = 4000;   % 真实初始y位置 [m]
params.init_true_v = 10.0;   % 真实初始速度 [m/s]
params.init_true_phi = deg2rad(45);  % 真实初始航向 [rad]

params.init_est_x = 3030;    % 估计初始x位置 [m]
params.init_est_y = 4030;    % 估计初始y位置 [m]
params.init_est_v = 12;      % 估计初始速度 [m/s]
params.init_est_phi = deg2rad(45)+0.5;   % 估计初始航向 [rad]

%% 噪声参数
% 过程噪声
params.sigma_x = 0.5;        % x位置过程噪声标准差 [m]
params.sigma_y = 0.5;        % y位置过程噪声标准差 [m]
params.q_v = 0.1;            % 速度过程噪声标准差 [m/s]
params.q_phi = 0.05;         % 航向过程噪声标准差 [rad]

params.Q_pos = diag([params.sigma_x^2, params.sigma_y^2]);
params.Q_vel = diag([params.q_v^2, params.q_phi^2]);

% 测量噪声
params.r_v = 0.1;            % 速度测量噪声标准差 [m/s]
params.r_phi = 0.05;         % 航向测量噪声标准差 [rad]
params.R_imu = diag([params.r_v^2, params.r_phi^2]);
params.sigma_pos = 1.0;      % 位置测量噪声标准差 [m]
params.R_pos = diag([params.sigma_pos^2, params.sigma_pos^2]);

%% 算法特定参数
switch lower(filter_type)
    case 'generic'
        % 比较模式的通用参数 - 避免算法间基础参数不一致
        params.N_particles = 100;    % 通用粒子数量
        params.Ne_eff_threshold = params.N_particles / 2;  % 有效粒子数阈值
        params.depthGatingThreshold = 5.0;  % 深度测量门限值 [m]
        params.resampling_threshold = 0.5;  % 重采样阈值
        % 设置通用可视化参数
        params.Pure_showVisualization = true;
        params.PMF_showVisualization = true;
        params.Hybrid_showVisualization = true;
        params.Pure_saveVideo = false;
        params.PMF_saveVideo = false;
        params.Hybrid_saveVideo = false;
        % PMF相关参数
        params.Ngrid = 50;
        params.dx_pmf = 4;
        params.grid_range = 1000;
        params.saveGridData = false;
        % Hybrid相关参数
        params.N_fixed = 25;
        params.fixed_offsets = [-80, -40, 0, 40, 80];
        params.Nparticles_random = params.N_particles - params.N_fixed;
        % RBPF相关参数 - 确保与rbpf case完全一致
        params.visualization.show_real_time = true;
        params.visualization.save_video = false;
        
    case 'pure_pf'
        % 纯粒子滤波参数
        params.N_particles = 100;    % 粒子数量
        params.Ne_eff_threshold = params.N_particles / 2;  % 有效粒子数阈值
        params.depthGatingThreshold = 5.0;  % 深度测量门限值 [m]
        params.Pure_showVisualization = true;  % 显示实时可视化
        params.Pure_saveVideo = false;  % 不保存视频
        
    case 'pmf'
        % PMF参数
        params.Ngrid = 100;           % 网格数量
        params.dx_pmf = 1;          % 网格分辨率
        params.N_particles = 100;    % 粒子数量（用于混合部分）
        params.Ne_eff_threshold = params.N_particles / 2;
        params.depthGatingThreshold = 5.0;
        params.PMF_showVisualization = true;  % 显示实时可视化
        params.PMF_saveVideo = false;  % 不保存视频
        params.saveGridData = false;  % 不保存网格数据
        
    case 'rbpf'
        % Rao-Blackwellized粒子滤波参数
        params.N_particles = 100;
        params.Ne_eff_threshold = params.N_particles / 2;
        params.depthGatingThreshold = 5.0;
        params.resampling_threshold = 0.5;  % 重采样阈值 - 确保与generic一致
        params.visualization.show_real_time = true;  % 显示实时可视化
        params.visualization.save_video = false;  % 不保存视频
        
    case 'hybrid'
        % Hybrid特有参数
        params.N_particles = 100;    % 总粒子数量
        params.N_fixed = 25;        % 固定粒子数量
        params.fixed_offsets = [-80, -40, 0, 40, 80];  % 固定粒子偏移量
        params.Nparticles_random = params.N_particles - params.N_fixed;  % 随机粒子数量
        params.Ne_eff_threshold = params.N_particles / 2;
        params.depthGatingThreshold = 5.0;
        params.Hybrid_showVisualization = true;  % 显示实时可视化
        params.Hybrid_saveVideo = false;  % 不保存视频
        
    case 'hybrid2'
        % Hybrid2特有参数 (与hybrid相同的参数配置)
        params.N_particles = 100;    % 总粒子数量
        params.N_fixed = 25;        % 固定粒子数量
        params.fixed_offsets = [-80, -40, 0, 40, 80];  % 固定粒子偏移量
        params.Nparticles_random = params.N_particles - params.N_fixed;  % 随机粒子数量
        params.Ne_eff_threshold = params.N_particles / 2;
        params.depthGatingThreshold = 5.0;
        params.Hybrid_showVisualization = true;  % 显示实时可视化
        params.Hybrid_saveVideo = false;  % 不保存视频
        
    case 'hybrid3'
        % Hybrid3特有参数 (自适应切换策略)
        params.N_particles = 100;    % 总粒子数量
        params.N_fixed = 25;        % 固定粒子数量
        params.fixed_offsets = [-80, -40, 0, 40, 80];  % 固定粒子偏移量
        params.Nparticles_random = params.N_particles - params.N_fixed;  % 随机粒子数量
        params.Ne_eff_threshold = params.N_particles / 2;
        params.depthGatingThreshold = 5.0;
        params.ts = 50.0;            % 策略切换时间 (秒)
        params.Hybrid_showVisualization = true;  % 显示实时可视化
        params.Hybrid_saveVideo = false;  % 不保存视频
end

%% 地图参数
params.dx_map = 30;  % 地图分辨率（米/像素）

%% 可视化参数
params.visualization = struct();
params.visualization.show_particles = true;
params.visualization.show_current_field = true;
params.visualization.show_trajectory = true;
params.visualization.show_real_time = true;  % 默认开启实时显示
params.visualization.update_interval = 1;
params.visualization.save_video = false;
params.visualization.video_filename = 'simulation.mp4';
params.zoomSize = 400;  % 局部地图窗口大小（米）

% 为了兼容性，设置各算法特定的可视化参数
params.Pure_showVisualization = params.visualization.show_real_time;
params.PMF_showVisualization = params.visualization.show_real_time;
params.Hybrid_showVisualization = params.visualization.show_real_time;
params.Pure_saveVideo = params.visualization.save_video;
params.PMF_saveVideo = params.visualization.save_video;
params.Hybrid_saveVideo = params.visualization.save_video;

%% 性能评估参数
params.evaluation = struct();
params.evaluation.compute_rmse = true;
params.evaluation.save_results = false;
params.evaluation.result_filename = 'simulation_results.mat';

% 更新基本参数
if isfield(gui_params, 'sim_time')
    params.T = gui_params.sim_time;
    params.N_total = round(params.T/params.dt_kf) + 1;
    params.N_pf = floor((params.N_total-1)/params.ratio) + 1;
    params.time_kf = 0:params.dt_kf:params.T;
    params.time_pf = 0:params.dt_pf:params.T;
end
if isfield(gui_params, 'N_particles')
    params.N_particles = gui_params.N_particles;
    params.Ne_eff_threshold = params.N_particles / 2;
end
if isfield(gui_params, 'dt_kf')
    params.dt_kf = gui_params.dt_kf;
end
if isfield(gui_params, 'dt_pf')
    params.dt_pf = gui_params.dt_pf;
    params.ratio = round(params.dt_pf / params.dt_kf);
end

% 更新初始化不确定度参数
if isfield(gui_params, 'init_pos_std')
    params.init_pos_std = gui_params.init_pos_std;
end
if isfield(gui_params, 'init_v_std')
    params.init_v_std = gui_params.init_v_std;
end
if isfield(gui_params, 'init_phi_std')
    params.init_phi_std = gui_params.init_phi_std;
end

% 更新噪声参数
if isfield(gui_params, 'sigma_x')
    params.sigma_x = gui_params.sigma_x;
    params.Q_pos = diag([params.sigma_x^2, params.sigma_y^2]);
end
if isfield(gui_params, 'sigma_y')
    params.sigma_y = gui_params.sigma_y;
    params.Q_pos = diag([params.sigma_x^2, params.sigma_y^2]);
end
if isfield(gui_params, 'q_v')
    params.q_v = gui_params.q_v;
    params.Q_vel = diag([params.q_v^2, params.q_phi^2]);
end
if isfield(gui_params, 'q_phi')
    params.q_phi = gui_params.q_phi;
    params.Q_vel = diag([params.q_v^2, params.q_phi^2]);
end
if isfield(gui_params, 'r_v')
    params.r_v = gui_params.r_v;
    params.R_imu = diag([params.r_v^2, params.r_phi^2]);
end
if isfield(gui_params, 'r_phi')
    params.r_phi = gui_params.r_phi;
    params.R_imu = diag([params.r_v^2, params.r_phi^2]);
end
if isfield(gui_params, 'sigma_pos')
    params.sigma_pos = gui_params.sigma_pos;
    params.R_pos = diag([params.sigma_pos^2, params.sigma_pos^2]);
end

% 更新算法特定参数
if isfield(gui_params, 'Ngrid')
    params.Ngrid = gui_params.Ngrid;
end
if isfield(gui_params, 'dx_pmf')
    params.dx_pmf = gui_params.dx_pmf;
end
if isfield(gui_params, 'grid_range')
    params.grid_range = gui_params.grid_range;
end
if isfield(gui_params, 'N_fixed')
    params.N_fixed = gui_params.N_fixed;
    params.Nparticles_random = params.N_particles - params.N_fixed;
end
if isfield(gui_params, 'N_random')
    params.N_random = gui_params.N_random;
    params.Nparticles_random = gui_params.N_random;
end
if isfield(gui_params, 'fixed_offsets')
    params.fixed_offsets = gui_params.fixed_offsets;
end
if isfield(gui_params, 'depthGatingThreshold')
    params.depthGatingThreshold = gui_params.depthGatingThreshold;
end
if isfield(gui_params, 'resampling_threshold')
    params.resampling_threshold = gui_params.resampling_threshold;
    params.Ne_eff_threshold = params.N_particles * gui_params.resampling_threshold;
end

% 更新可视化参数
if isfield(gui_params, 'visualization_enabled')
    params.visualization.show_real_time = gui_params.visualization_enabled;
    params.Pure_showVisualization = gui_params.Pure_showVisualization;
    params.PMF_showVisualization = gui_params.PMF_showVisualization;
    params.Hybrid_showVisualization = gui_params.Hybrid_showVisualization;
end

end 