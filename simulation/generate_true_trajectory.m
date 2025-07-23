function [true_data, meas_data] = generate_true_trajectory(params)
%GENERATE_TRUE_TRAJECTORY 生成包含水流影响的真实轨迹和测量数据
%   根据新模型的状态演化方程生成船舶真实轨迹和带噪声的传感器测量
%
%   输入:
%   - params: 仿真参数结构体
%
%   输出:
%   - true_data: 真实轨迹数据结构体
%       .x, .y: 位置轨迹 [m]
%       .v, .phi: 速度和航向轨迹 [m/s, rad]
%       .u_c, .v_c: 水流速度轨迹 [m/s]
%   - meas_data: 测量数据结构体
%       .v_imu, .phi_imu: 高频IMU测量 [m/s, rad]
%       .x_pos, .y_pos: 低频位置测量 [m]

%% 初始化数组
N_total = params.N_total;
N_pf = params.N_pf;

% 真实状态轨迹
true_x = zeros(1, N_total);
true_y = zeros(1, N_total);
true_v = zeros(1, N_total);
true_phi = zeros(1, N_total);

% 水流速度轨迹
true_u_c = zeros(1, N_total);
true_v_c = zeros(1, N_total);

% 控制输入轨迹
control_a_c = zeros(1, N_total);
control_delta_r = zeros(1, N_total);

% 测量数据
meas_v_imu = zeros(1, N_total);
meas_phi_imu = zeros(1, N_total);
meas_x_pos = zeros(1, N_pf);
meas_y_pos = zeros(1, N_pf);

%% 设置初始条件
true_x(1) = params.init_true_x;
true_y(1) = params.init_true_y;
true_v(1) = params.init_true_v;
true_phi(1) = params.init_true_phi;

% 计算初始位置的水流
[true_u_c(1), true_v_c(1)] = simulate_current_field(true_x(1), true_y(1), params);

% 初始测量
meas_v_imu(1) = true_v(1) + params.r_v * randn();
meas_phi_imu(1) = true_phi(1) + params.r_phi * randn();
meas_x_pos(1) = true_x(1) + params.sigma_pos * randn();
meas_y_pos(1) = true_y(1) + params.sigma_pos * randn();

fprintf('开始生成真实轨迹...\n');

%% 主循环：状态演化
for k = 2:N_total
    time_current = params.time_kf(k);
    
    %% 1. 生成控制输入
    % 恒定油门 + 周期性转向策略
    control_a_c(k) = params.control.throttle_constant;
    control_delta_r(k) = params.control.steering_amplitude * ...
                         sin(2*pi * time_current / params.control.steering_period);
    
    %% 2. 条件线性状态演化 (速度和航向)
    % 基础动力学演化
    % 速度演化: v_k = v_{k-1} + (a_c - k_d * v_{k-1}) * dt + w_v
    v_dynamics = control_a_c(k) - params.k_d * true_v(k-1);
    
    % 航向演化: φ_k = φ_{k-1} + (1/T_r * δ_r) * dt + w_φ
    phi_dynamics = control_delta_r(k) / params.T_r;
    
    % 深度耦合模型：加入水流切变效应
    if isfield(params, 'coupling') && (params.coupling.C_v ~= 0 || params.coupling.C_phi ~= 0)
        % 计算当前位置的水流场梯度（散度和旋度）
        [~, ~, divergence, vorticity] = simulate_current_field(true_x(k-1), true_y(k-1), params);
        
        % 速度切变效应：散度影响 (a_v_shear = C_v * divergence)
        v_shear_effect = params.coupling.C_v * divergence;
        
        % 航向切变效应：旋度影响 (a_phi_shear = C_phi * vorticity)  
        phi_shear_effect = params.coupling.C_phi * vorticity;
        
        % 将切变效应加入动力学方程
        v_dynamics = v_dynamics + v_shear_effect;
        phi_dynamics = phi_dynamics + phi_shear_effect;
    end
    
    % 状态更新
    true_v(k) = true_v(k-1) + v_dynamics * params.dt_kf + params.q_v * randn();
    true_phi(k) = true_phi(k-1) + phi_dynamics * params.dt_kf + params.q_phi * randn();
    
    % 航向角归一化到 [-π, π]
    true_phi(k) = wrapToPi(true_phi(k));
    
    %% 3. 计算当前位置的水流
    [true_u_c(k-1), true_v_c(k-1)] = simulate_current_field(true_x(k-1), true_y(k-1), params);
    
    %% 4. 非线性状态演化 (位置)
    % 位置演化: 船只对水速度 + 水流速度
    % x_k = x_{k-1} + (v_{k-1}*cos(φ_{k-1}) + u_c(x_{k-1}, y_{k-1})) * dt + w_x
    x_velocity = true_v(k-1) * cos(true_phi(k-1)) + true_u_c(k-1);
    true_x(k) = true_x(k-1) + x_velocity * params.dt_kf + params.sigma_x * randn();
    
    % y_k = y_{k-1} + (v_{k-1}*sin(φ_{k-1}) + v_c(x_{k-1}, y_{k-1})) * dt + w_y
    y_velocity = true_v(k-1) * sin(true_phi(k-1)) + true_v_c(k-1);
    true_y(k) = true_y(k-1) + y_velocity * params.dt_kf + params.sigma_y * randn();
    
    %% 5. 生成高频IMU测量 (每个时间步)
    meas_v_imu(k) = true_v(k) + params.r_v * randn();
    meas_phi_imu(k) = true_phi(k) + params.r_phi * randn();
    
    %% 6. 生成低频位置测量 (按ratio间隔)
    if mod(k-1, params.ratio) == 0
        pf_idx = floor((k-1)/params.ratio) + 1;
        if pf_idx <= N_pf
            meas_x_pos(pf_idx) = true_x(k) + params.sigma_pos * randn();
            meas_y_pos(pf_idx) = true_y(k) + params.sigma_pos * randn();
        end
    end
    
    % 进度显示
    if mod(k, 100) == 0
        fprintf('  生成进度: %d/%d (%.1f%%)\n', k, N_total, k/N_total*100);
    end
end

% 计算最后一个时间步的水流
[true_u_c(end), true_v_c(end)] = simulate_current_field(true_x(end), true_y(end), params);

%% 整理输出数据结构
true_data = struct();
true_data.time = params.time_kf;
true_data.x = true_x;
true_data.y = true_y;
true_data.v = true_v;
true_data.phi = true_phi;
true_data.u_c = true_u_c;
true_data.v_c = true_v_c;
true_data.control_a_c = control_a_c;
true_data.control_delta_r = control_delta_r;

meas_data = struct();
meas_data.v_imu = meas_v_imu;
meas_data.phi_imu = meas_phi_imu;
meas_data.x_pos = meas_x_pos;
meas_data.y_pos = meas_y_pos;

%% 计算轨迹统计信息
trajectory_length = sum(sqrt(diff(true_x).^2 + diff(true_y).^2));
x_range = [min(true_x), max(true_x)];
y_range = [min(true_y), max(true_y)];
v_range = [min(true_v), max(true_v)];
phi_range = [min(true_phi), max(true_phi)];

fprintf('真实轨迹生成完成!\n');
fprintf('轨迹统计信息:\n');
fprintf('  - 轨迹总长度: %.2f m\n', trajectory_length);
fprintf('  - X坐标范围: [%.2f, %.2f] m\n', x_range);
fprintf('  - Y坐标范围: [%.2f, %.2f] m\n', y_range);
fprintf('  - 速度范围: [%.2f, %.2f] m/s\n', v_range);
fprintf('  - 航向范围: [%.2f, %.2f] deg\n', rad2deg(phi_range));
fprintf('  - 高频测量点数: %d\n', N_total);
fprintf('  - 低频测量点数: %d\n', N_pf);

end 