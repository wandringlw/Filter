function detailed_analysis(params, true_data, est_data_input, meas_data_input, algorithm_names)
%DETAILED_ANALYSIS 详细分析 - 生成6图误差分析
%   支持单算法和多算法的结果对比显示
%
%   输入:
%   - params: 仿真参数结构体
%   - true_data: 真实轨迹数据
%   - est_data_input: 估计数据 (单算法：结构体; 多算法：cell数组)
%   - meas_data_input: 测量数据 (单算法：结构体; 多算法：cell数组)
%   - algorithm_names: 算法名称 (可选，多算法时必需)

%% 检测输入模式并准备数据
if nargin < 5
    algorithm_names = {'算法'};
end

% 处理输入数据，统一转换为cell数组格式
if iscell(est_data_input)
    % 多算法模式
    est_data_array = est_data_input;
    meas_data_array = meas_data_input;
    n_algorithms = length(est_data_array);
    if length(algorithm_names) ~= n_algorithms
        for i = 1:n_algorithms
            algorithm_names{i} = sprintf('算法%d', i);
        end
    end
else
    % 单算法模式，转换为cell数组格式
    est_data_array = {est_data_input};
    meas_data_array = {meas_data_input};
    n_algorithms = 1;
    algorithm_names = algorithm_names(1);
end

% 创建颜色和线型配置
[colors, line_styles] = get_plot_styles(n_algorithms);

%% 创建结果可视化图
if isscalar(algorithm_names)
    fig_title = sprintf('滤波算法结果分析 - %s', algorithm_names{1});
else
    fig_title = sprintf('滤波算法结果对比 (%d个算法)', n_algorithms);
end
figure('Name', fig_title, 'Position', [200, 200, 1400, 1000]);

%% 子图1: 轨迹对比
subplot(3, 3, 1);
hold on;

% 真实轨迹 (黑色)
plot(true_data.x, true_data.y, 'k-', 'LineWidth', 2, 'DisplayName', '真实轨迹');

% 所有算法的估计轨迹
for i = 1:n_algorithms
    est_data = est_data_array{i};
    plot(est_data.x, est_data.y, 'Color', colors{i}, 'LineStyle', line_styles{i}, ...
         'LineWidth', 1.8, 'DisplayName', algorithm_names{i});
end

% 起始点和终点标记
plot(true_data.x(1), true_data.y(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起始点');
plot(true_data.x(end), true_data.y(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

xlabel('X (m)'); ylabel('Y (m)');
if isscalar(algorithm_names)
    title('轨迹对比');
else
    title(sprintf('轨迹对比 (%d个算法)', n_algorithms));
end
legend('Location', 'best', 'NumColumns', min(3, ceil((n_algorithms+3)/2)));
grid on; axis equal;

%% 子图2: X位置误差
subplot(3, 3, 2);
hold on;

% 对齐时间序列并绘制所有算法的X误差
true_indices = 1:params.ratio:length(true_data.x);
rmse_x_all = zeros(1, n_algorithms);

for i = 1:n_algorithms
    est_data = est_data_array{i};
    if length(est_data.time) <= length(true_data.x)
        n_points = min(length(est_data.time), length(true_indices));
        
        true_x_aligned = true_data.x(true_indices(1:n_points));
        est_x_aligned = est_data.x(1:n_points);
        time_aligned = est_data.time(1:n_points);
        
        x_error = est_x_aligned - true_x_aligned;
        rmse_x_all(i) = sqrt(mean(x_error.^2));
        
        plot(time_aligned, x_error, 'Color', colors{i}, 'LineStyle', line_styles{i}, ...
             'LineWidth', 1.5, 'DisplayName', sprintf('%s (RMSE: %.2fm)', algorithm_names{i}, rmse_x_all(i)));
    end
end

xlabel('时间 (s)'); ylabel('X误差 (m)');
if isscalar(algorithm_names)
    title('X位置估计误差');
else
    title('X位置估计误差对比');
end
grid on;
legend('Location', 'best', 'FontSize', 8);
hold off;

%% 子图3: Y位置误差
subplot(3, 3, 3);
hold on;

% 对齐时间序列并绘制所有算法的Y误差
rmse_y_all = zeros(1, n_algorithms);

for i = 1:n_algorithms
    est_data = est_data_array{i};
    if length(est_data.time) <= length(true_data.y)
        n_points = min(length(est_data.time), length(true_indices));
        
        true_y_aligned = true_data.y(true_indices(1:n_points));
        est_y_aligned = est_data.y(1:n_points);
        time_aligned = est_data.time(1:n_points);
        
        y_error = est_y_aligned - true_y_aligned;
        rmse_y_all(i) = sqrt(mean(y_error.^2));
        
        plot(time_aligned, y_error, 'Color', colors{i}, 'LineStyle', line_styles{i}, ...
             'LineWidth', 1.5, 'DisplayName', sprintf('%s (RMSE: %.2fm)', algorithm_names{i}, rmse_y_all(i)));
    end
end

xlabel('时间 (s)'); ylabel('Y误差 (m)');
if isscalar(algorithm_names)
    title('Y位置估计误差');
else
    title('Y位置估计误差对比');
end
grid on;
legend('Location', 'best', 'FontSize', 8);
hold off;

%% 子图4: 速度误差分析
subplot(3, 3, 4);
hold on;

% 检查是否有速度数据
if isfield(true_data, 'v')
    rmse_v_est_all = zeros(1, n_algorithms);
    
    % 绘制所有算法的速度估计误差
    for i = 1:n_algorithms
        est_data = est_data_array{i};
        if isfield(est_data, 'v') && length(est_data.time) <= length(true_data.v)
            n_points = min(length(est_data.time), length(true_indices));
            
            true_v_aligned = true_data.v(true_indices(1:n_points));
            est_v_aligned = est_data.v(1:n_points);
            time_aligned = est_data.time(1:n_points);
            v_est_error = est_v_aligned - true_v_aligned;
            
            rmse_v_est_all(i) = sqrt(mean(v_est_error.^2));
            
            plot(time_aligned, v_est_error, 'Color', colors{i}, 'LineStyle', line_styles{i}, ...
                 'LineWidth', 1.5, 'DisplayName', sprintf('%s (RMSE: %.3fm/s)', algorithm_names{i}, rmse_v_est_all(i)));
        end
    end
    
    % 添加IMU测量误差作为参考（使用第一个算法的测量数据）
    meas_data = meas_data_array{1};
    if isfield(meas_data, 'v_imu') && length(meas_data.v_imu) <= length(true_data.v)
        true_v_imu_aligned = true_data.v(1:length(meas_data.v_imu));
        v_meas_error = meas_data.v_imu(1:length(true_v_imu_aligned)) - true_v_imu_aligned;
        time_imu = true_data.time(1:length(v_meas_error));
        
        rmse_v_meas = sqrt(mean(v_meas_error.^2));
        plot(time_imu, v_meas_error, 'k:', 'LineWidth', 1.2, ...
             'DisplayName', sprintf('IMU测量误差 (RMSE: %.3fm/s)', rmse_v_meas));
    end
    
    xlabel('时间 (s)'); ylabel('速度误差 (m/s)');
    if isscalar(algorithm_names)
        title('速度误差分析');
    else
        title('速度误差分析对比');
    end
    legend('Location', 'best', 'FontSize', 8);
    grid on;
else
    text(0.5, 0.5, '无速度数据', 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontSize', 12);
    title('速度误差分析');
end
hold off;

%% 子图5: 航向误差分析
subplot(3, 3, 5);
hold on;

% 检查是否有航向数据
if isfield(true_data, 'phi')
    rmse_phi_est_all = zeros(1, n_algorithms);
    
    % 绘制所有算法的航向估计误差
    for i = 1:n_algorithms
        est_data = est_data_array{i};
        if isfield(est_data, 'phi') && length(est_data.time) <= length(true_data.phi)
            n_points = min(length(est_data.time), length(true_indices));
            
            true_phi_aligned = true_data.phi(true_indices(1:n_points));
            est_phi_aligned = est_data.phi(1:n_points);
            time_aligned = est_data.time(1:n_points);
            
            % 航向误差需要考虑角度周期性
            phi_est_error = wrapToPi(est_phi_aligned - true_phi_aligned);
            phi_est_error_deg = rad2deg(phi_est_error);
            rmse_phi_est_all(i) = sqrt(mean(phi_est_error.^2));
            
            plot(time_aligned, phi_est_error_deg, 'Color', colors{i}, 'LineStyle', line_styles{i}, ...
                 'LineWidth', 1.5, 'DisplayName', sprintf('%s (RMSE: %.2f°)', algorithm_names{i}, rad2deg(rmse_phi_est_all(i))));
        end
    end
    
    % 添加IMU测量误差作为参考（使用第一个算法的测量数据）
    meas_data = meas_data_array{1};
    if isfield(meas_data, 'phi_imu') && length(meas_data.phi_imu) <= length(true_data.phi)
        % IMU数据时间对齐
        n_imu = length(meas_data.phi_imu);
        
        if isfield(params, 'dt_kf') && params.dt_kf > 0
            imu_indices = 1:min(n_imu, length(true_data.phi));
        else
            imu_indices = 1:round(length(true_data.phi)/n_imu):length(true_data.phi);
            imu_indices = imu_indices(1:min(n_imu, length(imu_indices)));
        end
        
        if ~isempty(imu_indices)
            true_phi_imu_aligned = true_data.phi(imu_indices);
            time_imu = true_data.time(imu_indices);
            
            phi_meas_error = wrapToPi(meas_data.phi_imu - true_phi_imu_aligned);
            phi_meas_error_deg = rad2deg(phi_meas_error);
            rmse_phi_meas = sqrt(mean(phi_meas_error.^2));
            
            plot(time_imu, phi_meas_error_deg, 'k:', 'LineWidth', 1.2, ...
                 'DisplayName', sprintf('IMU测量误差 (RMSE: %.2f°)', rad2deg(rmse_phi_meas)));
        end
    end
    
    xlabel('时间 (s)'); ylabel('航向误差 (度)');
    if isscalar(algorithm_names)
        title('航向误差分析');
    else
        title('航向误差分析对比');
    end
    legend('Location', 'best', 'FontSize', 8);
    grid on;
else
    text(0.5, 0.5, '无航向数据', 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontSize', 12);
    title('航向误差分析');
end
hold off;

%% 子图6: 总位置误差
subplot(3, 3, 6);
hold on;

% 重新计算每个算法的总位置误差
rmse_pos_all = zeros(1, n_algorithms);

for i = 1:n_algorithms
    est_data = est_data_array{i};
    if length(est_data.time) <= length(true_data.x)
        n_points = min(length(est_data.time), length(true_indices));
        
        true_x_aligned = true_data.x(true_indices(1:n_points));
        true_y_aligned = true_data.y(true_indices(1:n_points));
        est_x_aligned = est_data.x(1:n_points);
        est_y_aligned = est_data.y(1:n_points);
        time_aligned = est_data.time(1:n_points);
        
        % 计算总位置误差
        position_error = sqrt((est_x_aligned - true_x_aligned).^2 + (est_y_aligned - true_y_aligned).^2);
        rmse_pos_all(i) = sqrt(mean(position_error.^2));
        
        plot(time_aligned, position_error, 'Color', colors{i}, 'LineStyle', line_styles{i}, ...
             'LineWidth', 1.8, 'DisplayName', sprintf('%s (RMSE: %.2fm)', algorithm_names{i}, rmse_pos_all(i)));
    end
end

xlabel('时间 (s)'); ylabel('位置误差 (m)');
if isscalar(algorithm_names)
    title('总位置误差');
else
    title('总位置误差对比');
end
legend('Location', 'best', 'FontSize', 8);
grid on;
hold off;

%% 子图7: 粒子权重方差（置信度指标）
subplot(3, 3, 7);
hold on;

% 检查哪些算法支持权重方差
algorithms_with_variance = cell(1, n_algorithms);  % 预分配
variance_count = 0;

for i = 1:n_algorithms
    est_data = est_data_array{i};
    if isfield(est_data, 'weight_variance')
        variance_count = variance_count + 1;
        algorithms_with_variance{variance_count} = algorithm_names{i};
        
        plot(est_data.time, est_data.weight_variance, 'Color', colors{i}, 'LineStyle', line_styles{i}, ...
             'LineWidth', 1.8, 'DisplayName', algorithm_names{i});
    end
end

if variance_count > 0
    xlabel('时间 (s)'); ylabel('权重方差');
    if variance_count == 1
        title('粒子权重方差（置信度指标）');
    else
        title('粒子权重方差对比（置信度指标）');
    end
    legend('Location', 'best', 'FontSize', 8);
    grid on;
    
    % 添加说明文本
    text(0.02, 0.98, {'高方差 = 高置信度'; '低方差 = 低置信度'}, ...
         'Units', 'normalized', 'VerticalAlignment', 'top', ...
         'FontSize', 9, 'BackgroundColor', 'white');
else
    text(0.5, 0.5, {'所选算法均不支持', '权重方差计算'}, 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontSize', 12);
    title('粒子权重方差（置信度指标）');
end
hold off;

%% 子图8: 空置
subplot(3, 3, 8);
axis off;

%% 子图9: 空置  
subplot(3, 3, 9);
axis off;

%% 调整布局
if isscalar(algorithm_names)
    sgtitle(sprintf('滤波算法性能评估 - %s', algorithm_names{1}), 'FontSize', 16, 'FontWeight', 'bold');
else
    sgtitle(sprintf('滤波算法性能对比 (%d个算法)', n_algorithms), 'FontSize', 16, 'FontWeight', 'bold');
end

%% 打印性能摘要到命令行
fprintf('\n==============================================\n');
if isscalar(algorithm_names)
    fprintf('            性能评估摘要 - %s\n', algorithm_names{1});
else
    fprintf('            多算法性能对比摘要\n');
end
fprintf('==============================================\n');

% 打印每个算法的性能
for i = 1:n_algorithms
    fprintf('\n算法: %s\n', algorithm_names{i});
    fprintf('----------------\n');
    
    % 位置RMSE
    if exist('rmse_x_all', 'var') && i <= length(rmse_x_all)
        fprintf('位置RMSE:\n');
        fprintf('  - X方向: %.3f m\n', rmse_x_all(i));
    end
    if exist('rmse_y_all', 'var') && i <= length(rmse_y_all)
        fprintf('  - Y方向: %.3f m\n', rmse_y_all(i));
    end
    if exist('rmse_pos_all', 'var') && i <= length(rmse_pos_all)
        fprintf('  - 总体: %.3f m\n', rmse_pos_all(i));
    end
    
    % 速度RMSE
    if exist('rmse_v_est_all', 'var') && i <= length(rmse_v_est_all) && rmse_v_est_all(i) > 0
        fprintf('速度RMSE: %.3f m/s\n', rmse_v_est_all(i));
    end
    
    % 航向RMSE
    if exist('rmse_phi_est_all', 'var') && i <= length(rmse_phi_est_all) && rmse_phi_est_all(i) > 0
        fprintf('航向RMSE: %.3f度 (%.4f弧度)\n', rad2deg(rmse_phi_est_all(i)), rmse_phi_est_all(i));
    end
end

% 计算轨迹覆盖范围
x_range = max(true_data.x) - min(true_data.x);
y_range = max(true_data.y) - min(true_data.y);
fprintf('\n轨迹信息:\n');
fprintf('  - X方向跨度: %.2f m\n', x_range);
fprintf('  - Y方向跨度: %.2f m\n', y_range);
fprintf('  - 仿真时长: %.1f s\n', max(true_data.time) - min(true_data.time));

fprintf('\n==============================================\n');

end

%% 辅助函数

function [colors, line_styles] = get_plot_styles(n_algorithms)
%GET_PLOT_STYLES 获取绘图颜色和线型样式
%   为多算法对比生成不同的颜色和线型组合
%
%   输入:
%   - n_algorithms: 算法数量
%
%   输出:
%   - colors: 颜色cell数组
%   - line_styles: 线型cell数组

    % 定义基础颜色（RGB格式，便于视觉区分）
    base_colors = {
        [0.8, 0.2, 0.2];  % 红色
        [0.2, 0.4, 0.8];  % 蓝色  
        [0.2, 0.7, 0.3];  % 绿色
        [0.9, 0.5, 0.1];  % 橙色
        [0.6, 0.2, 0.8];  % 紫色
        [0.8, 0.6, 0.2];  % 金色
        [0.3, 0.7, 0.7];  % 青色
        [0.8, 0.3, 0.6]   % 洋红
    };
    
    % 定义线型样式
    base_line_styles = {'-', '--', ':', '-.'};
    
    % 为每个算法分配颜色和线型
    colors = cell(1, n_algorithms);
    line_styles = cell(1, n_algorithms);
    
    for i = 1:n_algorithms
        % 循环使用颜色
        color_idx = mod(i-1, length(base_colors)) + 1;
        colors{i} = base_colors{color_idx};
        
        % 循环使用线型
        style_idx = mod(i-1, length(base_line_styles)) + 1;
        line_styles{i} = base_line_styles{style_idx};
    end
end 