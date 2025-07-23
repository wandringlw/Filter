function results_dashboard(all_results, base_params, mc_runs)
    % 结果仪表板 - 负责MC统计汇总和界面管理
    % 输入:
    %   all_results - 所有算法的结果
    %   base_params - 基础参数配置
    %   mc_runs - Monte Carlo运行次数
    
    try
        % 根据结果中算法数量自动判断模式
        algo_names = fieldnames(all_results);
        
        if isscalar(algo_names)
            % 单算法模式：显示详细性能分析
            create_single_algorithm_visualization(all_results, base_params);
        else
            % 多算法比较模式：创建对比图表
            create_comparison_visualization(all_results, base_params, mc_runs);
        end
        
    catch ME
        error_msg = sprintf('可视化生成失败: %s\n位置: %s', ME.message, ME.stack(1).name);
        errordlg(error_msg, '可视化错误');
    end
end

function create_single_algorithm_visualization(all_results, base_params)
    % 单算法模式可视化
    algo_names = fieldnames(all_results);
    algo = algo_names{1};
    last_result = all_results.(algo).results{end};
    
    est_data = last_result.est_data;
    last_true_data = last_result.true_data;
    last_meas_data = last_result.meas_data;
    
    % 调用详细的性能分析可视化
    detailed_analysis(base_params, last_true_data, est_data, last_meas_data);
end

function create_comparison_visualization(all_results, base_params, mc_runs)
    % 算法比较模式可视化
    
    % 计算统计量
    [mean_rmse_pos, std_rmse_pos, mean_exec_time, std_exec_time, display_names] = ...
        calculate_comparison_statistics(all_results);
    
    % 创建主对比窗口
    fig = figure('Name', '算法性能对比', ...
               'NumberTitle', 'off', ...
               'Position', [100 100 800 600]);
    
    % 创建标签页组
    tabgp = uitabgroup(fig);
    
    % 创建各个标签页
    create_statistics_tab(tabgp, mean_rmse_pos, std_rmse_pos, mean_exec_time, std_exec_time, display_names);
    create_trajectory_comparison_tab(tabgp, all_results);
    create_rmse_timeline_tab(tabgp, all_results, base_params, mc_runs);
    create_detailed_analysis_tab(tabgp, all_results, base_params, mc_runs);
end

function [mean_rmse_pos, std_rmse_pos, mean_exec_time, std_exec_time, display_names] = ...
         calculate_comparison_statistics(all_results)
    % 计算比较统计量
    
    algo_names = fieldnames(all_results);
    n_algos = length(algo_names);
    
    mean_rmse_pos = zeros(1, n_algos);
    std_rmse_pos = zeros(1, n_algos);
    mean_exec_time = zeros(1, n_algos);
    std_exec_time = zeros(1, n_algos);
    display_names = cell(1, n_algos);
    
    for i = 1:n_algos
        algo = algo_names{i};
        results = all_results.(algo).results;
        display_names{i} = all_results.(algo).display_name;
        
        % 提取RMSE和执行时间
        rmse_values = cellfun(@(x) x.rmse_pos, results);
        exec_times = cellfun(@(x) x.exec_time, results);
        
        % 计算统计量
        mean_rmse_pos(i) = mean(rmse_values);
        std_rmse_pos(i) = std(rmse_values);
        mean_exec_time(i) = mean(exec_times);
        std_exec_time(i) = std(exec_times);
    end
end

function create_statistics_tab(tabgp, mean_rmse_pos, std_rmse_pos, mean_exec_time, std_exec_time, display_names)
    % 创建统计结果标签页
    tab1 = uitab(tabgp, 'Title', '性能统计');
    
    % RMSE对比子图
    subplot(2,1,1, 'Parent', tab1);
    bar(mean_rmse_pos);
    hold on;
    errorbar(1:length(display_names), mean_rmse_pos, std_rmse_pos, 'k.', 'LineWidth', 1.5);
    set(gca, 'XTickLabel', display_names);
    title('位置RMSE对比');
    ylabel('RMSE (m)');
    grid on;
    
    % 为每个柱子添加数值标签
    for i = 1:length(mean_rmse_pos)
        text(i, mean_rmse_pos(i) + std_rmse_pos(i) + 0.5, ...
             sprintf('%.2f±%.2f', mean_rmse_pos(i), std_rmse_pos(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 8);
    end
    
    % 执行时间对比子图
    subplot(2,1,2, 'Parent', tab1);
    bar(mean_exec_time);
    hold on;
    errorbar(1:length(display_names), mean_exec_time, std_exec_time, 'k.', 'LineWidth', 1.5);
    set(gca, 'XTickLabel', display_names);
    title('执行时间对比');
    ylabel('时间 (s)');
    grid on;
    
    % 为每个柱子添加数值标签
    for i = 1:length(mean_exec_time)
        text(i, mean_exec_time(i) + std_exec_time(i) + 0.01, ...
             sprintf('%.3f±%.3f', mean_exec_time(i), std_exec_time(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 8);
    end
end

function create_trajectory_comparison_tab(tabgp, all_results)
    % 创建轨迹比较标签页
    tab2 = uitab(tabgp, 'Title', '轨迹比较');
    axes('Parent', tab2);
    hold on;
    
    algo_names = fieldnames(all_results);
    
    % 使用最后一次MC的真实轨迹
    last_true_data = all_results.(algo_names{1}).results{end}.true_data;
    plot(last_true_data.x, last_true_data.y, 'k-', 'LineWidth', 2, 'DisplayName', '真实轨迹');
    
    % 绘制各算法的估计轨迹
    colors = get_algorithm_colors(length(algo_names));
    for i = 1:length(algo_names)
        algo = algo_names{i};
        est_data = all_results.(algo).results{end}.est_data;
        plot(est_data.x, est_data.y, [colors{i} '--'], ...
             'LineWidth', 1.5, ...
             'DisplayName', all_results.(algo).display_name);
    end
    
    hold off;
    grid on;
    legend('show', 'Location', 'best');
    title('轨迹比较 (最后一次MC运行)');
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal;
end

function create_rmse_timeline_tab(tabgp, all_results, base_params, mc_runs)
    % 创建RMSE时间序列标签页
    tab3 = uitab(tabgp, 'Title', 'RMSE时间序列');
    axes('Parent', tab3);
    hold on;
    
    algo_names = fieldnames(all_results);
    colors = get_algorithm_colors(length(algo_names));
    
    if mc_runs == 1
        % MC=1时：显示单次运行结果
        create_single_run_rmse_plot(all_results, algo_names, colors, base_params);
        title('位置RMSE随时间变化');
    else
        % MC≥2时：显示多次MC平均结果
        create_multi_run_rmse_plot(all_results, algo_names, colors, base_params, mc_runs);
        title(sprintf('位置RMSE随时间变化 (MC=%d次平均)', mc_runs));
    end
    
    hold off;
    grid on;
    legend('show', 'Location', 'best');
    xlabel('时间 (s)');
    ylabel('位置误差 (m)');
end

function create_single_run_rmse_plot(all_results, algo_names, colors, base_params)
    % 创建单次运行RMSE图
    for i = 1:length(algo_names)
        algo = algo_names{i};
        result = all_results.(algo).results{1};
        time = result.time;
        est_data = result.est_data;
        mc_true_data = result.true_data;
        true_indices = 1:base_params.ratio:length(mc_true_data.x);
        
        % 计算每个时间点的RMSE
        pos_error = sqrt((est_data.x - mc_true_data.x(true_indices)).^2 + ...
                       (est_data.y - mc_true_data.y(true_indices)).^2);
        
        plot(time, pos_error, colors{i}, ...
             'LineWidth', 1.5, ...
             'DisplayName', all_results.(algo).display_name);
    end
end

function create_multi_run_rmse_plot(all_results, algo_names, colors, base_params, mc_runs)
    % 创建多次运行平均RMSE图
    for i = 1:length(algo_names)
        algo = algo_names{i};
        results = all_results.(algo).results;
        
        % 计算所有MC运行的平均RMSE时间序列
        time = results{1}.time;
        % 预分配数据容器以提高性能
        est_data_sample = results{1}.est_data;
        num_time_steps = length(est_data_sample.x);
        all_pos_errors = zeros(mc_runs, num_time_steps);
        
        for mc = 1:mc_runs
            est_data = results{mc}.est_data;
            mc_true_data = results{mc}.true_data;
            true_indices = 1:base_params.ratio:length(mc_true_data.x);
            
            pos_error = sqrt((est_data.x - mc_true_data.x(true_indices)).^2 + ...
                           (est_data.y - mc_true_data.y(true_indices)).^2);
            all_pos_errors(mc, :) = pos_error;
        end
        
        % 计算平均和标准差
        mean_pos_error = mean(all_pos_errors, 1);
        std_pos_error = std(all_pos_errors, 0, 1);
        
        % 绘制均值曲线和标准差带
        fill([time, fliplr(time)], ...
             [mean_pos_error - std_pos_error, fliplr(mean_pos_error + std_pos_error)], ...
             colors{i}, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(time, mean_pos_error, colors{i}, ...
             'LineWidth', 2, ...
             'DisplayName', sprintf('%s (MC平均)', all_results.(algo).display_name));
    end
end

function create_detailed_analysis_tab(tabgp, all_results, base_params, mc_runs)
    % 创建详细分析标签页
    tab4 = uitab(tabgp, 'Title', '详细分析');
    
    % 获取数据用于详细分析
    algo_names = fieldnames(all_results);
    first_algo = algo_names{1};
    last_true_data = all_results.(first_algo).results{end}.true_data;
    last_meas_data = all_results.(first_algo).results{end}.meas_data;
    
    create_detailed_analysis_content(tab4, all_results, last_true_data, last_meas_data, base_params, mc_runs);
end

function create_detailed_analysis_content(parent_tab, all_results, true_data, meas_data, params, mc_runs)
    % 创建详细分析内容
    
    try
        if mc_runs == 1
            % MC=1时：提供详细分析按钮
            create_single_mc_analysis_content(parent_tab, all_results, true_data, meas_data, params);
        else
            % MC≥2时：显示统计性能对比
            create_multi_mc_analysis_content(parent_tab, all_results, mc_runs);
        end
    catch ME
        % 如果详细分析创建失败，显示错误信息
        uicontrol('Parent', parent_tab, ...
                 'Style', 'text', ...
                 'String', sprintf('详细分析创建失败: %s', ME.message), ...
                 'Position', [20 400 500 50], ...
                 'FontSize', 10, 'BackgroundColor', [1 0.8 0.8], ...
                 'HorizontalAlignment', 'left');
    end
end

function create_single_mc_analysis_content(parent_tab, all_results, ~, ~, ~)
    % 创建单次MC分析内容
    algo_names = fieldnames(all_results);
    n_algorithms = length(algo_names);
    
    if n_algorithms == 1
        button_text = 'MC=1时的详细分析将在独立窗口中显示';
        description_text = '点击下面的按钮查看详细的7图分析';
    else
        button_text = sprintf('多算法(%d个)详细对比分析', n_algorithms);
        description_text = '点击下面的按钮查看详细的多算法对比分析';
    end
    
    uicontrol('Parent', parent_tab, ...
             'Style', 'text', ...
             'String', {button_text, '', description_text}, ...
             'Position', [150 350 400 100], ...
             'FontSize', 12, ...
             'BackgroundColor', get(parent_tab, 'BackgroundColor'), ...
             'HorizontalAlignment', 'center');
    
    % 收集所有算法的数据（使用最后一次MC运行的数据）
    est_data_array = cell(1, n_algorithms);
    meas_data_array = cell(1, n_algorithms);
    algorithm_names = cell(1, n_algorithms);
    
    for i = 1:n_algorithms
        algo = algo_names{i};
        result = all_results.(algo).results{end}; % 使用最后一次MC运行的数据
        est_data_array{i} = result.est_data;
        meas_data_array{i} = result.meas_data;
        algorithm_names{i} = all_results.(algo).display_name;
    end
    
    % 使用最后一次MC的真实数据
    result_true_data = all_results.(algo_names{1}).results{end}.true_data;
    
    % 从结果数据中重构必要的params信息
    first_result = all_results.(algo_names{1}).results{end};
    reconstructed_params = struct();
    reconstructed_params.ratio = round(length(result_true_data.x) / length(first_result.est_data.x));
    if isfield(first_result, 'dt_kf')
        reconstructed_params.dt_kf = first_result.dt_kf;
    else
        reconstructed_params.dt_kf = 0.1; % 默认值
    end
    
    uicontrol('Parent', parent_tab, ...
             'Style', 'pushbutton', ...
             'String', '显示详细分析', ...
             'Position', [300 250 150 40], ...
             'FontSize', 12, ...
             'Callback', @(~,~) detailed_analysis(reconstructed_params, result_true_data, est_data_array, meas_data_array, algorithm_names));
end

function create_multi_mc_analysis_content(parent_tab, all_results, mc_runs)
    % 创建多次MC分析内容
    try
        algo_names = fieldnames(all_results);
        n_algos = length(algo_names);
        
        if n_algos == 0
            uicontrol('Parent', parent_tab, 'Style', 'text', ...
                     'String', '没有可分析的结果', ...
                     'Position', [150 400 200 50], ...
                     'FontSize', 12, 'HorizontalAlignment', 'center');
            return;
        end
        
        
        % 创建2x3布局显示统计结果
        for i = 1:6
            subplot(2, 3, i, 'Parent', parent_tab);
        end
        subplots = get(parent_tab, 'Children');
        subplots = flipud(subplots);
        
        % 计算详细统计量
        [stats] = calculate_detailed_statistics(all_results);
        
        % 安全地创建各个统计子图
        safe_create_plot(@() create_position_rmse_plot(subplots(1), stats, mc_runs), '位置RMSE');
        safe_create_plot(@() create_velocity_rmse_plot(subplots(2), stats), '速度RMSE');
        safe_create_plot(@() create_heading_rmse_plot(subplots(3), stats), '航向RMSE');
        safe_create_plot(@() create_execution_time_plot(subplots(4), stats), '执行时间');
        safe_create_plot(@() create_rmse_distribution_plot(subplots(5), all_results, stats), 'RMSE分布');
        safe_create_plot(@() create_performance_summary_plot(subplots(6), stats, mc_runs), '性能摘要');
        
    catch ME
        uicontrol('Parent', parent_tab, 'Style', 'text', ...
                 'String', sprintf('多MC分析创建失败: %s', ME.message), ...
                 'Position', [20 350 500 100], ...
                 'FontSize', 10, 'BackgroundColor', [1 0.8 0.8], ...
                 'HorizontalAlignment', 'left');
    end
end

function safe_create_plot(plot_func, plot_name)
    % 安全地创建图表
    try
        plot_func();
    catch ME
        fprintf('创建%s图表时出错: %s\n', plot_name, ME.message);
    end
end

function stats = calculate_detailed_statistics(all_results)
    % 计算详细统计量
    algo_names = fieldnames(all_results);
    n_algos = length(algo_names);
    
    stats = struct();
    stats.algo_names = algo_names;
    stats.display_names = cell(1, n_algos);
    stats.mean_rmse_pos = zeros(1, n_algos);
    stats.std_rmse_pos = zeros(1, n_algos);
    stats.mean_rmse_v = zeros(1, n_algos);
    stats.mean_rmse_phi = zeros(1, n_algos);
    stats.mean_exec_time = zeros(1, n_algos);
    
    for i = 1:n_algos
        algo = algo_names{i};
        results = all_results.(algo).results;
        stats.display_names{i} = all_results.(algo).display_name;
        
        % 提取各种性能指标
        rmse_pos_values = cellfun(@(x) x.rmse_pos, results);
        rmse_v_values = cellfun(@(x) x.rmse_v, results);
        rmse_phi_values = cellfun(@(x) x.rmse_phi, results);
        exec_times = cellfun(@(x) x.exec_time, results);
        
        % 计算统计量
        stats.mean_rmse_pos(i) = mean(rmse_pos_values);
        stats.std_rmse_pos(i) = std(rmse_pos_values);
        stats.mean_rmse_v(i) = mean(rmse_v_values);
        stats.mean_rmse_phi(i) = mean(rmse_phi_values);
        stats.mean_exec_time(i) = mean(exec_times);
    end
end

function create_position_rmse_plot(subplot_handle, stats, mc_runs)
    axes(subplot_handle);
    bar(stats.mean_rmse_pos);
    hold on;
    errorbar(1:length(stats.display_names), stats.mean_rmse_pos, stats.std_rmse_pos, 'k.', 'LineWidth', 1.5);
    set(gca, 'XTickLabel', stats.display_names);
    title(sprintf('位置RMSE对比 (MC=%d)', mc_runs));
    ylabel('RMSE (m)'); 
    grid on;
end

function create_velocity_rmse_plot(subplot_handle, stats)
    axes(subplot_handle);
    bar(stats.mean_rmse_v);
    set(gca, 'XTickLabel', stats.display_names);
    title('速度RMSE对比');
    ylabel('RMSE (m/s)'); 
    grid on;
end

function create_heading_rmse_plot(subplot_handle, stats)
    axes(subplot_handle);
    bar(rad2deg(stats.mean_rmse_phi));
    set(gca, 'XTickLabel', stats.display_names);
    title('航向RMSE对比');
    ylabel('RMSE (度)'); 
    grid on;
end

function create_execution_time_plot(subplot_handle, stats)
    axes(subplot_handle);
    bar(stats.mean_exec_time);
    set(gca, 'XTickLabel', stats.display_names);
    title('执行时间对比');
    ylabel('时间 (s)'); 
    grid on;
end

function create_rmse_distribution_plot(subplot_handle, all_results, stats)
    axes(subplot_handle);
    % 预分配数据容器以提高性能
    num_algos = length(stats.algo_names);
    total_samples = 0;
    for i = 1:num_algos
        algo = stats.algo_names{i};
        total_samples = total_samples + length(all_results.(algo).results);
    end
    
    rmse_data = zeros(total_samples, 1);
    group_labels = zeros(total_samples, 1);
    current_idx = 1;
    
    for i = 1:num_algos
        algo = stats.algo_names{i};
        rmse_values = cellfun(@(x) x.rmse_pos, all_results.(algo).results);
        num_values = length(rmse_values);
        
        rmse_data(current_idx:current_idx+num_values-1) = rmse_values';
        group_labels(current_idx:current_idx+num_values-1) = i;
        current_idx = current_idx + num_values;
    end
    
    boxplot(rmse_data, group_labels, 'Labels', stats.display_names);
    title('位置RMSE分布');
    ylabel('RMSE (m)'); 
    grid on;
end

function create_performance_summary_plot(subplot_handle, stats, mc_runs)
    axes(subplot_handle);
    text(0.1, 0.8, sprintf('Monte Carlo 统计 (N=%d)', mc_runs), 'FontSize', 12, 'FontWeight', 'bold');
    y_pos = 0.7;
    
    for i = 1:length(stats.display_names)
        text(0.1, y_pos, sprintf('%s:', stats.display_names{i}), 'FontWeight', 'bold');
        text(0.1, y_pos-0.05, sprintf('  位置RMSE: %.2f±%.2f m', stats.mean_rmse_pos(i), stats.std_rmse_pos(i)));
        text(0.1, y_pos-0.1, sprintf('  执行时间: %.3f s', stats.mean_exec_time(i)));
        y_pos = y_pos - 0.15;
    end
    
    set(gca, 'XLim', [0 1], 'YLim', [0 1], 'XTick', [], 'YTick', []);
    title('性能摘要');
end

function colors = get_algorithm_colors(n_algos)
    % 获取算法颜色列表
    base_colors = {'b', 'r', 'g', 'm', 'c', 'y'};
    colors = cell(1, n_algos);
    
    for i = 1:n_algos
        colors{i} = base_colors{mod(i-1, length(base_colors)) + 1};
    end
end