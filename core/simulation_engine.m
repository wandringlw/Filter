function [all_results, status] = simulation_engine(selected_algos, selected_display_names, ...
                                                  base_params, gui_params, param_handles, ...
                                                  status_callback)
    % 仿真执行引擎 - 负责Monte Carlo仿真的执行逻辑
    % 输入:
    %   selected_algos - 选中的算法列表
    %   selected_display_names - 算法显示名称
    %   base_params - 基础参数配置
    %   gui_params - GUI参数配置
    %   param_handles - 参数控件句柄
    %   status_callback - 状态更新回调函数
    % 输出:
    %   all_results - 所有算法的结果
    %   status - 执行状态结构体
    
    try
        % 初始化状态
        status = struct('success', true, 'error_msg', '', 'completed', false);
        
        % 执行参数验证和处理
        [seed_config, status] = prepare_seed_configuration(gui_params, param_handles, status);
        if ~status.success
            all_results = [];
            return;
        end
        
        % 获取Monte Carlo配置
        mc_runs = str2double(get(param_handles.mc_runs_edit, 'String'));
        
        % 加载地图数据
        status_callback('加载地图数据...');
        map_data = load_map_data();
        
        % 初始化结果存储（将在后续被重新赋值）
        
        % 执行Monte Carlo仿真
        num_algos = length(selected_algos);
        if num_algos > 1
            % 多算法比较模式
            [all_results, status] = run_comparison_mode(selected_algos, selected_display_names, ...
                                                       base_params, mc_runs, seed_config, ...
                                                       map_data, param_handles, status_callback);
        else
            % 单算法分析模式
            [all_results, status] = run_single_mode(selected_algos, selected_display_names, ...
                                                   base_params, mc_runs, seed_config, ...
                                                   map_data, param_handles, status_callback);
        end
        
        if status.success
            status.completed = true;
            status_callback('仿真完成');
        end
        
    catch ME
        status.success = false;
        status.error_msg = sprintf('仿真执行出错: %s\n位置: %s', ME.message, ME.stack(1).name);
        status.completed = false;
        all_results = [];
    end
end

%% 核心执行函数

function [seed_config, status] = prepare_seed_configuration(~, param_handles, status)
    % 准备种子配置
    seed_config = struct();
    
    try
        % 获取轨迹种子
        traj_seed_str = get(param_handles.traj_seed_edit, 'String');
        if strcmpi(traj_seed_str, 'random')
            base_traj_seed = rng('shuffle');
            seed_config.base_traj_seed = base_traj_seed.Seed;
            seed_config.use_fixed_traj_seed = false;
        else
            seed_config.base_traj_seed = str2double(traj_seed_str);
            seed_config.use_fixed_traj_seed = true;
        end
        
        % 获取算法种子
        algo_seed_str = get(param_handles.algo_seed_edit, 'String');
        if strcmpi(algo_seed_str, 'random')
            base_algo_seed = rng('shuffle');
            seed_config.base_algo_seed = base_algo_seed.Seed;
            seed_config.use_fixed_algo_seed = false;
        else
            seed_config.base_algo_seed = str2double(algo_seed_str);
            seed_config.use_fixed_algo_seed = true;
        end
        
        % 获取算法种子模式
        algo_seed_mode_value = get(param_handles.algo_seed_mode_popup, 'Value');
        if algo_seed_mode_value == 1
            seed_config.algo_seed_mode = 'random';
        else
            seed_config.algo_seed_mode = 'fixed';
        end
        
    catch ME
        status.success = false;
        status.error_msg = sprintf('种子配置失败: %s', ME.message);
    end
end

function [all_results, status] = run_comparison_mode(selected_algos, selected_display_names, ...
                                                    base_params, mc_runs, seed_config, ...
                                                    map_data, param_handles, status_callback)
    % 算法比较模式执行
    all_results = struct();
    
    for mc = 1:mc_runs
        status_callback(sprintf('MC运行 %d/%d: 生成轨迹数据...', mc, mc_runs));
        
        % 为每轮MC生成唯一的轨迹数据
        mc_traj_seed = seed_config.base_traj_seed + (mc - 1) * 100;
        [true_data, meas_data] = generate_trajectory_data(base_params, map_data, ...
                                                         mc_traj_seed, seed_config.use_fixed_traj_seed);
        
        % 所有算法使用这条轨迹
        for algo_idx = 1:length(selected_algos)
            algo = selected_algos{algo_idx};
            display_name = selected_display_names{algo_idx};
            status_callback(sprintf('MC运行 %d/%d: 运行%s算法...', mc, mc_runs, display_name));
            
            % 确定算法种子
            mc_algo_seed = calculate_algorithm_seed(seed_config, mc, algo_idx);
            
            % 运行单个算法
            fprintf('\n========== MC运行 %d/%d: %s (轨迹种子:%d, 算法种子:%d, 种子模式:%s) ==========\n', ...
                    mc, mc_runs, display_name, mc_traj_seed, mc_algo_seed, seed_config.algo_seed_mode);
            
            result = run_single_algorithm(algo, base_params, true_data, meas_data, ...
                                         map_data, mc_algo_seed, seed_config.use_fixed_algo_seed, ...
                                         param_handles);
            
            fprintf('算法完成 - 执行时间: %.3fs, 位置RMSE: %.2fm\n', result.exec_time, result.rmse_pos);
            
            % 存储结果
            all_results = store_algorithm_result(all_results, algo, display_name, ...
                                               result, true_data, meas_data, mc, mc_runs);
            
            pause(0.1); % 给界面响应时间
        end
    end
    
    status.success = true;
end

function [all_results, status] = run_single_mode(selected_algos, selected_display_names, ...
                                                base_params, mc_runs, seed_config, ...
                                                map_data, param_handles, status_callback)
    % 单算法模式执行
    all_results = struct();
    
    for mc = 1:mc_runs
        status_callback(sprintf('MC运行 %d/%d: 生成轨迹数据...', mc, mc_runs));
        
        % 为每轮MC生成唯一的轨迹数据
        mc_traj_seed = seed_config.base_traj_seed + (mc - 1) * 100;
        [true_data, meas_data] = generate_trajectory_data(base_params, map_data, ...
                                                         mc_traj_seed, seed_config.use_fixed_traj_seed);
        
        % 运行算法
        algo = selected_algos{1};
        display_name = selected_display_names{1};
        status_callback(sprintf('MC运行 %d/%d: 运行%s算法...', mc, mc_runs, display_name));
        
        % 确定算法种子
        mc_algo_seed = calculate_algorithm_seed(seed_config, mc, 1);
        
        % 运行单个算法
        fprintf('\n========== MC运行 %d/%d: %s (轨迹种子:%d, 算法种子:%d, 种子模式:%s) ==========\n', ...
                mc, mc_runs, display_name, mc_traj_seed, mc_algo_seed, seed_config.algo_seed_mode);
        
        result = run_single_algorithm(algo, base_params, true_data, meas_data, ...
                                     map_data, mc_algo_seed, seed_config.use_fixed_algo_seed, ...
                                     param_handles);
        
        % 存储结果
        all_results = store_algorithm_result(all_results, algo, display_name, ...
                                           result, true_data, meas_data, mc, mc_runs);
    end
    
    status.success = true;
end

%% 辅助函数

function mc_algo_seed = calculate_algorithm_seed(seed_config, mc, algo_idx)
    % 计算算法种子
    if strcmp(seed_config.algo_seed_mode, 'fixed')
        % 固定模式：每次MC使用相同的算法种子
        mc_algo_seed = seed_config.base_algo_seed + algo_idx;
    else
        % 随机模式：每次MC使用不同的算法种子
        mc_algo_seed = seed_config.base_algo_seed + (mc - 1) * 10 + algo_idx;
    end
end

function all_results = store_algorithm_result(all_results, algo, display_name, ...
                                            result, true_data, meas_data, mc, mc_runs)
    % 存储算法结果
    if ~isfield(all_results, algo)
        all_results.(algo) = struct();
        all_results.(algo).results = cell(1, mc_runs);
        all_results.(algo).display_name = display_name;
    end
    
    result.true_data = true_data;
    result.meas_data = meas_data;
    all_results.(algo).results{mc} = result;
end

function map_data = load_map_data()
    % 加载地图数据
    map_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'simulation', 'map.mat');
    load(map_path, 'depth_data');
    depth_data = flipud(depth_data);
    [mapH, mapW] = size(depth_data);
    dx_map = 30;
    xVecMap = 0:dx_map:dx_map*(mapW-1);
    yVecMap = 0:dx_map:dx_map*(mapH-1);
    map_data = struct('depth', depth_data, 'xVec', xVecMap, 'yVec', yVecMap);
end

function [true_data, meas_data] = generate_trajectory_data(params, map_data, mc_traj_seed, use_fixed_traj_seed)
    % 生成轨迹数据
    if use_fixed_traj_seed
        rng(mc_traj_seed);
    end
    [true_data, meas_data] = generate_true_trajectory(params);
    meas_data.depth = add_depth_measurements(true_data, map_data, params);
end

function result = run_single_algorithm(algo, base_params, true_data, meas_data, ...
                                      map_data, mc_algo_seed, use_fixed_algo_seed, ...
                                      param_handles)
    % 运行单个算法
    if use_fixed_algo_seed
        rng(mc_algo_seed);
    end
    
    % 复制基础参数并更新算法特定参数
    algo_params = base_params;
    
    switch algo
        case 'PMF'
            algo_params = update_pmf_params(algo_params, param_handles);
            [est_data, dr_data, exec_time] = PointMassFilter(algo_params, true_data, meas_data, map_data);
            
        case 'Pure_PF'
            algo_params = update_pf_params(algo_params, param_handles);
            [est_data, dr_data, exec_time] = ParticleFilter(algo_params, true_data, meas_data, map_data);
            
        case 'RBPF'
            algo_params = update_rbpf_params(algo_params, param_handles);
            [est_data, exec_time] = RaoBlackwellizedParticleFilter(algo_params, true_data, meas_data, map_data);
            dr_data = [];
            
        case 'Hybrid'
            algo_params = update_hybrid_params(algo_params, param_handles, false);
            [est_data, dr_data, exec_time] = HybridParticleFilter(algo_params, true_data, meas_data, map_data);
            
        case 'Hybrid2'
            algo_params = update_hybrid_params(algo_params, param_handles, false);
            [est_data, dr_data, exec_time] = HybridParticleFilter2(algo_params, true_data, meas_data, map_data);
            
        case 'Hybrid3'
            algo_params = update_hybrid_params(algo_params, param_handles, true);
            [est_data, dr_data, exec_time] = HybridParticleFilter3(algo_params, true_data, meas_data, map_data);
    end
    
    % 计算性能指标
    result = calculate_performance_metrics(est_data, dr_data, true_data, base_params, exec_time);
end

%% 参数更新函数

function algo_params = update_pmf_params(algo_params, param_handles)
    algo_params.Ngrid = str2double(get(param_handles.pmf_params_Ngrid, 'String'));
    algo_params.dx_pmf = str2double(get(param_handles.pmf_params_dx_pmf, 'String'));
    algo_params.grid_range = str2double(get(param_handles.pmf_params_grid_range, 'String'));
end

function algo_params = update_pf_params(algo_params, param_handles)
    algo_params.depthGatingThreshold = str2double(get(param_handles.pf_params_depthGatingThreshold, 'String'));
    algo_params.N_particles = str2double(get(param_handles.pf_params_N_particles, 'String'));
    algo_params.resampling_threshold = str2double(get(param_handles.pf_params_resampling_threshold, 'String'));
    algo_params.Ne_eff_threshold = algo_params.N_particles * algo_params.resampling_threshold;
end

function algo_params = update_rbpf_params(algo_params, param_handles)
    algo_params.N_particles = str2double(get(param_handles.rbpf_params_N_particles, 'String'));
    algo_params.resampling_threshold = str2double(get(param_handles.rbpf_params_resampling_threshold, 'String'));
    algo_params.Ne_eff_threshold = algo_params.N_particles * algo_params.resampling_threshold;
end

function algo_params = update_hybrid_params(algo_params, param_handles, include_ts)
    algo_params.N_fixed = str2double(get(param_handles.hybrid_params_N_fixed, 'String'));
    algo_params.N_random = str2double(get(param_handles.hybrid_params_N_random, 'String'));
    algo_params.resampling_threshold = str2double(get(param_handles.hybrid_params_resampling_threshold, 'String'));
    algo_params.N_particles = algo_params.N_fixed + algo_params.N_random;
    algo_params.Ne_eff_threshold = algo_params.N_particles * algo_params.resampling_threshold;
    
    if include_ts
        algo_params.ts = str2double(get(param_handles.hybrid_params_ts, 'String'));
    end
end

function result = calculate_performance_metrics(est_data, dr_data, true_data, base_params, exec_time)
    % 计算性能指标
    
    % 计算RMSE - 使用原始弧度制数据进行计算，避免修改输入数据
    true_indices = 1:base_params.ratio:length(true_data.x);
    rmse_pos = sqrt(mean((est_data.x - true_data.x(true_indices)).^2 + ...
                      (est_data.y - true_data.y(true_indices)).^2));
    rmse_v = sqrt(mean((est_data.v - true_data.v(true_indices)).^2));
    
    % 航向角误差计算 - 直接使用弧度制数据，确保角度差的正确计算
    phi_error_rad = angdiff(est_data.phi, true_data.phi(true_indices));
    rmse_phi = sqrt(mean(phi_error_rad.^2));  % 弧度制的RMSE
    
    % 数据有效性验证
    if max(abs(est_data.phi)) > 2*pi
        warning('警告：est_data.phi 包含超出 ±2π 范围的值，可能单位不正确');
    end
    if max(abs(true_data.phi(true_indices))) > 2*pi
        warning('警告：true_data.phi 包含超出 ±2π 范围的值，可能单位不正确');
    end
    
    % 返回结果结构体
    result = struct();
    result.est_data = est_data;
    result.dr_data = dr_data;
    result.exec_time = exec_time;
    result.rmse_pos = rmse_pos;
    result.rmse_v = rmse_v;
    result.rmse_phi = rmse_phi;
    result.time = (0:length(est_data.x)-1) * base_params.dt_pf;
end