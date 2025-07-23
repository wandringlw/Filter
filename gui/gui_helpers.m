function varargout = gui_helpers(action, varargin)
    % GUI辅助函数集合 - 提供GUI相关的工具函数
    % 使用方式: result = gui_helpers('function_name', param1, param2, ...)
    
    switch action
        case 'get_gui_params'
            varargout{1} = get_gui_params(varargin{:});
        case 'create_param_handles'
            varargout{1} = create_param_handles(varargin{:});
        case 'update_algorithm_visibility'
            update_algorithm_visibility(varargin{:});
        case 'validate_algorithm_selection'
            [varargout{1}, varargout{2}] = validate_algorithm_selection(varargin{:});
        case 'create_status_callback'
            varargout{1} = create_status_callback(varargin{:});
        case 'safe_button_operation'
            safe_button_operation(varargin{:});
        otherwise
            error('Unknown action: %s', action);
    end
end

function gui_params = get_gui_params(algo_checkboxes, algo_panel, algorithms, model_bg, ...
                                    algo_seed_mode_popup, sim_time_edit, vis_bg)
    % 获取GUI参数配置
    gui_params = struct();
    
    try
        % 从复选框获取选择的算法
        checkbox_names = getappdata(algo_panel, 'checkbox_names');
        % 预分配数组以提高性能
        selected_indices = zeros(1, length(checkbox_names));
        count = 0;
        for i = 1:length(checkbox_names)
            checkbox_name = checkbox_names{i};
            if isfield(algo_checkboxes, checkbox_name) && get(algo_checkboxes.(checkbox_name), 'Value')
                count = count + 1;
                selected_indices(count) = i;
            end
        end
        selected_indices = selected_indices(1:count);  % 截取有效部分
        
        selected_algos = algorithms(selected_indices);
        
        if isscalar(selected_indices)
            gui_params.filter_type = lower(selected_algos{1});
        else
            gui_params.filter_type = 'generic';
        end
        
        % 获取模型类型
        selected_model = get(model_bg.SelectedObject, 'String');
        if strcmp(selected_model, '耦合模型')
            gui_params.model_type = 'coupled';
        else
            gui_params.model_type = 'decoupled';
        end
        
        % 获取算法种子模式
        algo_seed_mode_value = get(algo_seed_mode_popup, 'Value');
        if algo_seed_mode_value == 1
            gui_params.algo_seed_mode = 'random';
        else
            gui_params.algo_seed_mode = 'fixed';
        end
        
        % 设置可视化参数（始终根据可视化控件的选择决定）
        vis_enabled = true;  % 默认启用可视化
        
        try
            if ishandle(vis_bg) && isprop(vis_bg, 'SelectedObject') && ~isempty(vis_bg.SelectedObject)
                selected_string = get(vis_bg.SelectedObject, 'String');
                vis_enabled = strcmp(selected_string, '实时可视化');
            end
        catch
            % 如果访问失败，使用默认值
            vis_enabled = true;
        end
        
        gui_params.visualization_enabled = vis_enabled;
        gui_params.Pure_showVisualization = vis_enabled;
        gui_params.PMF_showVisualization = vis_enabled;
        gui_params.Hybrid_showVisualization = vis_enabled;
        
        
        % 获取仿真时间
        sim_time_str = get(sim_time_edit, 'String');
        gui_params.sim_time = str2double(sim_time_str);
        if isnan(gui_params.sim_time) || gui_params.sim_time <= 0
            gui_params.sim_time = 1000;  % 默认值
            set(sim_time_edit, 'String', '1000');  % 更新GUI显示
        end
        
    catch ME
        % 如果出错，返回默认配置
        warning('获取GUI参数时出错: %s，使用默认配置', getReport(ME, 'basic'));
        gui_params = get_default_gui_params();
    end
end

function default_params = get_default_gui_params()
    % 获取默认GUI参数
    default_params = struct();
    default_params.filter_type = 'pmf';
    default_params.model_type = 'decoupled';
    default_params.algo_seed_mode = 'random';
    default_params.visualization_enabled = true;
    default_params.Pure_showVisualization = true;
    default_params.PMF_showVisualization = true;
    default_params.Hybrid_showVisualization = true;
    default_params.sim_time = 1000;
end

function param_handles = create_param_handles(pmf_params_Ngrid, pmf_params_dx_pmf, pmf_params_grid_range, ...
                                             pf_params_depthGatingThreshold, pf_params_N_particles, pf_params_resampling_threshold, ...
                                             hybrid_params_N_fixed, hybrid_params_N_random, hybrid_params_resampling_threshold, hybrid_params_ts, ...
                                             rbpf_params_N_particles, rbpf_params_resampling_threshold, ...
                                             traj_seed_edit, algo_seed_edit, algo_seed_mode_popup, mc_runs_edit)
    % 创建参数控件句柄结构体
    param_handles = struct();
    
    % PMF参数
    param_handles.pmf_params_Ngrid = pmf_params_Ngrid;
    param_handles.pmf_params_dx_pmf = pmf_params_dx_pmf;
    param_handles.pmf_params_grid_range = pmf_params_grid_range;
    
    % PF参数
    param_handles.pf_params_depthGatingThreshold = pf_params_depthGatingThreshold;
    param_handles.pf_params_N_particles = pf_params_N_particles;
    param_handles.pf_params_resampling_threshold = pf_params_resampling_threshold;
    
    % Hybrid参数
    param_handles.hybrid_params_N_fixed = hybrid_params_N_fixed;
    param_handles.hybrid_params_N_random = hybrid_params_N_random;
    param_handles.hybrid_params_resampling_threshold = hybrid_params_resampling_threshold;
    param_handles.hybrid_params_ts = hybrid_params_ts;
    
    % RBPF参数
    param_handles.rbpf_params_N_particles = rbpf_params_N_particles;
    param_handles.rbpf_params_resampling_threshold = rbpf_params_resampling_threshold;
    
    % 其他控件
    param_handles.traj_seed_edit = traj_seed_edit;
    param_handles.algo_seed_edit = algo_seed_edit;
    param_handles.algo_seed_mode_popup = algo_seed_mode_popup;
    param_handles.mc_runs_edit = mc_runs_edit;
end

function update_algorithm_visibility(selected_algo, pmf_group, pf_group, hybrid_group, rbpf_group, ...
                                   hybrid_params_ts_label, hybrid_params_ts)
    % 更新算法参数界面可见性
    try
        % 隐藏所有参数组
        set([pmf_group, pf_group, hybrid_group, rbpf_group], 'Visible', 'off');
        
        % 显示对应算法的参数组
        switch selected_algo
            case 'PMF'
                set(pmf_group, 'Visible', 'on');
            case 'Pure_PF'
                set(pf_group, 'Visible', 'on');
            case 'Hybrid'
                set(hybrid_group, 'Visible', 'on');
                set([hybrid_params_ts_label, hybrid_params_ts], 'Visible', 'off');
            case 'Hybrid2'
                set(hybrid_group, 'Visible', 'on');
                set([hybrid_params_ts_label, hybrid_params_ts], 'Visible', 'off');
            case 'Hybrid3'
                set(hybrid_group, 'Visible', 'on');
                set([hybrid_params_ts_label, hybrid_params_ts], 'Visible', 'on');
            case 'RBPF'
                set(rbpf_group, 'Visible', 'on');
            otherwise
                warning('未知算法类型: %s', selected_algo);
        end
        
    catch ME
        warning('更新算法界面可见性时出错: %s', getReport(ME, 'basic'));
    end
end

function [is_valid, error_msg] = validate_algorithm_selection(selected_indices)
    % 验证算法选择
    is_valid = true;
    error_msg = '';
    
    try
        if isempty(selected_indices)
            is_valid = false;
            error_msg = '请选择至少一个算法';
            return;
        end
        
        % 验证通过
        
    catch ME
        is_valid = false;
        error_msg = sprintf('验证算法选择时出错: %s', ME.message);
    end
end

function status_callback = create_status_callback(status_text)
    % 创建状态更新回调函数
    status_callback = @(msg) update_status_safely(status_text, msg);
end

function update_status_safely(status_text, message)
    % 安全地更新状态文本
    try
        if isvalid(status_text)
            set(status_text, 'String', message);
            drawnow;
        end
    catch ME
        % 静默处理状态更新错误，避免影响主要功能
        fprintf('状态更新失败: %s\n', ME.message);
    end
end

function safe_button_operation(button_handle, operation, text_during, text_after)
    % 安全地执行按钮操作
    % operation: 'disable'/'enable'
    % text_during: 操作期间的按钮文本
    % text_after: 操作完成后的按钮文本
    
    try
        if ~isvalid(button_handle)
            return;
        end
        
        switch operation
            case 'disable'
                set(button_handle, 'Enable', 'off');
                if nargin >= 3 && ~isempty(text_during)
                    set(button_handle, 'String', text_during);
                end
            case 'enable'
                set(button_handle, 'Enable', 'on');
                if nargin >= 4 && ~isempty(text_after)
                    set(button_handle, 'String', text_after);
                end
        end
        
    catch ME
        warning('按钮操作失败: %s', getReport(ME, 'basic'));
    end
end

