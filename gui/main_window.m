function main_window()
    % 滤波器平台GUI - 模块化、更好的错误处理和可维护性
    % 保存当前目录
    original_dir = pwd();
    
    % 获取项目根目录路径
    gui_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(gui_dir);
    cd(project_root);
    
    % 添加所需路径
    % 添加所有模块到路径
    addpath('core', 'gui', 'simulation', 'Algorithms');

    % 创建主窗口
    fig = create_main_window(@cleanup);
    
    % 创建界面组件
    [ui_components, param_handles] = create_gui_components(fig);
    
    % 设置回调函数
    setup_callbacks(ui_components, param_handles);
    
    % 清理函数
    function cleanup(~,~)
        % 恢复原始目录
        cd(original_dir);
        % 删除路径
        rmpath('core');
        rmpath('gui');
        rmpath('simulation');
        rmpath('Algorithms');
        % 关闭窗口
        delete(gcf);
    end
end

function fig = create_main_window(cleanup_func)
    % 创建主窗口
    fig = figure('Name', '滤波器平台', ...
        'Position', [100 100 500 850], ...
        'NumberTitle', 'off', ...
        'MenuBar', 'none', ...
        'Resize', 'off', ...
        'CloseRequestFcn', cleanup_func);
end

function [ui_components, param_handles] = create_gui_components(fig)
    % 创建所有GUI组件
    ui_components = struct();
    
    % 创建面板和控件
    [ui_components.algo_panel, ui_components.algo_checkboxes] = create_algorithm_selection_panel(fig);
    [ui_components.model_panel, ui_components.model_bg] = create_model_selection_panel(fig);
    [ui_components.seed_panel, seed_controls] = create_seed_control_panel(fig);
    [ui_components.vis_panel, ui_components.vis_bg] = create_visualization_panel(fig);
    [ui_components.param_panel, param_groups] = create_parameter_panel(fig);
    [ui_components.status_text, ui_components.run_button] = create_control_panel(fig);
    
    % 合并种子控件
    ui_components = merge_structures(ui_components, seed_controls);
    
    % 合并参数组
    ui_components = merge_structures(ui_components, param_groups);
    
    % 创建参数句柄结构
    param_handles = gui_helpers('create_param_handles', ...
        ui_components.pmf_params_Ngrid, ui_components.pmf_params_dx_pmf, ui_components.pmf_params_grid_range, ...
        ui_components.pf_params_depthGatingThreshold, ui_components.pf_params_N_particles, ui_components.pf_params_resampling_threshold, ...
        ui_components.hybrid_params_N_fixed, ui_components.hybrid_params_N_random, ui_components.hybrid_params_resampling_threshold, ui_components.hybrid_params_ts, ...
        ui_components.rbpf_params_N_particles, ui_components.rbpf_params_resampling_threshold, ...
        ui_components.traj_seed_edit, ui_components.algo_seed_edit, ui_components.algo_seed_mode_popup, ui_components.mc_runs_edit);
end


function [algo_panel, algo_checkboxes] = create_algorithm_selection_panel(fig)
    % 创建算法选择面板 - 使用复选框组实现直接点击选择
    algo_panel = uipanel('Parent', fig, 'Title', '算法选择', ...
        'Position', [0.05 0.76 0.9 0.20], ...
        'FontSize', 10, 'FontWeight', 'bold');
    
    algorithms = {'PMF', 'Pure_PF', 'RBPF', 'Hybrid', 'Hybrid2', 'Hybrid3'};
    algorithm_display_names = {'点质量滤波器 (PMF)', '粒子滤波器 (PF)', 'Rao-Blackwellized粒子滤波器 (RBPF)', ...
                              '混合粒子滤波器 (Hybrid PF)', '混合粒子滤波器2 (Hybrid PF2)', '自适应混合粒子滤波器 (Hybrid PF3)'};
    
    % 创建复选框组 - 3行2列布局
    algo_checkboxes = struct();
    checkbox_positions = [
        [20, 130, 200, 25];   % PMF - 第1行左列
        [240, 130, 200, 25];  % Pure_PF - 第1行右列
        [20, 100, 200, 25];   % RBPF - 第2行左列  
        [240, 100, 200, 25];  % Hybrid - 第2行右列
        [20, 70, 200, 25];    % Hybrid2 - 第3行左列
        [240, 70, 200, 25];   % Hybrid3 - 第3行右列
    ];
    
    checkbox_names = {'pmf', 'pure_pf', 'rbpf', 'hybrid', 'hybrid2', 'hybrid3'};
    
    for i = 1:6
        algo_checkboxes.(checkbox_names{i}) = uicontrol('Parent', algo_panel, ...
            'Style', 'checkbox', ...
            'String', algorithm_display_names{i}, ...
            'Position', checkbox_positions(i, :), ...
            'FontSize', 9, ...
            'Value', i == 1);  % 默认选中PMF
    end
    
    % 存储算法信息以供后续使用
    setappdata(algo_panel, 'algorithms', algorithms);
    setappdata(algo_panel, 'display_names', algorithm_display_names);
    setappdata(algo_panel, 'checkbox_names', checkbox_names);
end

function [model_panel, model_bg] = create_model_selection_panel(fig)
    % 创建模型选择面板
    model_panel = uipanel('Parent', fig, 'Title', '模型选择', ...
        'Position', [0.05 0.68 0.9 0.07], ...
        'FontSize', 10, 'FontWeight', 'bold');
    
    model_bg = uibuttongroup('Parent', model_panel, ...
        'Position', [0.05 0.15 0.9 0.7], ...
        'BorderType', 'none');
    
    uicontrol(model_bg, 'Style', 'radiobutton', ...
        'String', '解耦模型', ...
        'Position', [20 5 100 25], ...
        'FontSize', 9, 'Value', 1);
    
    uicontrol(model_bg, 'Style', 'radiobutton', ...
        'String', '耦合模型', ...
        'Position', [140 5 100 25], ...
        'FontSize', 9);
end

function [seed_panel, seed_controls] = create_seed_control_panel(fig)
    % 创建种子控制面板
    seed_panel = uipanel('Parent', fig, 'Title', '随机种子控制', ...
        'Position', [0.05 0.47 0.9 0.20], ...
        'FontSize', 10, 'FontWeight', 'bold');
    
    seed_controls = struct();
    
    % 轨迹种子
    uicontrol('Parent', seed_panel, 'Style', 'text', 'String', '轨迹种子:', ...
        'Position', [10 85 60 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    seed_controls.traj_seed_edit = uicontrol('Parent', seed_panel, 'Style', 'edit', ...
        'String', 'random', 'Position', [75 85 80 22], 'FontSize', 9);
    
    % 算法种子
    uicontrol('Parent', seed_panel, 'Style', 'text', 'String', '算法种子:', ...
        'Position', [10 60 60 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    seed_controls.algo_seed_edit = uicontrol('Parent', seed_panel, 'Style', 'edit', ...
        'String', 'random', 'Position', [75 60 80 22], 'FontSize', 9);
    
    % 算法种子模式
    uicontrol('Parent', seed_panel, 'Style', 'text', 'String', '算法种子模式:', ...
        'Position', [10 35 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    seed_controls.algo_seed_mode_popup = uicontrol('Parent', seed_panel, 'Style', 'popupmenu', ...
        'String', {'随机(每次MC不同)', '固定(每次MC相同)'}, ...
        'Position', [95 35 120 22], 'FontSize', 9, 'Value', 1);
    
    % MC次数
    uicontrol('Parent', seed_panel, 'Style', 'text', 'String', 'MC次数:', ...
        'Position', [240 85 60 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    seed_controls.mc_runs_edit = uicontrol('Parent', seed_panel, 'Style', 'edit', ...
        'String', '1', 'Position', [305 85 60 22], 'FontSize', 9);
    
    % 仿真时间
    uicontrol('Parent', seed_panel, 'Style', 'text', 'String', '仿真时间(s):', ...
        'Position', [240 60 70 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    seed_controls.sim_time_edit = uicontrol('Parent', seed_panel, 'Style', 'edit', ...
        'String', '1000', 'Position', [315 60 50 22], 'FontSize', 9);
    
    % 说明文字
    uicontrol('Parent', seed_panel, 'Style', 'text', ...
        'String', '说明: 轨迹种子每次MC+100; 算法种子模式影响噪声一致性', ...
        'Position', [10 5 400 20], 'FontSize', 8, ...
        'HorizontalAlignment', 'left', 'ForegroundColor', [0.6 0.6 0.6]);
end

function [vis_panel, vis_bg] = create_visualization_panel(fig)
    % 创建可视化控制面板
    vis_panel = uipanel('Parent', fig, 'Title', '可视化控制', ...
        'Position', [0.05 0.39 0.9 0.07], ...
        'FontSize', 10, 'FontWeight', 'bold');
    
    vis_bg = uibuttongroup('Parent', vis_panel, ...
        'Position', [0.05 0.15 0.9 0.7], ...
        'BorderType', 'none', ...
        'Visible', 'on');  % 初始可见，由回调函数控制
    
    uicontrol(vis_bg, 'Style', 'radiobutton', ...
        'String', '实时可视化', ...
        'Position', [20 5 100 25], ...
        'FontSize', 9, 'Value', 1);
    
    uicontrol(vis_bg, 'Style', 'radiobutton', ...
        'String', '无可视化', ...
        'Position', [130 5 100 25], ...
        'FontSize', 9);
end

function [param_panel, param_groups] = create_parameter_panel(fig)
    % 创建参数设置面板
    param_panel = uipanel('Parent', fig, 'Title', '参数设置', ...
        'Position', [0.05 0.08 0.9 0.29], ...
        'FontSize', 10, 'FontWeight', 'bold');
    
    param_groups = struct();
    
    % PMF参数组
    [param_groups.pmf_group, pmf_params] = create_pmf_parameter_group(param_panel);
    param_groups = merge_structures(param_groups, pmf_params);
    
    % PF参数组
    [param_groups.pf_group, pf_params] = create_pf_parameter_group(param_panel);
    param_groups = merge_structures(param_groups, pf_params);
    
    % Hybrid参数组
    [param_groups.hybrid_group, hybrid_params] = create_hybrid_parameter_group(param_panel);
    param_groups = merge_structures(param_groups, hybrid_params);
    
    % RBPF参数组
    [param_groups.rbpf_group, rbpf_params] = create_rbpf_parameter_group(param_panel);
    param_groups = merge_structures(param_groups, rbpf_params);
end

function [pmf_group, pmf_params] = create_pmf_parameter_group(parent)
    % 创建PMF参数组
    pmf_group = uipanel('Parent', parent, 'Title', 'PMF参数', ...
        'Position', [0.02 0.02 0.96 0.96], 'FontSize', 9, 'Visible', 'on');
    
    pmf_params = struct();
    
    uicontrol('Parent', pmf_group, 'Style', 'text', 'String', 'Ngrid:', ...
        'Position', [20 140 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    pmf_params.pmf_params_Ngrid = uicontrol('Parent', pmf_group, 'Style', 'edit', ...
        'String', '50', 'Position', [110 140 80 22], 'FontSize', 9);
    
    uicontrol('Parent', pmf_group, 'Style', 'text', 'String', 'dx_pmf:', ...
        'Position', [20 105 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    pmf_params.pmf_params_dx_pmf = uicontrol('Parent', pmf_group, 'Style', 'edit', ...
        'String', '4', 'Position', [110 105 80 22], 'FontSize', 9);
    
    uicontrol('Parent', pmf_group, 'Style', 'text', 'String', '网格范围:', ...
        'Position', [20 70 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    pmf_params.pmf_params_grid_range = uicontrol('Parent', pmf_group, 'Style', 'edit', ...
        'String', '1000', 'Position', [110 70 80 22], 'FontSize', 9);
    
end

function [pf_group, pf_params] = create_pf_parameter_group(parent)
    % 创建PF参数组
    pf_group = uipanel('Parent', parent, 'Title', 'PF参数', ...
        'Position', [0.02 0.02 0.96 0.96], 'FontSize', 9, 'Visible', 'off');
    
    pf_params = struct();
    
    uicontrol('Parent', pf_group, 'Style', 'text', 'String', '深度门限:', ...
        'Position', [20 140 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    pf_params.pf_params_depthGatingThreshold = uicontrol('Parent', pf_group, 'Style', 'edit', ...
        'String', '2.0', 'Position', [110 140 80 22], 'FontSize', 9);
    
    uicontrol('Parent', pf_group, 'Style', 'text', 'String', '粒子数量:', ...
        'Position', [20 105 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    pf_params.pf_params_N_particles = uicontrol('Parent', pf_group, 'Style', 'edit', ...
        'String', '100', 'Position', [110 105 80 22], 'FontSize', 9);
    
    uicontrol('Parent', pf_group, 'Style', 'text', 'String', '重采样阈值:', ...
        'Position', [20 70 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    pf_params.pf_params_resampling_threshold = uicontrol('Parent', pf_group, 'Style', 'edit', ...
        'String', '0.5', 'Position', [110 70 80 22], 'FontSize', 9);
    
end

function [hybrid_group, hybrid_params] = create_hybrid_parameter_group(parent)
    % 创建Hybrid参数组
    hybrid_group = uipanel('Parent', parent, 'Title', 'Hybrid参数', ...
        'Position', [0.02 0.02 0.96 0.96], 'FontSize', 9, 'Visible', 'off');
    
    hybrid_params = struct();
    
    uicontrol('Parent', hybrid_group, 'Style', 'text', 'String', 'N_fixed:', ...
        'Position', [20 140 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    hybrid_params.hybrid_params_N_fixed = uicontrol('Parent', hybrid_group, 'Style', 'edit', ...
        'String', '25', 'Position', [110 140 80 22], 'FontSize', 9);
    
    uicontrol('Parent', hybrid_group, 'Style', 'text', 'String', 'N_random:', ...
        'Position', [20 110 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    hybrid_params.hybrid_params_N_random = uicontrol('Parent', hybrid_group, 'Style', 'edit', ...
        'String', '100', 'Position', [110 110 80 22], 'FontSize', 9);
    
    uicontrol('Parent', hybrid_group, 'Style', 'text', 'String', '重采样阈值:', ...
        'Position', [20 80 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    hybrid_params.hybrid_params_resampling_threshold = uicontrol('Parent', hybrid_group, 'Style', 'edit', ...
        'String', '0.5', 'Position', [110 80 80 22], 'FontSize', 9);
    
    hybrid_params.hybrid_params_ts_label = uicontrol('Parent', hybrid_group, 'Style', 'text', ...
        'String', 'ts切换时间(s):', 'Position', [20 50 80 20], 'FontSize', 9, ...
        'HorizontalAlignment', 'right', 'Visible', 'off');
    hybrid_params.hybrid_params_ts = uicontrol('Parent', hybrid_group, 'Style', 'edit', ...
        'String', '50.0', 'Position', [110 50 80 22], 'FontSize', 9, 'Visible', 'off');
    
end

function [rbpf_group, rbpf_params] = create_rbpf_parameter_group(parent)
    % 创建RBPF参数组
    rbpf_group = uipanel('Parent', parent, 'Title', 'RBPF参数', ...
        'Position', [0.02 0.02 0.96 0.96], 'FontSize', 9, 'Visible', 'off');
    
    rbpf_params = struct();
    
    uicontrol('Parent', rbpf_group, 'Style', 'text', 'String', '粒子数量:', ...
        'Position', [20 105 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    rbpf_params.rbpf_params_N_particles = uicontrol('Parent', rbpf_group, 'Style', 'edit', ...
        'String', '100', 'Position', [110 105 80 22], 'FontSize', 9);
    
    uicontrol('Parent', rbpf_group, 'Style', 'text', 'String', '重采样阈值:', ...
        'Position', [20 70 80 20], 'FontSize', 9, 'HorizontalAlignment', 'right');
    rbpf_params.rbpf_params_resampling_threshold = uicontrol('Parent', rbpf_group, 'Style', 'edit', ...
        'String', '0.5', 'Position', [110 70 80 22], 'FontSize', 9);
    
end

function [status_text, run_button] = create_control_panel(fig)
    % 创建控制面板
    status_text = uicontrol('Parent', fig, 'Style', 'text', ...
        'String', '就绪', 'Position', [20 45 300 25], ...
        'FontSize', 9, 'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.94 0.94 0.94]);
    
    run_button = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
        'String', '运行仿真', 'Position', [330 25 120 40], ...
        'FontSize', 10, 'FontWeight', 'bold');
end

function setup_callbacks(ui_components, param_handles)
    % 设置所有回调函数
    
    % 获取算法信息
    algorithms = getappdata(ui_components.algo_panel, 'algorithms');
    checkbox_names = getappdata(ui_components.algo_panel, 'checkbox_names');
    
    % 为每个复选框设置回调
    for i = 1:length(checkbox_names)
        checkbox_name = checkbox_names{i};
        if isfield(ui_components.algo_checkboxes, checkbox_name)
            set(ui_components.algo_checkboxes.(checkbox_name), 'Callback', ...
                @(src, event) checkbox_changed_callback(src, event, ui_components, algorithms));
        end
    end
    
    % 运行仿真回调
    set(ui_components.run_button, 'Callback', ...
        @(src, event) run_simulation_callback(src, event, ui_components, param_handles, algorithms));
    
    % 设置初始状态 - 默认选中PMF，应该是单算法模式
    checkbox_changed_callback([], [], ui_components, algorithms);
end


function checkbox_changed_callback(~, ~, ui_components, ~)
    % 复选框状态变化回调函数
    try
        % 获取当前选中的算法
        [selected_indices, selected_algos] = get_selected_algorithms(ui_components.algo_checkboxes, ui_components.algo_panel);
        
        if isempty(selected_indices)
            % 没有选择任何算法，保持至少选中一个
            warning('至少需要选择一个算法');
            return;
        end
        
        % 根据选择算法数量自动调整UI
        
        % 可视化面板始终显示
        ui_components.vis_bg.Visible = 'on';
        
        if isscalar(selected_indices)
            % 单算法模式：显示对应算法参数
            selected_algo = selected_algos{1};
            
            % 更新算法参数界面可见性
            gui_helpers('update_algorithm_visibility', selected_algo, ...
                ui_components.pmf_group, ui_components.pf_group, ...
                ui_components.hybrid_group, ui_components.rbpf_group, ...
                ui_components.hybrid_params_ts_label, ui_components.hybrid_params_ts);
        else
            % 多算法比较模式：隐藏算法参数
            % 隐藏所有算法参数面板
            set([ui_components.pmf_group, ui_components.pf_group, ...
                 ui_components.hybrid_group, ui_components.rbpf_group], 'Visible', 'off');
        end
        
    catch ME
        warning('复选框状态变化时出错: %s', getReport(ME, 'basic'));
    end
end

function run_simulation_callback(~, ~, ui_components, param_handles, algorithms)
    % 运行仿真回调函数 - 重构后的精简版本
    
    % 禁用按钮
    gui_helpers('safe_button_operation', ui_components.run_button, 'disable', '运行中...', '');
    
    try
        % 1. 验证参数和算法选择
        [is_valid, error_msg] = validate_inputs(ui_components, param_handles);
        if ~is_valid
            show_error_dialog('参数验证失败', error_msg);
            return;
        end
        
        % 2. 准备仿真配置
        [selected_algos, selected_display_names, base_params, gui_params] = ...
            prepare_simulation_config(ui_components, param_handles, algorithms);
        
        % 3. 创建状态更新回调
        status_callback = gui_helpers('create_status_callback', ui_components.status_text);
        
        % 4. 算法已在prepare_simulation_config中获取，直接执行仿真
        
        % 5. 执行仿真
        status_callback('开始仿真...');
        [all_results, status] = simulation_engine(selected_algos, selected_display_names, ...
                                                  base_params, gui_params, param_handles, status_callback);
        
        % 6. 处理结果
        if status.success
            mc_runs = str2double(get(param_handles.mc_runs_edit, 'String'));
            results_dashboard(all_results, base_params, mc_runs);
            status_callback('仿真完成');
        else
            show_error_dialog('仿真执行失败', status.error_msg);
        end
        
    catch ME
        error_msg = sprintf('仿真过程发生意外错误:\n%s\n\n位置: %s', ...
                           ME.message, get_error_location(ME));
        show_error_dialog('意外错误', error_msg);
        ui_components.status_text.String = '运行出错';
    end
    
    % 恢复按钮
    gui_helpers('safe_button_operation', ui_components.run_button, 'enable', '', '运行仿真');
end

%% 辅助函数

function [selected_indices, selected_algos] = get_selected_algorithms(algo_checkboxes, algo_panel)
    % 获取复选框选择状态，返回选中的算法索引和名称
    algorithms = getappdata(algo_panel, 'algorithms');
    checkbox_names = getappdata(algo_panel, 'checkbox_names');
    
    % 预分配数组
    num_checkboxes = length(checkbox_names);
    selected_indices = zeros(1, num_checkboxes);
    selected_algos = cell(1, num_checkboxes);
    count = 0;
    
    for i = 1:num_checkboxes
        checkbox_name = checkbox_names{i};
        if isfield(algo_checkboxes, checkbox_name) && get(algo_checkboxes.(checkbox_name), 'Value')
            count = count + 1;
            selected_indices(count) = i;
            selected_algos{count} = algorithms{i};
        end
    end
    
    % 截取有效部分
    selected_indices = selected_indices(1:count);
    selected_algos = selected_algos(1:count);
end

function merged = merge_structures(struct1, struct2)
    % 合并两个结构体
    merged = struct1;
    fields = fieldnames(struct2);
    for i = 1:length(fields)
        merged.(fields{i}) = struct2.(fields{i});
    end
end

function [is_valid, error_msg] = validate_inputs(ui_components, param_handles)
    % 验证所有输入参数
    is_valid = true;
    error_msg = '';
    
    % 验证算法选择
    [selected_indices, ~] = get_selected_algorithms(ui_components.algo_checkboxes, ui_components.algo_panel);
    [valid, msg] = gui_helpers('validate_algorithm_selection', selected_indices);
    if ~valid
        is_valid = false;
        error_msg = [error_msg, msg, '\n'];
    end
    
    % 验证GUI参数
    algorithms = getappdata(ui_components.algo_panel, 'algorithms');
    gui_params = gui_helpers('get_gui_params', ui_components.algo_checkboxes, ui_components.algo_panel, algorithms, ...
        ui_components.model_bg, ui_components.algo_seed_mode_popup, ui_components.sim_time_edit, ui_components.vis_bg);
    
    [valid, msg, ~] = gui_validator(gui_params, param_handles);
    if ~valid
        is_valid = false;
        error_msg = [error_msg, msg];
    end
end

function [selected_algos, selected_display_names, base_params, gui_params] = ...
         prepare_simulation_config(ui_components, ~, algorithms)
    % 准备仿真配置
    
    % 获取选中的算法
    [selected_indices, selected_algos] = get_selected_algorithms(ui_components.algo_checkboxes, ui_components.algo_panel);
    algorithm_display_names = getappdata(ui_components.algo_panel, 'display_names');
    selected_display_names = algorithm_display_names(selected_indices);
    
    % 获取GUI参数
    gui_params = gui_helpers('get_gui_params', ui_components.algo_checkboxes, ui_components.algo_panel, algorithms, ...
        ui_components.model_bg, ui_components.algo_seed_mode_popup, ui_components.sim_time_edit, ui_components.vis_bg);
    
    % 生成基础参数配置
    base_params = config_params(gui_params);
end

function show_error_dialog(title, message)
    % 显示错误对话框
    try
        errordlg(message, title, 'modal');
    catch
        % 如果GUI显示失败，则输出到命令窗口
        fprintf('\n错误: %s\n%s\n', title, message);
    end
end

function location = get_error_location(ME)
    % 获取错误位置信息
    if ~isempty(ME.stack)
        location = sprintf('%s (行 %d)', ME.stack(1).name, ME.stack(1).line);
    else
        location = '未知位置';
    end
end