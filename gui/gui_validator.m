function [is_valid, error_msg, validated_params] = gui_validator(gui_params, param_handles)
    % GUI参数验证器 - 提供详细的参数验证和错误提示
    % 输入:
    %   gui_params - GUI参数结构体
    %   param_handles - 参数控件句柄结构体
    % 输出:
    %   is_valid - 验证是否通过
    %   error_msg - 错误信息
    %   validated_params - 经过验证和修正的参数
    
    is_valid = true;
    error_msg = '';
    validated_params = gui_params;
    
    try
        %% 1. 基础参数验证
        
        % 仿真时间验证
        if isfield(gui_params, 'sim_time')
            [valid, msg, value] = validate_sim_time(gui_params.sim_time);
            if ~valid
                is_valid = false;
                error_msg = [error_msg, msg, '\n'];
            else
                validated_params.sim_time = value;
            end
        end
        
        % Monte Carlo次数验证
        if isfield(param_handles, 'mc_runs_edit')
            mc_str = get(param_handles.mc_runs_edit, 'String');
            [valid, msg, value] = validate_mc_runs(mc_str);
            if ~valid
                is_valid = false;
                error_msg = [error_msg, msg, '\n'];
            else
                validated_params.mc_runs = value;
            end
        end
        
        % 种子参数验证
        if isfield(param_handles, 'traj_seed_edit')
            traj_seed_str = get(param_handles.traj_seed_edit, 'String');
            [valid, msg, value] = validate_seed(traj_seed_str, '轨迹种子');
            if ~valid
                is_valid = false;
                error_msg = [error_msg, msg, '\n'];
            else
                validated_params.traj_seed = value;
            end
        end
        
        if isfield(param_handles, 'algo_seed_edit')
            algo_seed_str = get(param_handles.algo_seed_edit, 'String');
            [valid, msg, value] = validate_seed(algo_seed_str, '算法种子');
            if ~valid
                is_valid = false;
                error_msg = [error_msg, msg, '\n'];
            else
                validated_params.algo_seed = value;
            end
        end
        
        %% 2. 算法特定参数验证
        
        % PMF参数验证
        if isfield(param_handles, 'pmf_params')
            [valid, msg, params] = validate_pmf_params(param_handles.pmf_params);
            if ~valid
                is_valid = false;
                error_msg = [error_msg, msg, '\n'];
            else
                validated_params.pmf = params;
            end
        end
        
        % PF参数验证
        if isfield(param_handles, 'pf_params')
            [valid, msg, params] = validate_pf_params(param_handles.pf_params);
            if ~valid
                is_valid = false;
                error_msg = [error_msg, msg, '\n'];
            else
                validated_params.pf = params;
            end
        end
        
        % Hybrid参数验证
        if isfield(param_handles, 'hybrid_params')
            [valid, msg, params] = validate_hybrid_params(param_handles.hybrid_params);
            if ~valid
                is_valid = false;
                error_msg = [error_msg, msg, '\n'];
            else
                validated_params.hybrid = params;
            end
        end
        
        % RBPF参数验证
        if isfield(param_handles, 'rbpf_params')
            [valid, msg, params] = validate_rbpf_params(param_handles.rbpf_params);
            if ~valid
                is_valid = false;
                error_msg = [error_msg, msg, '\n'];
            else
                validated_params.rbpf = params;
            end
        end
        
    catch ME
        is_valid = false;
        error_msg = sprintf('参数验证过程中发生错误: %s', ME.message);
    end
    
    % 清理错误消息末尾的换行符
    if ~isempty(error_msg)
        error_msg = strtrim(error_msg);
    end
end

%% 辅助验证函数

function [is_valid, error_msg, value] = validate_sim_time(sim_time)
    is_valid = true;
    error_msg = '';
    value = sim_time;
    
    if isnan(sim_time) || ~isnumeric(sim_time)
        is_valid = false;
        error_msg = '仿真时间必须是数字';
        value = 1000; % 默认值
    elseif sim_time <= 0
        is_valid = false;
        error_msg = '仿真时间必须大于0';
        value = 1000; % 默认值
    elseif sim_time < 10
        is_valid = false;
        error_msg = '仿真时间太短，建议至少10秒';
        value = 100; % 建议值
    elseif sim_time > 10000
        is_valid = false;
        error_msg = '仿真时间过长，建议小于10000秒';
        value = 1000; % 建议值
    end
end

function [is_valid, error_msg, value] = validate_mc_runs(mc_str)
    is_valid = true;
    error_msg = '';
    
    value = str2double(mc_str);
    
    if isnan(value)
        is_valid = false;
        error_msg = 'Monte Carlo次数必须是数字';
        value = 1;
    elseif value <= 0 || value ~= round(value)
        is_valid = false;
        error_msg = 'Monte Carlo次数必须是正整数';
        value = 1;
    elseif value > 100
        is_valid = false;
        error_msg = 'Monte Carlo次数过大，建议不超过100次';
        value = 10;
    end
end

function [is_valid, error_msg, value] = validate_seed(seed_str, param_name)
    is_valid = true;
    error_msg = '';
    
    if strcmpi(seed_str, 'random')
        value = 'random';
        return;
    end
    
    value = str2double(seed_str);
    
    if isnan(value)
        is_valid = false;
        error_msg = sprintf('%s必须是数字或"random"', param_name);
        value = 'random';
    elseif value < 0 || value ~= round(value)
        is_valid = false;
        error_msg = sprintf('%s必须是非负整数或"random"', param_name);
        value = 'random';
    end
end

function [is_valid, error_msg, params] = validate_pmf_params(handles)
    is_valid = true;
    error_msg = '';
    params = struct();
    
    % Ngrid验证
    ngrid = str2double(get(handles.Ngrid, 'String'));
    if isnan(ngrid) || ngrid <= 0 || ngrid ~= round(ngrid)
        is_valid = false;
        error_msg = [error_msg, 'PMF网格数量必须是正整数; '];
        params.Ngrid = 50;
    elseif ngrid < 10
        is_valid = false;
        error_msg = [error_msg, 'PMF网格数量太小，建议至少10; '];
        params.Ngrid = 20;
    elseif ngrid > 200
        is_valid = false;
        error_msg = [error_msg, 'PMF网格数量过大，建议不超过200; '];
        params.Ngrid = 100;
    else
        params.Ngrid = ngrid;
    end
    
    % dx_pmf验证
    dx_pmf = str2double(get(handles.dx_pmf, 'String'));
    if isnan(dx_pmf) || dx_pmf <= 0
        is_valid = false;
        error_msg = [error_msg, 'PMF网格分辨率必须是正数; '];
        params.dx_pmf = 4.0;
    elseif dx_pmf > 50
        is_valid = false;
        error_msg = [error_msg, 'PMF网格分辨率过大，建议小于50m; '];
        params.dx_pmf = 10.0;
    else
        params.dx_pmf = dx_pmf;
    end
    
    % grid_range验证
    grid_range = str2double(get(handles.grid_range, 'String'));
    if isnan(grid_range) || grid_range <= 0
        is_valid = false;
        error_msg = [error_msg, 'PMF网格范围必须是正数; '];
        params.grid_range = 1000;
    elseif grid_range < 100
        is_valid = false;
        error_msg = [error_msg, 'PMF网格范围太小，建议至少100m; '];
        params.grid_range = 500;
    else
        params.grid_range = grid_range;
    end
end

function [is_valid, error_msg, params] = validate_pf_params(handles)
    is_valid = true;
    error_msg = '';
    params = struct();
    
    % 深度门限验证
    depth_threshold = str2double(get(handles.depthGatingThreshold, 'String'));
    if isnan(depth_threshold) || depth_threshold <= 0
        is_valid = false;
        error_msg = [error_msg, '深度门限阈值必须是正数; '];
        params.depthGatingThreshold = 2.0;
    elseif depth_threshold > 10
        is_valid = false;
        error_msg = [error_msg, '深度门限阈值过大，建议小于10; '];
        params.depthGatingThreshold = 5.0;
    else
        params.depthGatingThreshold = depth_threshold;
    end
    
    % 粒子数量验证
    n_particles = str2double(get(handles.N_particles, 'String'));
    if isnan(n_particles) || n_particles <= 0 || n_particles ~= round(n_particles)
        is_valid = false;
        error_msg = [error_msg, '粒子数量必须是正整数; '];
        params.N_particles = 100;
    elseif n_particles < 10
        is_valid = false;
        error_msg = [error_msg, '粒子数量太少，建议至少10个; '];
        params.N_particles = 50;
    elseif n_particles > 1000
        is_valid = false;
        error_msg = [error_msg, '粒子数量过多，建议不超过1000个; '];
        params.N_particles = 500;
    else
        params.N_particles = n_particles;
    end
    
    % 重采样阈值验证
    resampling_threshold = str2double(get(handles.resampling_threshold, 'String'));
    if isnan(resampling_threshold) || resampling_threshold <= 0 || resampling_threshold > 1
        is_valid = false;
        error_msg = [error_msg, '重采样阈值必须在(0,1]范围内; '];
        params.resampling_threshold = 0.5;
    else
        params.resampling_threshold = resampling_threshold;
    end
end

function [is_valid, error_msg, params] = validate_hybrid_params(handles)
    is_valid = true;
    error_msg = '';
    params = struct();
    
    % N_fixed验证
    n_fixed = str2double(get(handles.N_fixed, 'String'));
    if isnan(n_fixed) || n_fixed < 0 || n_fixed ~= round(n_fixed)
        is_valid = false;
        error_msg = [error_msg, '固定粒子数必须是非负整数; '];
        params.N_fixed = 25;
    elseif n_fixed > 100
        is_valid = false;
        error_msg = [error_msg, '固定粒子数过多，建议不超过100个; '];
        params.N_fixed = 50;
    else
        params.N_fixed = n_fixed;
    end
    
    % N_random验证
    n_random = str2double(get(handles.N_random, 'String'));
    if isnan(n_random) || n_random <= 0 || n_random ~= round(n_random)
        is_valid = false;
        error_msg = [error_msg, '随机粒子数必须是正整数; '];
        params.N_random = 100;
    elseif n_random > 10000
        is_valid = false;
        error_msg = [error_msg, '随机粒子数过多，建议不超过10000个; '];
        params.N_random = 500;
    else
        params.N_random = n_random;
    end
    
    % 重采样阈值验证
    resampling_threshold = str2double(get(handles.resampling_threshold, 'String'));
    if isnan(resampling_threshold) || resampling_threshold <= 0 || resampling_threshold > 1
        is_valid = false;
        error_msg = [error_msg, '重采样阈值必须在(0,1]范围内; '];
        params.resampling_threshold = 0.5;
    else
        params.resampling_threshold = resampling_threshold;
    end
    
    % ts参数验证（如果存在）
    if isfield(handles, 'ts') && ishandle(handles.ts)
        ts_value = str2double(get(handles.ts, 'String'));
        if isnan(ts_value) || ts_value <= 0
            is_valid = false;
            error_msg = [error_msg, '切换时间ts必须是正数; '];
            params.ts = 50.0;
        else
            params.ts = ts_value;
        end
    end
end

function [is_valid, error_msg, params] = validate_rbpf_params(handles)
    is_valid = true;
    error_msg = '';
    params = struct();
    
    % 粒子数量验证
    n_particles = str2double(get(handles.N_particles, 'String'));
    if isnan(n_particles) || n_particles <= 0 || n_particles ~= round(n_particles)
        is_valid = false;
        error_msg = [error_msg, 'RBPF粒子数量必须是正整数; '];
        params.N_particles = 100;
    elseif n_particles < 10
        is_valid = false;
        error_msg = [error_msg, 'RBPF粒子数量太少，建议至少10个; '];
        params.N_particles = 50;
    elseif n_particles > 500
        is_valid = false;
        error_msg = [error_msg, 'RBPF粒子数量过多，建议不超过500个; '];
        params.N_particles = 200;
    else
        params.N_particles = n_particles;
    end
    
    % 重采样阈值验证
    resampling_threshold = str2double(get(handles.resampling_threshold, 'String'));
    if isnan(resampling_threshold) || resampling_threshold <= 0 || resampling_threshold > 1
        is_valid = false;
        error_msg = [error_msg, '重采样阈值必须在(0,1]范围内; '];
        params.resampling_threshold = 0.5;
    else
        params.resampling_threshold = resampling_threshold;
    end
end