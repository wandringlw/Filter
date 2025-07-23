function [u_current, v_current, varargout] = simulate_current_field(x, y, params)
%SIMULATE_CURRENT_FIELD 仿真水流场函数
%   根据位置计算水流的u和v分量，可选计算水流场梯度
%
%   输入:
%   - x, y: 位置坐标 (标量或矩阵)
%   - params: 完整的参数结构体，包含params.current_field
%
%   输出:
%   - u_current: 水流u分量 (东向) [m/s]
%   - v_current: 水流v分量 (北向) [m/s]
%   - varargout{1}: 散度 (divergence = ∂u/∂x + ∂v/∂y) [可选] [1/s]
%   - varargout{2}: 旋度 (vorticity = ∂v/∂x - ∂u/∂y) [可选] [1/s]

%% 输入验证
if ~isfield(params, 'current_field')
    error('参数结构体必须包含 current_field 字段');
end

%% 计算基础水流
[u_current, v_current] = compute_current_velocity(x, y, params.current_field);

%% 计算梯度（如果需要）
if nargout >= 3
    % 检查是否需要计算梯度
    need_gradients = isfield(params, 'coupling') && ...
                    (params.coupling.C_v ~= 0 || params.coupling.C_phi ~= 0);
    
    if need_gradients && isscalar(x) && isscalar(y)
        % 获取梯度计算步长
        delta = params.coupling.gradient_delta;
        
        % 使用中心差分法计算梯度
        [divergence, vorticity] = compute_current_gradients(x, y, params.current_field, delta);
        
        varargout{1} = divergence;
        if nargout >= 4
            varargout{2} = vorticity;
        end
    else
        % 不需要梯度或输入是数组
        varargout{1} = 0;
        if nargout >= 4
            varargout{2} = 0;
        end
    end
end

end

%% 私有函数：计算水流速度
function [u, v] = compute_current_velocity(x, y, current_field)
%COMPUTE_CURRENT_VELOCITY 计算给定位置的水流速度
%   水流场模型：背景水流 + 涡旋水流

% 提取参数
vortex_strength = current_field.vortex_strength;    % 涡旋强度 [m/s]
vortex_center = current_field.vortex_center;        % 涡旋中心 [x_c, y_c] [m]
decay_radius = current_field.decay_radius;          % 衰减半径 [m]
background_current = current_field.background_current; % 背景水流 [u_bg, v_bg] [m/s]

%% 1. 背景水流（恒定）
u_bg = background_current(1);
v_bg = background_current(2);

%% 2. 涡旋水流（随距离衰减的旋转流）
if vortex_strength == 0
    % 无涡旋
    u_vortex = zeros(size(x));
    v_vortex = zeros(size(y));
else
    % 计算相对位置
    dx = x - vortex_center(1);
    dy = y - vortex_center(2);
    
    % 计算距离
    r = sqrt(dx.^2 + dy.^2);
    
    % 处理中心点奇异性
    r(r == 0) = eps;
    
    % 高斯衰减因子
    decay_factor = exp(-r.^2 / (2 * decay_radius^2));
    
    % 角向速度（逆时针为正）
    v_theta = vortex_strength * decay_factor;
    
    % 转换为笛卡尔坐标速度分量
    % 在极坐标中：v_r = 0, v_θ = v_theta
    % 在笛卡尔坐标中：
    %   u = v_r * cos(θ) - v_θ * sin(θ) = -v_θ * sin(θ) = -v_θ * (dy/r)
    %   v = v_r * sin(θ) + v_θ * cos(θ) = v_θ * cos(θ) = v_θ * (dx/r)
    u_vortex = -v_theta .* (dy ./ r);
    v_vortex = v_theta .* (dx ./ r);
    
    % 处理中心点
    center_mask = (r <= eps);
    u_vortex(center_mask) = 0;
    v_vortex(center_mask) = 0;
end

%% 3. 总水流 = 背景水流 + 涡旋水流
u = u_bg + u_vortex;
v = v_bg + v_vortex;

end

%% 私有函数：计算水流梯度
function [divergence, vorticity] = compute_current_gradients(x, y, current_field, delta)
%COMPUTE_CURRENT_GRADIENTS 使用中心差分法计算水流场梯度
%   计算散度和旋度

% 计算四个方向的水流值
[u_px, v_px] = compute_current_velocity(x + delta, y, current_field);  % +x方向
[u_mx, v_mx] = compute_current_velocity(x - delta, y, current_field);  % -x方向
[u_py, v_py] = compute_current_velocity(x, y + delta, current_field);  % +y方向
[u_my, v_my] = compute_current_velocity(x, y - delta, current_field);  % -y方向

% 中心差分法计算偏导数
du_dx = (u_px - u_mx) / (2 * delta);  % ∂u/∂x
du_dy = (u_py - u_my) / (2 * delta);  % ∂u/∂y
dv_dx = (v_px - v_mx) / (2 * delta);  % ∂v/∂x
dv_dy = (v_py - v_my) / (2 * delta);  % ∂v/∂y

% 计算散度和旋度
divergence = du_dx + dv_dy;  % 散度：∇·V = ∂u/∂x + ∂v/∂y
vorticity = dv_dx - du_dy;   % 旋度：∇×V = ∂v/∂x - ∂u/∂y (z分量)

end 