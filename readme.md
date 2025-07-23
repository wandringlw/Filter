# Filter Platform

A MATLAB-based navigation and positioning filter algorithm testing platform for surface vessels.

*This document provides comprehensive guidance for working with this codebase.*

## 快速启动 | Quick Start

```matlab
main    % 推荐启动方式 | Recommended way to launch
```

## 项目概述

这是一个水面航行器导航定位的滤波器算法测试平台，使用MATLAB开发。平台实现了多种先进的滤波算法，结合地图深度测量和传感器数据，在复杂水流环境下实现高精度的航行器位置估计和轨迹跟踪。该平台为滤波算法研究提供了完整的仿真环境，支持多种算法的性能对比和统计分析。

## 使用说明

### 测试和调试
```matlab
% 单算法详细调试（启用实时可视化）
% 在GUI中选择一个算法，系统自动进入单算法分析模式

% 算法性能对比
% 在GUI中选择多个算法，系统自动进入算法比较模式

% Monte Carlo统计验证
% 在GUI中设置MC次数 > 1
```

## 核心架构

### 系统分层架构
```
用户界面层 (GUI)        ← gui/main_window.m (主窗口控制)
├── 算法执行层           ← Algorithms/ (6种滤波算法)
├── 数据处理层           ← simulation/ (轨迹生成、测量仿真)
├── 配置管理层           ← core/config_params.m
├── 可视化分析层         ← gui/detailed_analysis.m
└── 仿真控制层           ← main.m, core/simulation_engine.m
```

### 关键组件详解

#### 1. GUI控制系统 (gui/main_window.m)
**核心特性**：
- **多模式支持**: 单算法模式(详细分析) vs 算法比较模式(性能对比)
- **智能参数管理**: 动态显示算法特定参数界面
- **种子控制策略**: 轨迹种子(每次MC自增100) + 算法种子(固定/随机)
- **Monte Carlo仿真**: 支持多次重复运行和统计分析
- **实时进度显示**: 完整的仿真状态监控和错误处理

**参数界面设计**：
- PMF参数: Ngrid, dx_pmf, grid_range
- PF参数: 深度门限, 粒子数量, 重采样阈值  
- Hybrid参数: N_fixed, N_random, 切换时间ts
- RBPF参数: 粒子数量, 重采样阈值

#### 2. 滤波算法模块 (Algorithms/)
支持6种先进滤波算法:

**算法统一接口**：
```matlab
[est_data, dr_data, exec_time] = Algorithm(params, true_data, meas_data, map_data)
```

- **PMF**: 点质量滤波器 (PointMassFilter.m) - 网格化贝叶斯滤波
- **Pure_PF**: 标准粒子滤波器 (ParticleFilter.m) - 全局KF+位置粒子
- **RBPF**: Rao-Blackwellized粒子滤波器 (RaoBlackwellizedParticleFilter.m) - 条件最优估计
- **Hybrid**: 混合粒子滤波器 (HybridParticleFilter.m) - 固定+随机粒子
- **Hybrid2**: 增强混合滤波器 (HybridParticleFilter2.m) - 改进固定粒子管理
- **Hybrid3**: 自适应混合滤波器 (HybridParticleFilter3.m) - 时间自适应策略

#### 3. 配置管理系统 (core/config_params.m)
**分层参数架构**：
- **基础仿真参数**: 时间步长、噪声水平、初始状态
- **模型特定参数**: 耦合/解耦模型配置
- **算法特定参数**: 每种算法的专用参数
- **GUI参数覆盖**: 支持运行时动态配置

#### 4. 数据生成与处理 (simulation/)
**核心模块**：
- **generate_true_trajectory.m**: 生成考虑水流影响的真实轨迹
- **simulate_current_field.m**: 水流场仿真(背景流+高斯涡旋)
- **add_depth_measurements.m**: 基于地图插值生成深度测量

**地图数据**：`map.mat` 存储在 `simulation/` 文件夹中，由 `core/simulation_engine.m` 中的 `load_map_data()` 函数加载

#### 5. 可视化分析系统 (gui/detailed_analysis.m)
**6子图综合分析**：
- 轨迹对比图: 真实vs估计轨迹叠加显示
- X/Y位置误差: 时间序列误差分析
- 速度/航向误差: 估计精度评估
- 总位置误差: 误差分布统计
- 权重方差: 滤波器置信度指示
- 扩展空间: 预留算法特定诊断

### 数据流架构

```
GUI参数配置 → config_params → 统一参数结构
        ↓
真实轨迹生成 → true_data {time, x, y, v, phi, u_c, v_c, control_*}
        ↓
传感器测量 → meas_data {v_imu, phi_imu, x_pos, y_pos, depth}
        ↓
地图数据 → map_data {depth, xVec, yVec}
        ↓
滤波算法执行 → est_data {time, x, y, v, phi}
        ↓
可视化分析 → 6图分析 + RMSE统计
```

#### 关键数据结构
- **params**: 完整配置参数结构体
- **true_data**: 真实轨迹和控制输入  
- **meas_data**: 双频传感器测量(高频IMU + 低频GPS/深度)
- **map_data**: 海底地形深度地图
- **est_data**: 滤波估计结果和置信度

## 物理建模系统

### 船舶动力学模型
**4维状态空间**: X = [x, y, v, φ]ᵀ
- x, y: 东北坐标系下的位置 [m]
- v: 对水速度 [m/s] 
- φ: 相对正北方向的航向角 [rad]

### 运动学方程

#### 解耦模型（简化版）
```matlab
% 位置演化
x_{k+1} = x_k + v_k·cos(φ_k)·dt + w_x
y_{k+1} = y_k + v_k·sin(φ_k)·dt + w_y

% 速度和航向演化（独立随机游走）
v_{k+1} = v_k + w_v
φ_{k+1} = φ_k + w_φ
```

#### 耦合模型（完整版）
```matlab
% 位置演化（含水流）
x_{k+1} = x_k + [v_k·cos(φ_k) + u_c(x_k,y_k)]·dt + w_x
y_{k+1} = y_k + [v_k·sin(φ_k) + v_c(x_k,y_k)]·dt + w_y

% 速度和航向演化（含切变效应）
v_{k+1} = v_k + (a_c - k_d·v_k + C_v·∇·V)·dt + w_v
φ_{k+1} = φ_k + (δ_r/T_r + C_φ·∇×V)·dt + w_φ
```

### 水流场模型

#### 总水流组成
```matlab
水流速度：
u_c(x,y) = u_bg + u_vortex(x,y)  % 东向分量
v_c(x,y) = v_bg + v_vortex(x,y)  % 北向分量

涡旋水流：
u_vortex = -v_θ·(dy/r)  % 切向速度的东向投影
v_vortex = v_θ·(dx/r)   % 切向速度的北向投影
v_θ = A·exp(-r²/(2σ²))  % 高斯衰减涡旋

其中：
- (u_bg, v_bg): 背景恒定水流 [m/s]
- A: 涡旋强度 [m/s]
- (x_c, y_c): 涡旋中心坐标 [m]
- σ: 涡旋衰减半径 [m]
- r = √[(x-x_c)² + (y-y_c)²]: 到涡旋中心距离
- dx = x-x_c, dy = y-y_c: 相对位置
```

#### 水流切变效应
```matlab
散度（影响速度）：
∇·V = ∂u_c/∂x + ∂v_c/∂y

旋度（影响航向）：
∇×V = ∂v_c/∂x - ∂u_c/∂y

切变耦合参数：
- C_v = 0.02 [1/s]: 速度-散度耦合系数
- C_φ = 0.01 [1/s]: 航向-旋度耦合系数
```

## 传感器测量系统

### 双频传感器架构
- **高频更新** (10Hz, dt_kf=0.1s): IMU测量 (速度, 航向)
- **低频更新** (1Hz, dt_pf=1.0s): GPS位置 + 深度测量  
- **时间同步**: 高低频比例固定为10:1

### 测量方程

#### 高频IMU测量
```matlab
频率：10Hz (dt_kf = 0.1s)

速度测量：
z_v = v_true + n_v,   n_v ~ N(0, σ_v²)

航向测量：
z_φ = φ_true + n_φ,   n_φ ~ N(0, σ_φ²)

参数：
σ_v = 0.1 m/s    (速度测量噪声标准差)
σ_φ = 0.05 rad   (航向测量噪声标准差)
```

#### 低频位置测量
```matlab
频率：1Hz (dt_pf = 1.0s)

GPS位置测量：
z_x = x_true + n_x,   n_x ~ N(0, σ_pos²)
z_y = y_true + n_y,   n_y ~ N(0, σ_pos²)

深度测量：
z_depth = interp2(map, x_true, y_true) + n_depth
n_depth ~ N(0, σ_pos²)

参数：
σ_pos = 1.0 m    (位置测量噪声标准差)
```

#### 深度测量模型
```matlab
1. 地图插值：
   depth_true = interp2(xVecMap, yVecMap, depth_data, x, y, 'linear')

2. 测量生成：
   z_depth = depth_true + σ_pos · randn()

3. 似然函数：
   L(z_depth|x,y) = (1/√(2πσ_pos²)) · exp(-½((z_depth - h(x,y))/σ_pos)²)
```

## 滤波算法详细原理

### 1. 点质量滤波器（PMF）
**数学原理**: 状态空间网格化 + 贝叶斯递推

**算法实现**:
```matlab
1. 初始化网格：
   - 网格大小：Ngrid × Ngrid  
   - 网格分辨率：dx_pmf [m]
   - 初始概率：高斯分布 exp(-r²/(2σ²))

2. 预测步骤：
   对每个网格点(x_i, y_j)：
   - 使用全局KF估计v_k, φ_k
   - 位置传播：p(x_{k+1}|x_k, z_{1:k-1})
   - 高斯卷积处理过程噪声

3. 更新步骤：
   - 深度测量似然：L(z_k|x_k, y_k)  
   - 贝叶斯更新：p(x_k|z_{1:k}) ∝ L(z_k|x_k)p(x_k|z_{1:k-1})
   - 概率归一化：Σ p(x_i|z_{1:k}) = 1

4. 状态估计：
   - 加权平均：x̂_k = Σ x_i·p(x_i|z_{1:k})
```

**计算复杂度**: O(Ngrid²)

### 2. 纯粒子滤波器（PF）
**数学原理**: 粒子集合{x_i, w_i}表示后验分布

**算法创新**: 全局KF处理速度航向 + 粒子处理位置
```matlab
1. 初始化：
   - 粒子数：N_particles
   - 初始粒子：x_0^i ~ N(μ_0, Σ_0)
   - 初始权重：w_0^i = 1/N
   - 全局KF状态：[v_est, φ_est]

2. 预测步骤：
   全局KF更新：
   - 预测：v_pred, φ_pred = KF_predict()
   - 更新：v_est, φ_est = KF_update(z_imu)
   
   粒子传播：
   - x_k^i = x_{k-1}^i + v_est·cos(φ_est)·dt + w_x
   - y_k^i = y_{k-1}^i + v_est·sin(φ_est)·dt + w_y

3. 更新步骤：
   - 权重更新：w_k^i = w_{k-1}^i · L(z_depth|x_k^i, y_k^i)
   - 深度门限：|z_depth - h(x_k^i, y_k^i)| < threshold·σ
   - 权重归一化：w_k^i = w_k^i / Σ w_k^j

4. 重采样：
   - 有效粒子数：N_eff = 1 / Σ (w_k^i)²
   - 条件：N_eff < resampling_threshold × N_particles
   - 方法：系统重采样(systematic resampling)
```

### 3. Rao-Blackwellized粒子滤波器（RBPF）
**数学原理**: 状态空间分解 + 条件最优估计

**状态分解策略**:
```matlab
1. 状态分解：
   - 非线性状态：s_k = [x_k, y_k]ᵀ (位置)
   - 条件线性状态：θ_k = [v_k, φ_k]ᵀ (速度和航向)

2. 粒子-KF混合结构：
   每个粒子i维护：
   - 位置：s_k^i = [x_k^i, y_k^i]ᵀ
   - 条件KF：θ_k^i|s_k^i ~ N(μ_k^i, Σ_k^i)

3. 递推算法：
   位置传播：
   - s_k^i = f(s_{k-1}^i, θ_{k-1}^i) + w_s
   
   条件KF更新：
   - 预测：μ_k^i = F·μ_{k-1}^i + B·u_k  
   - 更新：μ_k^i += K·(z_imu - H·μ_k^i)
   - 协方差：Σ_k^i = (I - K·H)·Σ_{k-1}^i

4. 权重更新：
   - 边际似然：w_k^i ∝ L(z_depth|s_k^i)
   - 最终估计：p(s_k|z_{1:k}) = Σ w_k^i·δ(s_k - s_k^i)
```

**理论优势**: 线性子空间的最优估计，降维处理

### 4. 混合粒子滤波器（Hybrid PF）
**设计思想**: 固定粒子 + 随机粒子的混合策略

**Hybrid PF 1 (基础版)**:
```matlab
1. 粒子初始化：
   - 固定粒子：网格布局，N_fixed = 25
   - 随机粒子：高斯采样，N_random = 100
   - 粒子标记：isFixed标志位

2. 差异化传播：
   固定粒子：
   - 仅跟随中心估计平移，无随机扰动
   - pos_fixed^i = pos_center + fixed_offset^i
   
   随机粒子：
   - 正常动力学传播 + 过程噪声
   - pos_random^i = f(pos_old^i) + noise

3. 统一更新：
   - 权重更新：所有粒子统一处理
   - 重采样：仅随机粒子参与，固定粒子保持
```

**Hybrid PF 2 (增强版)**:
```matlab
改进策略：
- 固定粒子周期性重定位
- 基于估计协方差调整固定粒子分布  
- 改进的权重平衡机制
```

**Hybrid PF 3 (自适应版)**:
```matlab
时间自适应策略：
t < ts: 固定粒子跟随航位推算(DR)
t ≥ ts: 固定粒子跟随滤波估计

策略切换：
1. 初期阶段 (t < ts)：
   fixed_pos^i = DR_pos + offset^i
   
2. 稳定阶段 (t ≥ ts)：  
   fixed_pos^i = filter_pos + offset^i

3. 切换处理：
   - 平滑过渡权重
   - 重新布局固定粒子网格
```

## 算法参数配置

### PMF参数
```matlab
核心参数：
- Ngrid: 网格数量 (默认50, 范围20-200)
- dx_pmf: 网格分辨率 (默认4.0m, 影响精度和计算量)
- grid_range: 网格覆盖范围 (默认1000m)

计算关系：
- 总网格点数 = Ngrid²  
- 覆盖区域 = grid_range × grid_range
- 内存需求 = O(Ngrid²)
- 计算复杂度 = O(Ngrid²)
```

### 粒子滤波参数
```matlab
核心参数：
- N_particles: 粒子数量 (默认100, 影响精度和计算量)
- depthGatingThreshold: 深度门限阈值 (默认2.0σ, 剔除异常测量)
- resampling_threshold: 重采样阈值 (默认0.5, N_eff阈值)

计算关系：
- Ne_eff_threshold = N_particles × resampling_threshold
- 重采样条件: N_eff < Ne_eff_threshold
- 内存需求 = O(N_particles)
- 计算复杂度 = O(N_particles)
```

### Hybrid滤波器参数
```matlab
核心参数：
- N_fixed: 固定粒子数 (默认25, 提供结构化采样)
- N_random: 随机粒子数 (默认100, 提供探索能力)
- fixed_offsets: 固定粒子偏移 (默认±80m, ±40m网格)
- ts: 策略切换时间 (Hybrid3专用, 默认50.0s)

计算关系：
- N_total = N_fixed + N_random
- 固定粒子布局 = √N_fixed × √N_fixed 网格
- 网格间距 = fixed_offsets间距
```

## 性能评估体系

### 精度评估指标

#### 位置精度
```matlab
RMSE位置 = √(1/N ∑_{k=1}^N [(x̂_k - x_k)² + (ŷ_k - y_k)²])

统计量：
- 均值误差：μ_error = mean(error_sequence)
- 标准差：σ_error = std(error_sequence)  
- 最大误差：max_error = max(abs(error_sequence))
- 收敛时间：t_conv (误差降至阈值以下的时间)
```

#### 速度和航向精度
```matlab
RMSE速度 = √(1/N ∑_{k=1}^N (v̂_k - v_k)²)

RMSE航向 = √(1/N ∑_{k=1}^N (φ̂_k - φ_k)²)

特殊处理：
- 航向角周期性：φ_error = wrapToPi(φ̂_k - φ_k)
- 单位统一：航向可转换为度进行显示
- 速度相对误差：relative_error = |v̂_k - v_k|/v_k
```

#### 置信度评估
```matlab
权重方差（粒子滤波器）：
weight_variance = var(weights)
confidence_indicator = 1 - weight_variance

有效粒子数：
N_eff = 1 / ∑_{i=1}^N (w_i)²
```

### 计算性能评估

#### 执行时间分析
```matlab
时间测量：
- 总执行时间：total_exec_time
- 平均单步时间：mean_step_time = total_time / N_steps
- 实时性因子：RT_factor = total_time / simulation_time

复杂度分析：
PMF: O(Ngrid²) 时间，O(Ngrid²) 空间
PF: O(N_particles) 时间，O(N_particles) 空间  
RBPF: O(N_particles × d_linear) 时间
Hybrid: O(N_total) 时间，O(N_total) 空间
```

### Monte Carlo统计分析
```matlab
统计量计算：
- 均值性能：mean_rmse = mean(rmse_array)
- 性能标准差：std_rmse = std(rmse_array)
- 置信区间：CI_95 = [quantile(rmse, 0.025), quantile(rmse, 0.975)]
- 性能分布：histogram(rmse_array)

算法对比：
- 相对性能：relative_perf = rmse_alg / rmse_baseline
- 显著性检验：t_test, wilcoxon_test
- 箱线图对比：boxplot([rmse1, rmse2, ...])
```

## 开发工作流

### 添加新算法
1. **算法实现**：在 `Algorithms/` 下创建新的 `.m` 文件
2. **接口规范**：实现统一接口 `[est_data, dr_data, exec_time] = NewAlgorithm(params, true_data, meas_data, map_data)`
3. **GUI集成**：在 `gui/main_window.m` 中添加到算法列表和参数面板
4. **参数配置**：在 `config_params.m` 中添加算法特定配置
5. **测试验证**：使用test.m进行初步验证

### 修改运动模型
1. **物理模型**：主要修改 `generate_true_trajectory.m` 中的状态演化方程
2. **环境模型**：如需新的水流模型，修改 `simulate_current_field.m`
3. **参数更新**：更新 `config_params.m` 中的模型参数
4. **算法适配**：检查各算法是否需要相应修改

### 调试和测试策略
1. **快速测试**：使用 `test.m` 进行基本功能验证
2. **单算法调试**：启用单算法模式和实时可视化，详细分析算法行为
3. **算法对比**：使用比较模式评估多算法性能差异
4. **统计验证**：使用Monte Carlo模式获得可靠的统计性能

### 参数调优指南
1. **PMF调优**：
   - 增加Ngrid提高精度，但计算量平方增长
   - 减小dx_pmf提高分辨率，需要相应增加grid_range
   
2. **粒子滤波调优**：
   - 增加粒子数提高精度和鲁棒性
   - 调整深度门限平衡精度和鲁棒性
   - 优化重采样阈值避免过度重采样
   
3. **混合滤波调优**：
   - 平衡固定和随机粒子比例
   - 调整固定粒子网格覆盖范围
   - 优化策略切换时间(Hybrid3)

## 系统运行控制

### GUI操作流程
1. **启动系统**：运行 `main` (推荐)
2. **模式选择**：单算法模式或算法比较模式
3. **算法配置**：选择算法并设置特定参数
4. **模型选择**：耦合模型或解耦模型
5. **种子管理**：设置轨迹种子和算法种子策略
6. **执行仿真**：点击"运行仿真"开始执行
7. **结果分析**：查看6图分析和性能统计

### 种子管理策略
```matlab
轨迹种子控制：
- 每次MC运行自动递增100，确保轨迹多样性  
- 可设置为'random'实现完全随机
- 固定数值确保可重复性

算法种子策略：
- 随机模式：每次MC使用不同种子，测试算法鲁棒性
- 固定模式：每次MC使用相同种子，确保公平比较
```

### 可视化控制
```matlab
显示策略：
- 单算法模式：强制启用实时可视化，便于详细分析
- 算法比较模式：可选择关闭可视化，提升计算速度
- Monte Carlo模式：自动关闭实时可视化，仅显示最终统计
```

### 错误处理机制
- **参数验证**：GUI自动检查参数合理性
- **数值稳定性**：权重归一化、协方差正定性检查
- **内存管理**：大规模仿真的内存监控
- **异常恢复**：算法发散检测和处理

## 地图数据要求

### 地图文件格式
```matlab
文件要求：
- 文件路径：simulation/map.mat
- 变量名：depth_data (深度数据矩阵)
- 数据类型：double矩阵
- 坐标系统：需要执行flipud()修正图像坐标

数据结构：
map_data = struct(
    'depth', depth_matrix,    % 深度数据 [m]
    'xVec', x_coordinates,    % X坐标向量 [m]  
    'yVec', y_coordinates     % Y坐标向量 [m]
);
```

### 地图规格
```matlab
分辨率：30米/像素
覆盖范围：根据仿真区域设定
插值方法：双线性插值 (interp2)
边界处理：外推或设置默认值
```

## 注意事项和最佳实践

### 数值稳定性
- **权重归一化**：粒子权重必须及时归一化
- **协方差正定**：KF协方差矩阵的正定性维护
- **角度处理**：使用wrapToPi处理角度周期性
- **除零保护**：插值和除法运算的安全检查

### 内存和性能优化
- **向量化计算**：充分利用MATLAB向量化特性
- **内存预分配**：大数组的预分配避免动态扩展
- **循环优化**：避免不必要的嵌套循环
- **并行计算**：Monte Carlo仿真可考虑并行化

### 实验设计原则
- **对照实验**：使用相同轨迹对比不同算法
- **统计显著性**：足够的Monte Carlo次数
- **参数公平性**：算法间参数设置的公平比较
- **结果可重现性**：合理的种子管理策略