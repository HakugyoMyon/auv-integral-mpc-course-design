# 检索与工程决策记录

## 静态分支范围
静态目标不采用动态目标丢失后的时间扩张包络。占据地图来自有时间戳的多俯仰角几何声呐扫描，静止占据长期保留；自由证书按捕获时刻估计位姿和误差预算转换。不是用环境真值直接规划。

## 已查主来源
- Morency 等，Use of a low-cost forward-looking sonar for collision avoidance in small AUVs, analysis and experimental results，https://arxiv.org/abs/2309.05785 。启示：感知与机动能力要联合考虑，有限波束不等于全空间已知。
- EROAS，https://arxiv.org/abs/2411.05516v3 。启示：转动/多俯仰扫描可补垂向信息；其执行器架构与本欠驱动AUV不同，没有移植其执行控制器。
- FIESTA，https://github.com/HKUST-Aerial-Robotics/FIESTA 。启示：增量地图和距离查询；本项目没有宣称复现其ESDF实现。
- Owen、Beard、McLain，Implementing Dubins Airplane Paths on Fixed-Wing UAVs，https://scholarsarchive.byu.edu/facpub/1900/ 。启示：空间曲率/爬升约束；本实现是有限S型弧/直线与有界变深库，并非完整Dubins airplane求解器。
- Viswanathan 等，Efficient Trajectory Library Filtering for Quadrotor Flight in Unknown Environments，IROS2020，https://publications.ri.cmu.edu/efficient-trajectory-library-filtering-for-quadrotor-flight-in-unknown-environments 。启示：用多候选轨迹库检查局部可行性；本实现未复制BiTE位运算，也不具有其性能结论。
- Tordesillas 等，FASTER: Fast and Safe Trajectory Planner for Navigation in Unknown Environments，https://arxiv.org/abs/2001.04420 。启示：区分未知名义尾部与已验证提交前缀，以及持续可行性的必要性。本实现没有已证明的终端安全停泊/制动备用轨迹，不能继承其安全保证。

## 首次随机多障碍失败及对应修改
S07：6球seed82001，70秒时拒绝继续，尚无碰撞，净距7.66m。已观察的远处障碍距约55m，所有候选的近端仍可行，但20m固定保持距离导致过早接回原航线，完整名义候选在N109/E-4附近与已知占据管相交。
修改只扩展候选保持距离为20/60/100m，继续执行原25m转弯半径、260m路线长度、全艇尺寸、导航/传感器误差、0.5m残余净距和实际MPC门控。不缩小误差预算，也不把未知空间视为空闲。S10用同seed检验是否真正解决。

## 撤回的试验配置
S08使用全局E[-20,20]航区，原任务Dubins返航几何校验即拒绝。R25掉头需要约50m横向尺度，另有航区余量，40m宽全局走廊不适合作为该任务垂向绕障正例。保留拒绝证据，不算控制失败，也不将不成立的任务条件包装成正例。

## 资格边界
采样碰撞审计与置信包络不是连续时间/实海确定性安全证明。无可行局部候选时停止的是仿真，不是已证明能够物理安全停住的AUV。未验证密集障碍全局完备规划和任意随机场必达性。

## 运行性能诊断 2026-11-09约10:58UTC
S19在变深后出现局部规划耗时大幅增加。用MATLAB官方Editor Pause定位到sta_map_query，在sta_local_plan/addCandidate→sta_route_check→历史帧角度计算中，不是求解器断开。暂时暂停后只调整MATLAB数值线程为1并继续原航次；未修改位姿、导航、账本、控制器或安全预算。原航次保留。独立lazy候选筛选器已准备，尚未替换或用于本航次。

## 已实施：可能占据与完整观测自由证书融合

旧guard是测距端点与位姿/离散误差形成的可能占据外包，不能解释为整球确定被实体占据。静态世界中，若完整观测体积F在全部声明预算成立时属于真实占据集O的补集，则从可能集合G移除F不丢失真实占据：`(G\F)∩O=G∩O`。据此增加独立fusion查询，经合成、真实传感器包驱动的随机球/有限柱体property检查以及失败快照对照后用于新命名航次；没有缩小艇体、导航界或残余净距。人工launch prior仍不覆盖guard；普通单条ray、概率miss和未知空间均不能代替完整球自由证书。

OctoMap主来源：https://octomap.github.io/octomap/doc/classoctomap_1_1OccupancyOcTreeBase.html ，原始论文：https://www.hrl.uni-bonn.de/papers/wurm10octomap.pdf/@@download/file 。必须区分：OctoMap概率更新中同帧occupied优先，以避免互删地面空洞；本实现不是引用其“free优先”规则，而是基于更强的完整体积条件证书与可能/确定占据的区别。统计包络同时成立、真实声学检测和动态环境均没有由该集合恒等式自动证明。

当前S27/S29/S30采用sta_map_query_fusion，完整到点返航及map/control/physics/nav/body预测管道等独立审计通过。有限property为3800查询、1309自由球、147可能guard冲突覆盖、0 false-free；有限检查不等于形式化安全证明。

## 已实施候选：收到的测高平面坡度与名义变深v2

基线四波束高度估计本就拟合局部海底平面，但原接口仅输出高度、丢弃坡度。新增sta_terrain_plane仅接收四波束报文、采样时刻估计位姿和传感器配置，按`R*(mount+beams.*ranges)`构造局部点并拟合平面；保留原有效性、残差和捕获时间，输出北东坡度及艇体中心处底深。坡度用时间常数2s平滑并将范数裁剪至0.07，仅用于名义路线预测；这不是实海坡度上界或防撞保证。无效帧不刷新last-valid，超过0.7s不可用于当前预测。

主来源：A Terrain-Following Control Method for Autonomous Underwater Vehicles with Single-Beam Sensor Configuration，JMSE 2024,12(3),366，https://www.mdpi.com/2077-1312/12/3/366 ，DOI:10.3390/jmse12030366。原文第2.2节利用波束推导的坡度生成高度修正，支持同时使用高度与局部坡度的工程动机。本项目只借鉴测量信息和参考预测，不复现其三单波束布局、反步控制器、稳定性结论或性能数据；实际运动控制仍为原多变量MPC。

S33为该测量平面/名义高度预测v2的新命名巡检实验，运行中且未形成最终report，不能提前声称巡检闭环通过。最终资格仍需原任务全部作业确认、返航以及独立原始数据审计，不通过则保留原v4并限定静态阶段的适用任务。
