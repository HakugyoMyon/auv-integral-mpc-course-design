# 数值参考数据

`v3/` 的 24 个 MAT 文件分别对应 12 个工况和 PID / MPC_v3，来自原始冻结实验。它们仅保留数值结构 `reference`：

| 字段 | 含义 |
|---|---|
| `t` | 控制采样时刻，s |
| `z` | 每行 15 状态：[u v w p q r N E D phi theta psi 实际方向舵 实际升降舵 螺旋桨RPM] |
| `command` | 方向舵、升降舵、螺旋桨转速指令 |
| `reference` | 原仿真记录的参考信号数组；字段定义以 `simulate_active_control.m` 为准 |
| `ey` | 真实横向路径误差，m |
| `C` | 数值控制配置结构 |

角度 rad，角速度 rad/s，位置 m，速度 m/s，转速 RPM。无函数句柄、机器路径、优化器对象或个人身份字段。导出脚本使用 `isequaln` 检查重新加载的数值与导出源相同。`numeric_reference_sha256.json` 记录这些文件的 SHA-256；`source_manifest.json` 记录冻结内核、模型和 MSS 源码校验值。
