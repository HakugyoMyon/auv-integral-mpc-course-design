# 来源及第三方说明

本包是课程MIL代码，不附带MATLAB软件或许可证。静态依赖识别为 MATLAB R2024b、Control System Toolbox、Optimization Toolbox、Navigation Toolbox；实际许可证仍由使用者环境提供。运行入口使用当前MATLAB，不启动额外进程，不依赖MCP或CAD程序。

`src/d15_dubins_candidates.m` 是 Andrew Walker / Dubins-Curves 六种Dubins路径的MATLAB适配，保留源码中的完整MIT版权及许可声明。上游地址和固定版本见该文件头部。

当前非线性物理对象为 `auv6_*` 工程模型，带机械质量/浮力输入、附加质量、阻尼、四舵及推力执行器动态。它不是直接调用先前二维工程中的 `remus100.m`。水动力和执行器时常包含待辨识的假设，不能视作实艇标定参数。

`src/data/mass_budget.json`、`sensor_mounts.json` 以及根目录 `input_budget.json` 是本工程机械/导航接口的候选输入。内部出现的原CAD绝对路径是来源元数据，运行不读取那些CAD路径。

`src/data/T200_static_16V.csv` 来自 Blue Robotics 2019年公开性能数据工作簿的16 V工作表，共201行。已从机械阶段归档找回厂家原表，验证SHA-256，并逐项核对：六列原始数值不变，新增 `force_N=force_kgf×9.80665`，数值最大差异为0。原始工作簿、校验记录及核验脚本见 `provenance`。这是厂家零来流静态系柱推力数据，不是本项目实测或任意来流下的标定性能；不据此补充额外授权声明。

本阶段未重新发布冻结的动态避障仓库，也没有自动向GitHub推送。独立包源码及输入哈希见 `PACKAGE_MANIFEST.json`，任务证据与资格结果另附。
