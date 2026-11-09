# 数据与提交材料

Git目录收录可运行源码、数值输入、结果图和审计摘要，不包含大型原始MAT、课程Word/PDF、含个人联络信息的教师说明或完整厂家CAD。
完整原始数据与原生机械模型已保留在本地教师提交包，当前尚未创建v2 GitHub Release或上传附件；不要把此说明当作下载链接。

原始七点巡检mil.mat：618420207 bytes，SHA-256 `238f2679677fc104b7945e576d72d744c58dc04b47949fa0cb9715caec399b61`。
仅有摘要JSON不能执行独立重放；需要该原档及对应源码快照。可按主页命令重新生成新的航次并独立审计。
既有数值仅证明声明工况，不保证不同MATLAB/硬件的逐位一致性。

后续发布附件候选（当前仅本地准备）：
- 机械提交包：SHA-256 `7e661e25e99acb2d264115a2485e9729f76265d015c70b33bb85c2deca852e2e`。
- 控制代码与完整原档包：SHA-256 `ab99fbff29564b5dab1a953c1d97fe54f0e379ce7cf87a4180252db81722c55f`。

公开JSON中的本机绝对来源路径已替换为local-origin标识；这不改变物理参数、命令或指标。
原创MATLAB与CSV运行文件和教师提交版相同。文件校验清单见mission_3d/PUBLIC_SOURCE_MANIFEST.json。

`baseline_v4/PACKAGE_MANIFEST.json`、`EXECUTION_ASSETS_SHA256.json`及其中docs的清单为历史校验清单，保留原阶段记录；当前公开目录以PUBLIC_SOURCE_MANIFEST.json为准。
