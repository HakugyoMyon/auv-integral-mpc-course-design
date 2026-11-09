# 三维任务模块

在本目录运行 `START_HERE`，或从仓库根目录运行同名入口。详见仓库主页。
运行依赖位于planning、perception、baseline_v4/src；baseline_v4不是旧二维MSS代码。
不要将evidence或历史v1源码用genpath混入路径。

已声明工况下的完整航次和GUI审计摘要见evidence，原始大MAT档不在Git历史中。
`sta_audit_job`需要原始mil.mat，不能仅用摘要JSON替代重放。
公开JSON只清理了本机绝对来源路径，保留数值字段；原创.m和CSV与本地提交版逐字节相同。
静态区域巡测、任意随机任务、强流组合和真实声学环境不在本轮通过范围。
