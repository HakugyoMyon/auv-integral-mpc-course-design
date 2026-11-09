# Third-party notices

## Marine Systems Simulator (MSS)

- Author/maintainer: Thor I. Fossen and the MSS contributors.
- Source: https://github.com/cybergalactic/MSS
- Frozen upstream commit: `ed2ce7b4d8f44e3a724972499d8b5cd6189afc0e`.
- License: MIT; original text at `references/code/MSS/LICENSE`.
- This repository retains the snapshot's MATLAB `.m` files and original headers. Large documentation, example data, and demo models are omitted. It is a runtime source subset, not a complete distribution of MSS.
- The copied hydrodynamics functions are not presented as newly developed or newly identified by this project. Per-file checksums are in `reference_data/source_manifest.json`.

## MATLAB and Simulink

MATLAB, Simulink, and Optimization Toolbox are proprietary MathWorks products and are not included. Run with your own licensed installation. The Level-2 S-function adapters and the `.slx` model here are project implementations, not a redistributed MATLAB runtime.

## Figures and data

Repository figures and numeric references are generated from this project's simulations. No third-party platform photos, school logos, papers, course Word/PDF reports, credentials, or local development caches are distributed.


## 三维任务系统新增来源

当前mission_3d使用自编auv6工程物理模型，未以MSS原艇参数已辨识为依据。Dubins路径适配保留
mission_3d/baseline_v4/src/d15_dubins_candidates.m头部Andrew Walker的MIT许可。
T200静态数据来自Blue Robotics 2019工作簿16V表；原数据、转换核验及来源见该模块baseline_v4/provenance。
这是厂家静态系柱数据，非本项目实测或任意来流标定，不对制造商数据及CAD另行授予MIT许可。
根目录MIT适用于本项目原创软件；制造商件、数据及其他第三方材料按各自来源说明处理。
当前DWG/PDF为课程结构审查材料，不是承压、加工或实艇安全认证。
