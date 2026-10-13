# PiPER 计划二开发验证

日期：2026-10-13。分支：`codex/piper-monitor`。基线：`f5ca598278a3f015cd9503d674a81d2d75b122fa`。

## 结果

- 新增监控测试 20 项：通过。
- 指定回归集合：180 passed、114 subtests passed。
- 实际 Edge headless + 未修改 Core 渲染器：通过；528 个矩阵值对比，无页面脚本错误。
- 浏览器场景：三组实测角度模拟、过期冻结、读取失败提示、恢复、双设备隔离。
- ROS transport 使用假 ROS 对象验证 Core context、QoS、消息序列化、重复关闭及部分初始化失败释放。
- 官方源 URDF 与许可证原始字节 SHA256 核验：通过。
- 监控生命周期中假硬件记录的控制命令数：0。
- 未执行真实 CAN 读取、机械臂动作、模式切换、使能/失能、拍照或现场部署。

Python 3.12 回归命令（pytest、PyYAML、OpenCV 为测试环境依赖，不是本提交新增的驱动依赖）：

```bash
python -m pytest -q \
  tests/test_piper_monitor.py tests/test_piper_ten_cards.py \
  tests/test_piper_cards.py tests/test_piper_controls.py tests/test_piper_prepare.py \
  tests/test_piper_deployment.py tests/test_piper_mac.py tests/test_common_lifecycle.py \
  tests/test_dockerfile_copies.py tests/test_vendor_runtime_logging.py
```

初次回归环境缺少 PyYAML/OpenCV，补齐隔离测试环境后通过。首次打包检查发现新增模块间依赖需要
提前更改 Docker COPY，随后把模型导出合并进独立 `monitor.py`；最终打包检查通过，未修改 Dockerfile。

## 浏览器模拟证据

- 本机输出目录：`/Users/lei/Documents/ChatGPT/第四范式实习/acceptance/20261013_piper_monitor/`
- `monitor-simulation.png`：两台骨架及中文参数（明确标注模拟反馈）。
- `result.json`：场景、矩阵对比数、模型请求和脚本错误记录。
- 骨架渲染器 SHA256：`2f1f9f7856c36c1a019ad0cf46666fac41ed720b695c2be3d3aabac4967b14f4`。
- 测试中仅调用两次 loopback 假 MCP 的 `model`，没有请求现场 API。
- 证据截图及 Third-party JS 仅在本机，未提交到 Git；可用浏览器脚本重新生成。

## 待集成与用户测试

按 `MONITOR_INTEGRATION.md` 由计划一提供无控制副作用的 `read_monitor_snapshot()`、注册插件、
复制模块和资源、调整画布布局。此独立分支尚未把插件注册进现场服务；假反馈验收不能替代
集成后的 ROS2 联调或真机模型方向/尺寸核对。

用户要求先测试再提交 PR，因此本阶段保留本地独立提交，不创建或更新 PR。
最终目标：[驱动 PR #382](https://github.com/4paradigm/phanthymotus-driver/pull/382)。
本次监控版本：**尚未生成**。
查询时该 PR 历史最近成功 Bot Version 为 `release.261011.ec99be9`，对应历史构建，
不代表本次监控提交或当前 PR HEAD；提交 SHA 不作为 Version。
