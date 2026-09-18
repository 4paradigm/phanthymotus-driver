# PR #154 同步上游计划

## 范围

将官方 main 合并到 PR #154，保留提交历史，不部署、不操作机器人。
解决 G1 相机配置、卡片清单、RealSense 生命周期与 bundle 退出顺序冲突。
保留上游 vision_capture 和本 PR 的导航、RGB/Depth 自描述帧能力。

## 实施与验证

1. 核对 PR head、上游 main 和干净工作树，以 merge 提交同步。
2. 相机同时保留状态队列与 RGB 缓存订阅，停止时两者均清理，重启重建缓存。
3. bundle 先停止 loco，再停止录像消费者，最后停止相机；单插件异常不阻断后续清理。
4. 保留子进程原子日志初始化，运行全部 G1 本地测试和差异检查。
5. 推送原 PR 分支，核对远端 head 和冲突状态，再发送 /request_bot_review。

## 文档核对

合并后的 README/README_zh 保留导航自描述帧说明；上游拍照录像能力在
driver.yaml 清单与 config.yaml 中保留。未更改 topic、schema 或对外动作语义，
无需重写既有接口说明。验证仅为本地单元/契约测试，不代表真机验收。
