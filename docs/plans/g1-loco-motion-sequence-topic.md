# G1 loco 提案 topic 调整

## 范围

按用户确认，将 G1 loco 唯一默认输入从
`/ubuntu/navigation/nav2/velocity_proposal` 改为
`/ubuntu/navigation/motion_sequence`。用户进一步确认端口改为 `motion_sequence`、
schema 改为 `phanthy.navigation.motion_sequence.v1`，拒绝旧 schema。
String 类型、消息字段、QoS 及执行逻辑不变。
不订阅旧 topic，不修改其他机型或 Core/ActuCore。用户已授权提交推送及北京 G1
部署准备；镜像构建完成后，非 Shadow Driver 容器由用户切换，再只读验活。
端口/schema 修复已通过本地验证，用户追加授权提交推送及北京 G1 部署准备。
切换前必须停止智能控制，发布端和 Canvas 同步迁移后才能恢复。

## 实施与验证

- 修改权威默认常量；loco 工具声明、订阅准入和安全门复用该常量。
- 同步配置注释、中英文 README 与旧连线迁移说明。
- 更新 topic 契约与 loco 生命周期测试，运行 G1 全量测试和差异检查。
- 补充旧 schema 拒绝测试，确认新 schema 接受且字段与安全检查不变。
- 发布端及 Canvas 由消费端同步调整；本地测试不代表真机链路验收。
- 端口/schema 修改后本地 G1 全量 unittest 271 项通过，`git diff --check`
  通过；中英文 README 已明确旧 topic/schema 迁移及切换前停止智能控制的要求。
