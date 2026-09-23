# PICO 设备实现（teleop_device）

本模块由独立 [PICO Driver](../../pico/pico/README.md) 加载，复用原生 OpenXR、WSS/RTC、安装配对和最新输入缓存。它不加载机器人模型，不求解 IK，不申请运动租约，不调用 ActuCore。实现依据为 [#329 两 Driver 契约](../../docs/plans/pico-teleop-device.md)。`ext_vr` 保留为内部源目录名，不再是公开卡片名。

## 公共接口

普通 MCP 工具 `teleop_device` 提供 `info/config/start/stop`，服务位于本机 15742。`info(instance_id)` 在未配置、未启动时也返回真实 DDS 路径；第一次配置初始化设备 HTTPS/WSS 15741，项目未启动也可下载与配对。一个服务只允许一个头显实例，第二实例明确拒绝。

| 方向 | topic | 格式/schema |
|---|---|---|
| 设备输出 | `/<namespace>/teleop/<instance>/command` | `data/teleop-cmd` / `motus.teleop.command/1` |
| 执行反馈 | 同一实例 `/feedback` | `data/teleop-state` / `motus.teleop.feedback/1` |

Canvas 只画设备到控制卡的正向连线；反向由双方从该输入绑定派生。协议校验复用 `common/teleop_contract.py`。DDS 使用本机 domain 42、RELIABLE / KEEP_LAST(16) / VOLATILE；应用层只有一个待发姿态和最多16个待发操作。独立 writer 隔离阻塞发布，满队列不会阻塞 RTC、配对服务或停止请求的受理。

输入沿用头显平铺字段，显式带 `kind=input`。OpenXR 转换为位置 `[-z,-x,y]`、四元数 `[-qz,-qx,qy,qw]` 并归一化，单位米，跟踪系 X前/Y左/Z上，不是机器人基座系。原始采样时钟仅用于源端顺序；接收时间带 Linux boot ID，心跳不刷新旧位姿。

**松握与再次双握都是输入数据，设备不标定。** 松握期间继续采集；重握首帧是最新当前位置，连接和空间代次不变。实际断连或 OpenXR 空间重置另行使旧空间失效。输入滤波默认关闭以便冻结对照；可设30ms进行独立A/B。滤波只作用位姿，不延迟握把、跟踪失效或停止。

## 操作与回执

PICO 的开始映射为 `begin`，结束为 `finish`，立即停止为 `stop`；保留 `calibrate` 协议能力供控制端显式使用。操作带独立 request_id 和原始5秒入站截止时间，100ms有界重试不更新时间。受理后等待反馈最终结果，完成状态不由显示或HTTP成功推断。60秒仍无最终反馈时报告结果未知，不重建会话、不自动释放或补发新开始。

操作缓存与姿态分开；停止可取消旧操作重试并优先发出。回执最多保留256项，普通动作淘汰后的ID仍防重放；幂等停止只按实际保留的回执精确去重，不受概率过滤器误判或普通请求容量阻塞。RTC跟踪丢失时，只要已认证WSS仍连接，停止仍可发送。连接代次变化后取消旧请求。回执按原操作的device_id/connection_epoch/space_epoch匹配，不依赖反馈顶层最后姿态的代次；旧姿态不能更新当前连接的可启动状态。反馈校验绑定、时钟、生产者代次和顺序，未知旧生产者不能恢复显示。

原生App内部沿用已验证的WSS操作/显示协议。默认关闭骨架，反馈只用于按钮和状态；头显透视、输入和停止不等待模型绘制。

## 配置与配对鉴权

齿轮配置包含安装确认（默认否）、设备名、Driver生成的安装地址、输入滤波以及敏感密码字段。普通Core不执行readOnly提示，Driver仍拒绝更改安装地址。未手动确认安装不开始采集。运行中拒绝修改，保存后原子写入并读回。生命周期修改串行，兼容Core保存后立即启动时的重复配置请求；采集和反馈不等待该锁。普通 Core 不需要专用Header、管理绑定或密钥注入。

配对管理页由Driver自身HTTPS托管。用户在齿轮设置至少12字符的管理密码；Driver只持久化PBKDF2哈希，不把密码或哈希返回info/DDS。网页登录得到15分钟Secure/HttpOnly/SameSite会话，写操作还验证同源与CSRF。下载只读；批准、撤销和邀请不可匿名执行。邀请单次、有期且只授权配对，不授权运动。没有配置密码时明确提示从齿轮设置，不生成需要找后台文件的秘密。

用户已接受首版在普通齿轮复制网址后打开Driver配对页，无需Core增量。隔离浏览器已验证真实Core字段渲染、保存、分享脱敏、启动/停止和专用格式的文字监控。PICO系统浏览器对独立自签TLS的首次信任、下载安装仍需设备验收，研发ADB不代替该流程。

## 复用与验证

原始来源、许可证及采用文件摘要见 [NOTICE](NOTICE.md)、[ADOPTED_SOURCE](ADOPTED_SOURCE.json)。客户端编译说明见 [原生App](openxr_capture_native/README.md)。本机Python tests覆盖实际localhost MCP、HTTPS、WSS/RTC、配置、认证、重握、latest缓存、堵塞writer和操作恢复；不是PICO、真实DDS或机器人验收。
