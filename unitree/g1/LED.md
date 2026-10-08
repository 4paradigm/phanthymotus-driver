# G1 LED 持续颜色与周期控制

扩展现有 `led` 卡片，不新增第二个竞争同一灯带的卡片。原 `device.LedPlugin`
导入路径保留，实现迁至 `led_control.py`；Dockerfile 增加该源文件的 COPY，
不引入新的依赖或容器。`main.py`、config 中原 LED 开关及 hooks 继续使用。

## 动作

| action | 参数 | 行为 |
|---|---|---|
| start | 无 | 启动输出线程，空闲时不发灯光命令；不开始周期 |
| set | r, g, b, refresh_hz | 持续显示全灯带单色，替换当前周期 |
| cycle | sequence, repeat_count, refresh_hz, end_behavior | 异步启动颜色序列；重复调用会从头替换旧序列 |
| pause / interrupt | 无 | 暂停周期计时，继续刷新当前颜色；可 resume |
| resume | 无 | 从暂停位置继续，不计算暂停期间的时间 |
| off | 无 | 取消周期，按默认 10 Hz 持续输出黑色，保持熄灯 |
| stop | 无 | 终止周期、停止所有刷新并尝试发送一次黑色；不可 resume |
| info | 无 | 查询状态，不发硬件命令 |
| state | state | 原状态灯：idle/hearing/thinking/speaking/error |

`set/cycle/off` 可自动启动已停止的服务。新参数通过完整校验后才替换旧任务。
`start` 本身不会替换正在运行的任务。

- RGB 均为整数 0–255，控制整条灯带，不支持逐灯珠寻址。
- `sequence` 支持原生数组或画布文本框中的 JSON 数组文本。每阶段必填
  `r/g/b/duration_sec`，不接受未知字段。1–256 个阶段。
- 阶段时长上限 3600 秒，下限为一次刷新间隔：10 Hz 下 0.1 秒，5 Hz 下 0.2 秒。
- `repeat_count`：1 表示完整序列一次；N 表示 N 次；0 无限循环。
  最大 10000。默认 1。
- `refresh_hz`：可选 5、10，默认 10。这是后台 SDK 调用的目标频率，
  不是模型调用频率；受 SDK 阻塞、网络与线程调度影响，不能保证硬实时。
- `end_behavior`：`release`（默认）完成后发一次黑色，释放控制，允许后续状态灯；
  `hold` 持续最后一个颜色；`off` 持续黑色。后两项即使周期状态为 completed，
  仍会刷新并占用 LED，直到 stop 或新 set/cycle/off/error。
- `stop/release` 停止刷新后，固件可能恢复默认灯效；需要持续熄灯用 `off`。

## Agent Core 调用示例

下面参数传给 `led`；拆分工具调用 `led.cycle` 时省略 action：

```json
{
  "action": "cycle",
  "sequence": [
    {"r": 0, "g": 255, "b": 0, "duration_sec": 10},
    {"r": 255, "g": 180, "b": 0, "duration_sec": 3},
    {"r": 255, "g": 0, "b": 0, "duration_sec": 10}
  ],
  "repeat_count": 3,
  "refresh_hz": 10,
  "end_behavior": "release"
}
```

此调用立即返回，驱动自主完成 69 秒颜色流程。不需要 Timer 卡片、Bash、
后台子代理、模型轮询或逐色调用。返回 cycle_status=running 表示周期任务已接收，不能据此
断言硬件已经点亮；可用 info 检查错误。当前不发布周期完成事件到 Core。

## 控制权及状态

只有一个后台输出线程，所有硬件调用及控制命令串行协调，避免旧阶段命令
在 stop 返回之后继续输出。正在执行的 SDK RPC 不能被 Python 强行取消，
暂停/停止可能等待该调用返回。错过阶段时按单调时钟跳到当前应显示阶段，
不会快速补发过期颜色。

手动单色、周期以及暂停保持期间，普通 hearing/thinking/speaking/idle hooks
返回 `ignored=true`，不能覆盖颜色。error hook 会终止当前周期并切为原错误灯，
其周期状态为 interrupted，不能 resume。停止服务后所有状态 hooks 均忽略。

info 返回：

- `state`：执行器生命周期 ready/idle，区别于具体周期状态。
  start 返回 ready；stop 返回 idle。全新 start 清除旧周期进度和错误，
  cycle_status 恢复 idle；已启动时重复 start 不重置当前周期。
- `mode`：state（语义灯）/cycle/paused/hold（持续色）。
- `cycle_status`：idle/running/paused/completed/stopped/interrupted/failed。
- `stage_index`、`cycle_index`：从 1 开始；无阶段时为 0。
- `elapsed_sec`、`remaining_sec`：周期时间；无限循环剩余时间为 null。
- `r/g/b`：当前逻辑颜色；不是独立硬件反馈。
- `refresh_hz`：手动输出目标频率；原语义灯仍使用约 30 ms 刷新。
- `last_error`：最近硬件失败。失败停止该灯效，避免自动高频重试；错误日志最多
  每 5 秒一次。新的 set/cycle/off 清除此错误并可重试。

## 本地验证与实机验收

```bash
python3 -m pytest -q tests/test_g1_led_cycle.py
```

无硬件测试覆盖阶段边界、有限/无限重复、JSON文本兼容、参数校验、暂停恢复、
普通hooks抑制/error抢占、SDK错误处理、5/10Hz实际线程输出、停止与在途RPC竞争、
完成释放后的语义灯及停止后重启。

部署后仍需实测：5/10Hz能否覆盖固件默认灯效、实际可见颜色、SDK延迟，以及
说话时是否保持周期色。先使用上述示例的 repeat_count=1；在一个阶段内测试
pause/resume，再单独测试 stop/off。不可将本地通过解释为已经验证实机灯效。
