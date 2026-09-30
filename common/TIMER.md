# 通用 Timer 卡片

`common.timer.TimerPlugin` 是不包含机器人业务的 `processor` 卡片。它使用单调时钟维护
多个正向或倒计时器，调用 `create` 后立即返回，之后通过 `data/json` 输出报警、周期进度和
完成事件。

G1 默认将事件发布到 `/<机器人 namespace>/timer/events`。在画布上把 Timer 的 JSON
输出连接到 `decision_core`，Agent 才会收到主动提醒。卡片本身不会调用 TTS、LED 或动作。

## 倒计时

```json
{
  "action": "create",
  "timer_id": "green-light",
  "mode": "countdown",
  "duration_sec": 30,
  "alarms": [
    {
      "alarm_id": "ten-left",
      "trigger_type": "remaining",
      "trigger_sec": 10,
      "event": "green-ending"
    },
    {
      "alarm_id": "zero",
      "trigger_type": "remaining",
      "trigger_sec": 0,
      "event": "green-completed"
    }
  ],
  "payload": {"scene": "traffic-light", "cycle": 1}
}
```

## 正向计时

```json
{
  "action": "create",
  "timer_id": "exercise",
  "mode": "countup",
  "duration_sec": 120,
  "alarms": [
    {
      "alarm_id": "two-minutes",
      "trigger_type": "elapsed",
      "trigger_sec": 120,
      "event": "exercise-two-minutes"
    }
  ]
}
```

## 控制操作

```json
{"action":"start"}
{"action":"stop"}
{"action":"create", "timer_id":"green-light", "mode":"countdown", "duration_sec":30}
{"action":"pause", "timer_id":"green-light"}
{"action":"resume", "timer_id":"green-light"}
{"action":"cancel", "timer_id":"green-light"}
{"action":"reset", "timer_id":"green-light"}
{"action":"info", "timer_id":"green-light"}
{"action":"list"}
```

`emit_interval_sec` 默认为 `0`，此时只发送报警和完成事件。设置为正数后会输出
`timer_tick`。进程发生延迟时，过期的 tick 会合并为一条，避免一次向 Agent Core 补发
大量陈旧事件。

`start` 和 `stop` 只管理卡片生命周期；创建具体计时器必须使用 `create`。

每次 `create` 或 `reset` 都会生成新的 `run_id`。业务流程应同时核对 `timer_id` 和
`run_id`，忽略已被替换的旧运行事件。
