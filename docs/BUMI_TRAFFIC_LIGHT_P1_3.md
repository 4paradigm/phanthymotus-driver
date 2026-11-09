# P1-3：歧义检测框不能继承旧交通灯历史

分支：`feat/bumi-driver`；HEAD：`814b3ca953d33eba0aca9a61dcefa773f5da483b`。基于492项主机回归，只修复Driver共享包的tracking association/history ownership。主仓库只读，保留所有已有工作区修改；没有部署、启动服务或机器人操作。

## 原关联算法及复现

`common/traffic_light/processor.py::Track`（52行）保存signal_id、bbox、last_seen和最多5份history；observations是history长度，不是终身观测总数。`schemas.py::Config`（35行）默认min_detection_confidence=0.3、track_ttl_s=1.0、max_tracks=32。`_prune`（88行）清除到期轨迹；未匹配目标的history在帧末清空。

原`_process`按输入顺序逐框处理，只在尚未分配的available tracks中按IoU排序。最佳IoU需至少0.3，且与次佳差至少0.15（只有一个候选也可匹配）。随后立即追加当前分类证据、更新bbox/last_seen并投票。整个步骤没有计算同一旧track在整帧有多少current claimants。

已在内存中加载修改前processor与新processor对照，不写回旧代码：

1. A=`[0,0,100,100]`、B=`[60,0,160,100]`，均积累3次green/circle，stable。
2. 新帧两个bbox均为`[10,0,110,100]`，置信度分别0.9、0.31。
3. 第一框对A/B的IoU为约0.818/0.333，差约0.485，先选A；第二框看不到已占用的A，只剩B可选，于是继承B。
4. 旧实现输出signal_001/002、observations=4、green/circle/stable。新实现输出signal_003/004、observations=1、unknown/unknown/unstable。

0.31本来就是有效检测，不能通过提高confidence阈值掩盖关联错误。

## 新规则和歧义处理

`TrafficLightProcessor._associate`（174行）在任何轨迹分配、bbox更新和history写入之前，基于整帧有效检测与所有尚未过期旧轨迹建立候选图：

- 维持原有效bbox/confidence检查，IoU阈值仍是0.3，不新增配置。
- 只有detection恰有一个旧track候选，且该旧track恰有这一个detection候选时，才继承ID/history。这是候选图的孤立一对一边。
- 多对一、一对多、多对多，以及通过共享候选相连的歧义组件，其全部旧track端点从self.tracks退休。无需另建复杂图算法：互为唯一的边必然是独立组件；其他非空候选边的旧端点全部属于歧义组件。
- 当前歧义框不删除、不按最高confidence合并。每个有效框在原容量限制内使用单调递增的新ID、新deque，仅写当前一次证据。原3/5投票会返回unknown/unstable，observations=1；原容量不足仍走track_capacity/unknown。
- 没有候选的有效框仍按原路径新建。低于confidence阈值的框仍为无ID、observations=0、unknown，不参与候选关系或写入证据。
- 不相关的一对一组件继续正常跟踪；失踪清空历史、TTL回收、历史上限5、color/symbol分别投票均保持。

选择退休旧track，而非仅让当前输出临时unknown，是为了保证下一帧无法重新连回歧义前的stable历史。歧义后的新轨迹至少需要3份新的有效同标签证据才能stable；如果下一帧仍有候选歧义，会再次退休，不能靠重复歧义框积累稳定状态。

关联复杂度O(N×M)，没有Kalman、光流、ReID、Hungarian或额外依赖。Runtime已有RLock在image/detection/tick/close及info/latest路径串行访问processor（`plugin.py:63/74/86/93/232/244`）；本轮沿用，没有新增锁或改回调模型。

## 身份边界

signal_id仍表示当前观察流内的局部空间轨迹，不是经ReID验证的物理信号灯身份。可见交叉/合并/分裂形成歧义时会重建ID、重新确认。两个目标若在相邻采样间瞬时完整交换位置、却没有任何可见几何歧义，IoU无法检测这种事件；不能声称已解决该物理身份问题。测试另验证颜色和箭头同时交换时，现有current-label投票返回unknown/unstable，不沿用旧输出；相同外观的不可观测交换不在V1能力内。

## 修改文件与不变内容

- `common/traffic_light/processor.py`：仅增加_associate，替换_process中的贪心分配区；证据分类、投票和缺失清理路径不变。
- `tests/test_traffic_light_tracking.py`：新增18项确定性测试，用真实JPEG、OpenCV颜色/形状路径，不替换分类器。
- `docs/BUMI_SMART_TRAFFIC_POLICE_V1.md`：补充新规则、完整测试命令及结果。
- 本文：复现、边界、验收记录。

没有修改schemas、HSV/颜色/箭头/circle/convexity/template/bulbs算法、source_stamp纳秒join、帧缓存/pending/stale/waiting/JPEG保护、ROS Topic/type/QoS、Card生命周期、Bumi注册/配置/打包、gesture/Skill、observation/loco/servo、SDK/HighController、Agent Core、ACP、global stop_all。

## 关键测试

`tests/test_traffic_light_tracking.py`直接检查signal_id、observations、color、symbol、稳定性及内部history归属，不以“无异常”替代结果断言。

| 场景 | 测试函数 |
| --- | --- |
| 正常两灯、三灯，检测顺序变化，独立颜色/箭头/ID/history，上限5 | test_clear_multiple_targets_keep_independent_ids_histories_colors_and_symbols |
| 完全重复框、confidence 0.31 | test_identical_duplicate_boxes_cannot_consume_two_stable_tracks |
| 高重叠多对多、输入顺序变化 | test_high_overlap_many_to_many_retires_both_old_histories |
| 旧2→新1，即使最佳IoU显著更大也不能继承 | test_one_detection_with_two_old_candidates_cannot_choose_the_largest_iou |
| 旧1→新2，不能拆分旧历史 | test_two_detections_claiming_one_track_both_start_without_its_history |
| 歧义后恢复，必须重新有3次证据 | test_after_ambiguity_clear_frames_need_three_new_observations |
| 连续重复框永远不能借旧历史stable | test_repeated_duplicates_never_accumulate_stable_evidence |
| confidence 0.29仍unknown/0，不干扰有效单独匹配 | test_weak_duplicate_point29_is_unknown_and_does_not_steal_valid_history |
| 全部弱检测不计数、清空未匹配旧历史 | test_only_weak_frame_clears_history_without_counting_invalid_evidence |
| 连通歧义组件全部退休，独立第三灯继续stable | test_ambiguity_retires_its_whole_connected_component_but_not_a_separate_target |
| 可见交叉后分离，旧历史不能复活 | test_visible_crossing_resets_before_targets_separate_in_swapped_order |
| 无几何歧义的瞬时交换：不伪称物理ID，改变的颜色/箭头不沿用旧输出 | test_instant_position_swap_with_changed_evidence_does_not_claim_stable_labels |
| bbox显著缩小、失踪/重现及TTL | test_large_scale_change_starts_new_history_and_does_not_widen_iou_gate、test_disappearance_and_ttl_expiry_keep_existing_reconfirmation_behavior |
| 在既有max_tracks上限下先退休再分配 | test_ambiguity_retires_before_allocating_under_the_existing_track_capacity |

按要求先运行`tests/test_traffic_light.py tests/test_traffic_light_tracking.py`：79 passed（0.65s）；再运行packaging/bundle测试：22 passed（0.88s）；最后运行完整Bumi智能交警主机回归，命令见[总说明](BUMI_SMART_TRAFFIC_POLICE_V1.md#主机测试与未覆盖项)。使用既有临时Python环境，禁用pytest自动插件、缓存和字节码，未安装依赖。

最终统一结果：**510 collected、510 passed、0 failed、0 skipped，58.85秒**，包含原492项及新增18项。没有删除或修改任何原测试。

范围复核：除本文列出的4个文件外，所有Driver文件的内容聚合SHA256与开始一致，索引哈希未变；processor只有_process关联区改变并增加_associate，source/join/JPEG保护前半段、分类/投票/bulbs/未匹配清理后半段及其余方法均保持。只读主仓库623个文件的聚合SHA256仍为`c11944f4e940604a701e5c652e5cdf9c67f1e140df6e81594f56b1f5e982d645`，Git status及索引哈希亦一致。

已执行git status --short、git diff --stat和git diff --check；check通过。tracked diff仍为开始时的8 files / +184 / -29；本轮涉及的processor及文档原本未跟踪，新增测试/说明也未跟踪，因此不计入该stat。没有git add、commit、push、build、deploy、SSH或真机动作。本轮P1-3修复通过，其他P1/P2未处理。
