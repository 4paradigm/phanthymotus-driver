# Q5 左手末端视觉重复定位测量

`visual_ee_repeatability` 是只读测量卡片。第一版测量手部 AprilTag 相对躯干 AprilTag 的重复定位离散程度；它不测 TCP 绝对精度，也不发送机械臂控制命令。

## 代码阅读顺序

1. **先看配置**：`apriltag_q5.yaml` 决定 Tag ID、坐标系和实际边长；`config.yaml` 的 `visual_ee_repeatability` 段决定采样阈值。特别注意 `tag_size_confirmed` 在实测前必须保持 `false`。
2. **再看数据从哪里来**：`q5_bundle_entrypoint.sh` 启动检测器；`visual_ee_repeatability.py` 的 `_on_detections` 读取双 Tag 可见性与质量，`_process_pending` 按图像时间戳查两个 TF，`_joint_state` 检查关节反馈。
3. **然后看一帧怎样变成一个样本**：`relative_pose` 计算手相对躯干的位姿；`_capture_sample` 等待停稳并收集不同帧；`aggregate_frames` 剔除视觉异常帧并合成为一次到位样本。
4. **最后看结果怎样形成**：`repeatability_report` 计算多次到位之间的 RMS、P95 等统计量；`_report` 加上可见率与拒收原因。`test_visual_ee_repeatability.py` 包含可在无机器人环境运行的关键测试。

阅读时请区分三层数据：**检测帧**、**单次到位样本**、**整次测量会话**。报告中的重复定位误差使用到位样本计算，不能把同一次到位的 25 帧当作 25 次重复运动。

## 真机准备

1. 将 ID 0 的 `tag36h11` 固定在能被头部相机持续看到的躯干刚性外壳上，将 ID 1 固定在左手背或腕部刚性外壳上。不要贴在手指、软胶或抓取面。
2. 用卡尺测量两个 Tag 的**黑色方框实际边长**，分别填入 `apriltag_q5.yaml` 的 `tag.sizes`。只有完成这一步后，才把 `config.yaml` 中的 `tag_size_confirmed` 设为 `true` 并重建镜像。
3. 在真机确认图像话题 `/camera/camera/color/image_raw`、内参 `/camera/camera/color/camera_info` 和 `/joint_states` 存在，图像与内参时间戳匹配。若有已校正的彩色图，优先将 `q5_bundle_entrypoint.sh` 中的 `image_rect` 改映射至该话题；不得把未经校正的图像当作校正图使用。
4. 确认 `/q5/apriltag/detections` 与 `/tf` 中两个 Tag 的坐标系持续出现。`info` 应显示检测器、内参、关节反馈在线且双 Tag 可见。
5. 让底盘、腰部、头部和左臂保持静止 30～60 秒，反复调用 `info`。`static_noise_baseline` 出现后，确认其位置噪声显著低于希望辨别的机械臂误差，再开始运动循环。

Q5 手册规定 ROS 2 Humble 使用 domain 211。真机调试时备好急停遥控器；运动姿态 A、B 须经机器人 Master 确认。测量过程中保持底盘、腰部、头部和灵巧手状态不变，并清空运动范围。

## 卡片操作

1. 调用 `start_session`，参数 `side=left`、`expected_samples=10`。
2. 人工使用已有的安全控制卡让左臂从离开姿态 B 返回测量姿态 A，待手臂停稳后调用 `capture_sample`。首次样本在 A 直接采集；之后每次都必须观察到关节离开 A 再返回。卡片要求七个左臂关节速度不超过配置阈值、状态时间新鲜、双 Tag 同帧且连续稳定至少 0.5 秒，然后收集 25 个不同图像时间戳的有效帧。
3. 完成 10 次后调用 `report`，读取毫米和角度的 RMS、P95、最大误差、各轴标准差及质量统计。`reset` 清空会话。

失败会返回原因码，例如 `INPUT_NOT_READY`、`DEPARTURE_NOT_OBSERVED`、`INSUFFICIENT_VALID_FRAMES`。不要把失败的一次采集计为到位样本。`report` 在样本不足 10 次时会将 `complete` 标为 `false`。

## 本地验证

在本目录运行：

```bash
python3 -m unittest -v test_visual_ee_repeatability.py
```

还需要在 Q5 真机验证检测器与 TF 的实时时间戳、彩色图校正情况、静止噪声基线，以及人工执行的 10 次运动循环。本地离线测试不能替代这些步骤。
