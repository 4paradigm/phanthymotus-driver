# Bumi RGB-D / IMU 里程计实验

独立实验，不接入运动卡片，不发送运动命令。生产驱动镜像、旋转和声音分支无需修改。

## 本地距离评估

```sh
python3 -m pytest -q tests/test_bumi_odometry_evaluate.py
python3 experiments/bumi_odometry/evaluate.py --manifest /path/to/trials.json --output /path/to/report.json
```

manifest 示例（相对路径以 manifest 所在目录为基准）：

```json
{"data_source":"measured","trials":[{"trial_id":"room-01","samples":"room-01.csv","truth":"room-01.json"}]}
```

合成数据必须标记 `data_source: synthetic`；未知来源或合成数据不会输出真机精度验证通过。

CSV 表头：`stamp_ns,x,y,z,qx,qy,qz,qw,status,segment_id`。位姿为同一水平里程计坐标系下的**机身参考点**，米、纳秒、xyzw 四元数。不要直接将相机位姿冒充机身位姿。

真值 JSON 示例：

```json
{"trial_id":"room-01","start_ns":[1000000000,1200000000],"end_ns":[11000000000,11200000000],"forward_m":1.003,"lateral_m":0.012,"uncertainty_m":0.005,"scene":"room","speed_m_s":0.1,"split":"acceptance"}
```

起终点窗口必须静稳（位置各轴跨度不超过 2 厘米），且处于同一 tracking 段。取位置均值，以起始窗口第一帧机身 X 轴的水平投影定义前向；报告前向、横向及水平向量误差。时间倒退、段重置、失跟、采样间隙超过 0.5 秒、无有效窗口均判为无效，不从报告中剔除。

验收需至少 20 次独立验收试次、至少两个场景，全部有效且 `abs(前向误差)+测量不确定度 ≤0.05m`。误差区间跨边界则为 inconclusive。还须人工核查真实测量、独立留出数据、不同朝向及拟使用速度覆盖；JSON 中的 measured 是来源声明，不是对实测真实性的自动认证。该报告不验证自动行走的制动精度。
