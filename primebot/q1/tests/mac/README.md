# Apple 芯片 Mac 有线联调

启动 Docker Desktop 和本机 Motus Core，把有线网络位置切到 Q1。联调用的 Mac 有线地址为 `10.1.1.99/24`，Q1 地址为 `10.1.1.100`，Core 容器名为 `fourparadigm-intern-core-1`。以下命令从驱动仓库根目录运行。

```bash
python3 primebot/q1/deploy/q1.py build
python3 primebot/q1/deploy/q1.py start
python3 primebot/q1/deploy/q1.py status
```

`build` 只在首次使用或更新镜像时需要。`start` 生成有线 DDS 配置并启动两个 ARM64 容器和一个 macOS DDS 转发进程。宿主机进程把来自 Q1 的发现公告与 RTPS 数据报送入 Docker。driver 共享 Core 容器的网络与 IPC，Core 在域 42 通过回环访问 MCP 与卡片数据。脚本从 Core 容器读取 DDS 配置文件的挂载路径和 Docker 网络。

`status` 显示 `Core registration: registered` 和 `battery.state: live` 后，在 [Motus 画布](http://127.0.0.1:15678) 添加「启元 Q1 探索版 → battery」，点「查看数据流」。结束联调时运行：

```bash
python3 primebot/q1/deploy/q1.py stop
```

该命令停止本机转发与 Q1 容器。下次接上网线、切换到 Q1 的有线网络位置后，执行 `start` 即可。若地址或配置变化，先停止 Q1，再传入新配置启动，例如：

```bash
python3 primebot/q1/deploy/q1.py start --mac-ip 10.1.1.99 --robot-ip 10.1.1.100 \
  --core-container fourparadigm-intern-core-1 --core-url http://127.0.0.1:15678
```

Core 接入多个 Docker 网络时，通过 `--docker-network` 指定接收 DDS 转发的网络。若 Core 容器重建，待 Core 就绪后重新执行相同的 Q1 启动命令。运行日志与验收边界见 [Q1 电池卡验证记录](../../验证记录.md)。
