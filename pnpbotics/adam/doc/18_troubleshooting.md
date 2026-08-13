# 常见问题 (FAQ)

> Source: https://wiki.pndbotics.com/robot/trouble_shooting/

**Q：运行 `sh run.sh` 后显示 `No such file or directory`**

A：请检查xbox遥控器是否已经成功连接到NUC。如果控制器连接成功，可以在ubuntu系统中找到 `/dev/input/js*` 目录。然而，如果控制器没有连接成功，程序会一直检测是否存在 `/dev/input/js*` 目录，如果不存在，就会出现 `No such file or directory` 错误提示。所以请确认控制器是否连接成功，如果连接成功，会有 `xbox connected` 提示。

---

**Q：运行日志的存储路径在哪里？**

A：启动程序后，日志就会自动记录到到bin文件夹中。日志路径：`/home/pnd-humanoid/Documents/adam_demo/bin/*.txt`

---

**Q：站立模式下保持平衡的条件限制是什么？**

A：站立状态下质心位置不能超出脚板与地面的接触面。如果质心超出此范围，Adam不能继续保持平衡。

---

**Q：如果Adam自动连接失败，如何手动连接路由器和控制器？**

A：（参见原文）

---

**Q：SDK运行失败，返回错误**

A：错误原因是在使用Conda创建环境时，手动或自动调用 `source /opt/ros/humble/setup.bash` 污染了环境，Conda与Ros2不能同时使用。解决办法：重新创建没有调用该指令的环境。
