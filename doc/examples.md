### 常用命令与操作流程

本指南介绍了 ZIT6 AUV 的基本操作流程，包括启动、校准、参数设置及控制。

#### 0. 基础环境准备 (bashrc 别名)
建议在 `~/.bashrc` 中添加以下别名以简化操作：
```bash
# AUV ROS2 工作空间与工具启动
alias zit_src='source /home/doc049/dev/2026-auv-sub/AUV_zit6_cmake/install/setup.bash'
alias zit_agt='zit_src && MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'
alias zit_fox='zit_src && ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
alias zit_rqt='zit_src && rqt'
alias zit_cfg='zit_src && ros2 run upper_examples config_setter'
alias zit_gui='zit_src && ros2 run upper_examples gui'
```

---

#### 1. 启动 INS 并等待校准
首先启动 Agent 以建立与嵌入式的通信，然后监控导航状态。
- **启动通信**: `zit_agt`
- **监控状态**:
  ```bash
  ros2 topic echo /zit6/state/status
  ```
- **说明**: 
  - 观察 `ins_state` 字段：`0:待机, 1:粗对准, 2:精对准, 3:SINS/GPS/DVL (正常)`。
  - 等待 `navigation_ready: true`。只有当导航准备就绪时，系统才允许进入位置控制模式。

#### 2. 设置 PID 参数
使用 `update_params` 服务在线更新 PID 参数。支持 JSON 格式的全量或增量更新。
- **命令示例**:
  ```bash
  ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams "{json: '{\"chassis\":{\"pid\":{\"pos\":{\"kp\":0.015,\"ki\":0.001}}}}'}"
  ```
- **例程用法**: 也可以使用预定义的 python 脚本（该脚本默认读取 `setcfg.json` 中的配置）:
  ```bash
  ros2 run upper_examples pid_setter
  ```

#### 3. 启动 agxhbt (心跳与解锁)
嵌入式系统需要接收来自上位机的心跳包才能解锁（Arm）。
- **正常解锁 (需导航就绪)**:
  ```bash
  ros2 topic pub -r 10 /zit6/cmd/agxhbt std_msgs/msg/UInt32 "{data: 1}"
  ```
- **强制解锁 (遥控模式，绕过导航检查)**:
  ```bash
  ros2 topic pub -r 10 /zit6/cmd/agxhbt std_msgs/msg/UInt32 "{data: 3}"
  ```
- **说明**: 持续发送该话题，嵌入式收到连续心跳后 `is_armed` 将变为 `true`。

#### 4. 下发控制目标 (Setpoint)
系统解锁后，即可发送控制指令。
- **位置控制 (POS)**: 移动到世界坐标 (x=1.0, y=0, z=-1.0)
  ```bash
  ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint "{control_key: 0, type_mask: 7, x: 1.0, y: 0.0, z: -1.0, yaw: 0.0}"
  ```
- **速度控制 (VEL)**: 以 0.2m/s 向前移动
  ```bash
  ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint "{control_key: 1, type_mask: 1, x: 0.2, y: 0.0, z: 0.0, yaw: 0.0}"
  ```
- **机体系增量控制**: 向前移动 0.5m
  ```bash
  ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint "{control_key: 48, type_mask: 1, x: 0.5, y: 0.0, z: 0.0, yaw: 0.0}"
  # control_key=48 (0x30): Bit4(Body)+Bit5(Inc)
  ```

---

#### 常见调试命令
- **查看位置反馈**: `ros2 topic echo /zit6/state/pos`
- **查看推力输出**: `ros2 topic echo /zit6/state/thr`
- **查看实时日志**: `ros2 topic echo /zit6/log`
- **检查所有参数**: `ros2 service call /zit6/get_params zit6_interfaces/srv/GetParams "{paths: []}"`
- **打开综合主控制台（含日志监控面板）**: `ros2 run upper_examples gui`
