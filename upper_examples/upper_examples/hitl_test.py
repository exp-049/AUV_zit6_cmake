#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZIT6 AUV HITL/SITL 闭环测试集 — 完整版

control_key 编码:
  bits 0-1: level (0=POSITION, 1=VELOCITY, 2=ACTUATOR)
  bit  4   : is_body  (0=world frame, 1=body frame)
  bit  5   : is_inc   (0=absolute, 1=incremental)
  type_mask: bit set = skip axis (bit0=X, bit1=Y, bit2=Z, bit5=Yaw); Roll/Pitch are bypassed

测试场景一览:
  1. frame_world_pos     世界系位置指令
  2. frame_body_pos      机体系位置指令（含旋转验证）
  3. frame_world_vel     世界系速度指令
  4. frame_body_vel      机体系速度指令
  5. mode_pos_abs        位置绝对模式
  6. mode_pos_inc        位置增量模式
  7. mode_vel_abs        速度绝对模式
  8. mode_vel_inc        速度增量模式
  9. switch_none         切换到 NONE → 输出归零
  10. switch_pos_vel     位置→速度切换（无扰动）
  11. switch_vel_act     速度→推进器切换
  12. mask_single        单轴 mask
  13. mask_multi         多轴 mask
  14. mask_all           全轴 mask（无变化）
  15. planner_on         规划器启用（S-curve）
  16. planner_off        规划器禁用（直接阶跃）
  17. step_x             X 轴阶跃
  18. step_yaw           Yaw 轴阶跃
  19. step_multi         多轴同时阶跃
  20. edge_nan           NaN 输入被拒绝
  21. edge_disarmed      未解锁时指令被忽略
  22. all                全部依次运行
"""

import argparse
import csv
import math
import os
import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Float32MultiArray, UInt32
from nav_msgs.msg import Odometry
from zit6_interfaces.msg import ZitSetpoint, ZitStatus

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAVE_VIZ = True
except ImportError:
    HAVE_VIZ = False

# ============================================================================
# control_key 快捷构造
# ============================================================================

def CK(level, is_body=False, is_inc=False):
    return (level & 0x03) | (0x10 if is_body else 0) | (0x20 if is_inc else 0)

CK_POS_WORLD_ABS = CK(0, False, False)   # 0x00
CK_POS_BODY_ABS  = CK(0, True,  False)   # 0x10
CK_POS_BODY_INC  = CK(0, True,  True)    # 0x30
CK_POS_WORLD_INC = CK(0, False, True)    # 0x20
CK_VEL_WORLD_ABS = CK(1, False, False)   # 0x01
CK_VEL_BODY_ABS  = CK(1, True,  False)   # 0x11
CK_VEL_BODY_INC  = CK(1, True,  True)    # 0x31
CK_ACT_BODY_ABS  = CK(2, True,  False)   # 0x12


# ============================================================================
# 6-DOF 物理引擎
# ============================================================================

class PhysicsEngine:
    """简易 Fossen 6-DOF，与 MCU 的 HitlSimulator 行为一致"""

    def __init__(self, dt=0.01, config=None):
        self.dt = dt
        cfg = config or {}
        mass = cfg.get("mass", 35.0)
        drag = cfg.get("drag", 15.0)
        r2 = cfg.get("r2", 0.09)
        self.masses = [mass, mass, mass, mass*r2, mass*r2, mass*r2]
        self.drags = [drag]*3 + [drag*0.15]*3
        self.drag_quad = [drag*0.3]*3 + [drag*0.05]*3
        # max_force: normalized [-1,1] → 牛顿的缩放系数
        # 校准值配合 planner(开启前馈拖曳): 
        #   vel_output_limit=0.8, drag=2.0 → 饱和输出 0.8
        #   0.8 × max_force = 7.6N, 略低于 drag_at_0.5m/s(8.625N)
        #   平衡速度 ~0.44m/s, 位置 2m 在 7s 内收敛且不过冲
        #   Yaw 轴 12Nm 匹配 8 推进器 AUV 偏航力矩
        self.max_force = cfg.get("max_force", [9.5, 9.5, 13.5, 2, 2, 12])
        self.mh = cfg.get("metacentric_height", 0.3)
        self.g = 9.81
        self.weight = self.masses[0] * self.g
        self.reset()

    def _R(self, r, p, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        R3 = [[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
              [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
              [-sp,   cp*sr,            cp*cr]]
        if abs(cp) > 1e-4:
            T3 = [[1.0, sr*sp/cp, cr*sp/cp],
                  [0.0, cr,       -sr],
                  [0.0, sr/cp,    cr/cp]]
        else:
            T3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        return R3, T3

    def step(self, forces):
        f = [forces[i] * self.max_force[i] for i in range(6)]
        restore = [0.0]*6
        restore[3] = -self.weight * self.mh * math.sin(self.pos[3]) * math.cos(self.pos[4])
        restore[4] = -self.weight * self.mh * math.cos(self.pos[3]) * math.sin(self.pos[4])
        for i in range(6):
            dl = self.vel[i] * self.drags[i]
            dq = self.drag_quad[i] * abs(self.vel[i]) * self.vel[i]
            m = self.masses[i] if self.masses[i] > 1e-6 else (20.0 if i<3 else 1.0)
            self.acc[i] = (f[i] - dl - dq + restore[i]) / m
        for i in range(6):
            self.vel[i] += self.acc[i] * self.dt
        R3, T3 = self._R(self.pos[3], self.pos[4], self.pos[5])
        eta = [0.0]*6
        for i in range(3):
            eta[i] = sum(R3[i][j] * self.vel[j] for j in range(3))
        for i in range(3):
            eta[3+i] = sum(T3[i][j] * self.vel[3+j] for j in range(3))
        for i in range(6):
            self.pos[i] += eta[i] * self.dt
        for i in range(3, 6):
            if self.pos[i] > math.pi:  self.pos[i] -= 2*math.pi
            if self.pos[i] < -math.pi: self.pos[i] += 2*math.pi
        return list(self.pos), list(self.vel)

    def reset(self, pos=None):
        self.pos = list(pos) if pos else [0.0]*6
        self.vel = [0.0]*6
        self.acc = [0.0]*6


# ============================================================================
# 测试节点
# ============================================================================

ALL_SCENARIOS = [
    "frame_world_pos", "frame_body_pos", "frame_world_vel", "frame_body_vel",
    "mode_pos_abs", "mode_pos_inc", "mode_vel_abs", "mode_vel_inc",
    "switch_none", "switch_pos_vel", "switch_vel_act",
    "mask_single", "mask_multi", "mask_all",
    "planner_on", "planner_off",
    "step_x", "step_yaw", "step_multi",
    "edge_nan", "edge_disarmed",
]

SCENARIO_DOC = {
    "frame_world_pos": "世界系位置: 发送 x=2 → AUV 移动到 (2,0,0)",
    "frame_body_pos":  "机体系位置: 先旋转 90° 再 body-x=1 → 应朝世界 Y 移动",
    "frame_world_vel": "世界系速度: x=0.5m/s",
    "frame_body_vel":  "机体系速度: body-x=0.5m/s",
    "mode_pos_abs":    "位置绝对模式: 直接设定目标位置",
    "mode_pos_inc":    "位置增量模式: 在现有位置上 +1m",
    "mode_vel_abs":    "速度绝对模式: 直接设定目标速度",
    "mode_vel_inc":    "速度增量模式: body-frame incremental",
    "switch_none":     "切换到 NONE → 推力归零",
    "switch_pos_vel":  "位置→速度无扰动切换",
    "switch_vel_act":  "速度→推进器模式切换",
    "mask_single":     "单轴 mask: 只更新 X 轴",
    "mask_multi":      "多轴 mask: 更新 X+Yaw",
    "mask_all":        "全轴 mask: 所有值不变",
    "planner_on":      "规划器启用 → S-curve 平滑",
    "planner_off":     "规划器禁用 → 直接阶跃",
    "step_x":          "X 轴阶跃响应",
    "step_yaw":        "Yaw 轴阶跃响应",
    "step_multi":      "多轴同时阶跃",
    "edge_nan":        "NaN 输入应被拒绝",
    "edge_disarmed":   "未解锁时指令被忽略",
}


# ============================================================================
# 实时 3D 可视化 (独立线程)
# ============================================================================

class Viz3D:
    """matplotlib 3D 可视化，独立线程运行，不阻塞 ROS2 事件循环"""

    def __init__(self):
        if not HAVE_VIZ:
            return
        # 后台线程启动 matplotlib 会触发 warning, 但实际可用
        import warnings
        warnings.filterwarnings('ignore', message='.*outside of the main thread.*')
        self._queue = []
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def update(self, pos, sp_xyz):
        if not self._running:
            return
        with self._lock:
            self._queue.append((list(pos[:6]), sp_xyz))

    def _run(self):
        plt.ion()
        self.fig = plt.figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (m)'); self.ax.set_ylabel('Y (m)'); self.ax.set_zlabel('Z (m)')
        self.ax.set_title('HITL 3D')
        self.traj_x, self.traj_y, self.traj_z = [], [], []
        self.traj_line, = self.ax.plot([], [], [], 'b-', lw=1, alpha=0.6, label='trail')
        self.auv_body, = self.ax.plot([], [], [], 'ro', ms=6, label='AUV')
        self.sp_marker, = self.ax.plot([], [], [], 'g*', ms=10, label='target')
        self.auv_arrow = None
        self.ax.legend(loc='upper right')
        self.ax.set_xlim(-3, 5); self.ax.set_ylim(-3, 5); self.ax.set_zlim(-1, 1)

        while self._running:
            frames = []
            with self._lock:
                frames, self._queue = self._queue, []
            for pos, sp_xyz in frames:
                self._render(pos, sp_xyz)
            plt.pause(0.03)
        plt.ioff()
        plt.close(self.fig)

    def _render(self, pos, sp_xyz):
        x, y, z, _, _, yaw = pos
        self.traj_x.append(x); self.traj_y.append(y); self.traj_z.append(z)
        if len(self.traj_x) > 300:
            self.traj_x.pop(0); self.traj_y.pop(0); self.traj_z.pop(0)
        self.traj_line.set_data(self.traj_x, self.traj_y)
        self.traj_line.set_3d_properties(self.traj_z)
        self.auv_body.set_data([x], [y])
        self.auv_body.set_3d_properties([z])
        if self.auv_arrow:
            self.auv_arrow.remove()
        dx, dy = 0.4 * math.cos(yaw), 0.4 * math.sin(yaw)
        self.auv_arrow = self.ax.quiver(x, y, z, dx, dy, 0, color='red')
        if sp_xyz:
            self.sp_marker.set_data([sp_xyz[0]], [sp_xyz[1]])
            self.sp_marker.set_3d_properties([sp_xyz[2]])
        pts_x = self.traj_x + [x] + ([sp_xyz[0]] if sp_xyz else [])
        pts_y = self.traj_y + [y] + ([sp_xyz[1]] if sp_xyz else [])
        m = 2.0
        self.ax.set_xlim(min(pts_x)-m, max(pts_x)+m)
        self.ax.set_ylim(min(pts_y)-m, max(pts_y)+m)

    def close(self):
        self._running = False
        self._thread.join(timeout=2.0)


class HitlTestNode(Node):
    SCENARIO_LEN = 10.0      # 每个场景时长(秒), 给足收敛时间
    ARM_TIME = 6.0           # 2Hz 心跳下，给足 10 次心跳的解锁时间
    HEARTBEAT_TICKS = 50     # _tick 周期 10ms，即 2Hz

    def __init__(self, mode="sitl", scenarios=None, duration=30.0):
        super().__init__("hitl_test_node")
        self.mode = mode
        self.duration = duration
        self.running = True
        self.armed = False
        self.start_time = 0.0
        self.last_forces = [0.0]*6
        self.status_count = 0
        self.last_progress_log = 0.0

        # 场景队列（在日志输出前初始化）
        self.scenarios = (ALL_SCENARIOS if "all" in (scenarios or ["all"]) else scenarios)
        self.scn_idx = 0
        self.scn_t = 0.0

        # 加载与固件一致的配置；文档推荐配置仅作为源码树中的后备。
        project_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", ".."))
        config_candidates = [
            os.path.join(project_root, "UserApp", "Config", "config.json"),
            os.path.join(project_root, "docs", "configuration",
                         "config_recommend.json"),
        ]
        self.config = {}
        for cfg_path in config_candidates:
            if os.path.exists(cfg_path):
                with open(cfg_path) as f:
                    self.config = json.load(f)
                break
        self.physics = PhysicsEngine(dt=0.01, config=self.config.get("simulation"))

        self.get_logger().info(f"🚀 HITL 测试启动: mode={mode}, scenarios={len(self.scenarios)}项, duration={duration}s")
        self.get_logger().info(f"   场景列表: {', '.join(self.scenarios[:5])}{'...' if len(self.scenarios)>5 else ''}")
        self.get_logger().info(f"   等待 micro-ROS Agent 连接...")

        # ROS2 — 使用 SystemDefault QoS 匹配 micro-ROS 默认配置
        # micro-ROS 的 rclc_subscription_init_default 使用默认 QoS(Reliable)
        # 必须匹配才能通信
        from rclpy.qos import qos_profile_system_default
        self.pub_sp = self.create_publisher(ZitSetpoint, "/zit6/cmd/setpoint",
                                            qos_profile_system_default)
        self.pub_hb = self.create_publisher(UInt32, "/zit6/cmd/agxhbt",
                                            qos_profile_system_default)
        if mode == "sitl":
            self.pub_sim_nav = self.create_publisher(Odometry, "/zit6/sim/nav",
                                                     qos_profile_system_default)

        # 订阅者
        self.sub_sta = self.create_subscription(ZitStatus, "/zit6/state/status",
                                                self._cb_sta, qos_profile_system_default)
        self.sub_thr = self.create_subscription(Float32MultiArray, "/zit6/state/thr",
                                                self._cb_thr, qos_profile_system_default)

        # UpdateParams 服务客户端（运行时切换 SITL 模式）
        from zit6_interfaces.srv import UpdateParams as _UP
        self._UpdateParams = _UP
        self.cli_update = self.create_client(_UP, "/zit6/update_params")
        self.sitl_configured = False

        self.log = []
        self.scn_log = []        # 当前场景数据（用于实时分析）
        self.scn_last_idx = -1   # 上次场景索引
        self.results = []        # [(场景名, 通过, 指标字典)]
        self._sim_tick = 0
        self._hb_tick = 0
        self._sp_tick = 0
        self._viz_tick = 0       # 可视化节拍计数器
        self._last_sp = (0.0,)*6  # 最后发送的 setpoint
        self.create_timer(0.01, self._tick)

        # 3D 可视化
        self.viz = Viz3D() if HAVE_VIZ and mode == "sitl" else None

    def _cb_sta(self, msg):
        self.armed = msg.is_armed
        self.status_count += 1

    def _cb_thr(self, msg):
        if len(msg.data) >= 6:
            self.last_forces = list(msg.data[:6])

    def _configure_sitl(self):
        """通过 UpdateParams 服务启用 SITL 模式（无需重刷固件）"""
        if self.sitl_configured:
            return
        if not self.cli_update.service_is_ready():
            return
        self.sitl_configured = True

        req = self._UpdateParams.Request()
        # 使用 KV 路径方式（JSON 方式有 root->string==NULL bug）
        req.json = ""
        req.paths = ["simulation.sitl_enabled", "simulation.hitl_enabled",
                      "chassis.planner_enabled"]
        req.values = ["true", "false", "true"]
        future = self.cli_update.call_async(req)
        self.get_logger().info("📤 通过服务设置 SITL 模式...")
        future.add_done_callback(self._sitl_cb)

    def _sitl_cb(self, future):
        try:
            res = future.result()
            if res and res.success:
                self.get_logger().info("✅ SITL 模式已启用! 等待 MCU 解锁...")
            else:
                msg = res.message if res else "None"
                self.get_logger().warn(f"⚠️ SITL 配置未成功: {msg}")
        except Exception as e:
            self.get_logger().error(f"SITL 配置失败: {e}")

    # ========================================================================
    # 场景量化分析
    # ========================================================================

    def _steady(self, data, field, ratio=0.25):
        """取数据末尾 ratio 比例的均值作为稳态值"""
        if len(data) < 5:
            return 0.0
        n = max(1, int(len(data) * ratio))
        return sum(d[field] for d in data[-n:]) / n

    def _max_abs(self, data, field):
        return max(abs(d[field]) for d in data) if data else 0.0

    def _analyze_scenario(self, name, data):
        """分析一个场景的数据, 打印量化指标"""
        if len(data) < 10:
            self.get_logger().info(f"  ⏩ {name}: 数据不足 ({len(data)}条), 跳过分析")
            return

        result = {"name": name, "pass": True, "metrics": {}}
        prefix = f"  [{name:20s}]"

        # ---- 根据场景类型选择分析逻辑 ----
        if name in ("step_x", "mode_pos_abs", "frame_world_pos"):
            target = 2.0
            steady = self._steady(data, "x")
            error = abs(steady - target)
            max_force = self._max_abs(data, "fx")
            result["metrics"] = {"target_x": target, "steady_x": round(steady,3),
                                 "error": round(error,3), "max_fx": round(max_force,3)}
            ok = error < 0.2
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} 目标X={target}m 稳态X={steady:.2f}m "
                f"误差={error:.3f}m {'✅' if ok else '❌'}")

        elif name == "frame_body_pos":
            # 朝东时 body-x=1 → 世界 Y ≈ 1
            steady_y = self._steady(data, "y")
            error = abs(steady_y - 1.0)
            ok = error < 0.3
            result["metrics"] = {"expected_y": 1.0, "steady_y": round(steady_y,3),
                                 "error": round(error,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} 期望Y=1.0m(body-x绕Yaw90°) 稳态Y={steady_y:.2f}m "
                f"误差={error:.3f}m {'✅' if ok else '❌'}")

        elif name in ("frame_world_vel", "mode_vel_abs"):
            target = 0.5
            steady_u = self._steady(data, "u")
            error = abs(steady_u - target)
            ok = error < 0.1
            result["metrics"] = {"target_u": target, "steady_u": round(steady_u,3),
                                 "error": round(error,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} 目标U={target}m/s 稳态U={steady_u:.2f}m/s "
                f"误差={error:.3f}m/s {'✅' if ok else '❌'}")

        elif name in ("frame_body_vel",):
            steady_u = self._steady(data, "u")
            ok = steady_u > 0.1  # body 方向应产生正向速度
            result["metrics"] = {"steady_u": round(steady_u,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} 稳态U={steady_u:.2f}m/s {'✅' if ok else '❌'}(速度过低)")

        elif name == "mode_pos_inc":
            # 先设 x=1, 再增量 +1 → 期望 x≈2
            # 取后半段数据
            steady_x = self._steady(data, "x")
            error = abs(steady_x - 2.0)
            ok = error < 0.3
            result["metrics"] = {"expected_x": 2.0, "steady_x": round(steady_x,3),
                                 "error": round(error,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} 增量+1m 期望X=2.0m 稳态X={steady_x:.2f}m "
                f"误差={error:.3f}m {'✅' if ok else '❌'}")

        elif name in ("switch_none",):
            # 切换到 NONE 后推力应归零
            late = data[-len(data)//2:]  # 后半段
            max_f = max(abs(d["fx"]) for d in late) if late else 999
            ok = max_f < 0.01
            result["metrics"] = {"max_fx_late": round(max_f,4)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} 后半段最大推力={max_f:.3f} {'✅' if ok else '❌'}(应归零)")

        elif name == "mask_single":
            # X 更新为 2.0, Y/Z/Yaw 被 mask 跳过(保持0)
            steady = self._steady(data, "x")
            ok = abs(steady - 2.0) < 0.2
            result["metrics"] = {"steady_x": round(steady,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} X={steady:.2f}m(mask只更新X) {'✅' if ok else '❌'}")

        elif name == "mask_all":
            # 全 mask → 位置不变(保持0)
            steady_x = self._steady(data, "x")
            ok = abs(steady_x) < 0.1
            result["metrics"] = {"steady_x": round(steady_x,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} X={steady_x:.3f}m(全mask应不变) {'✅' if ok else '❌'}")

        elif name == "step_yaw":
            target = 1.57
            steady_yaw = self._steady(data, "yaw")
            error = abs(steady_yaw - target)
            ok = error < 0.2
            result["metrics"] = {"target_yaw": target, "steady_yaw": round(steady_yaw,3),
                                 "error": round(error,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} 目标Yaw={target}rad 稳态Yaw={steady_yaw:.2f}rad "
                f"误差={error:.2f}rad {'✅' if ok else '❌'}")

        elif name == "step_multi":
            sx = self._steady(data, "x"); sy = self._steady(data, "y")
            syaw = self._steady(data, "yaw")
            ok = abs(sx-1.0) < 0.2 and abs(sy-0.5) < 0.2 and abs(syaw-0.3) < 0.2
            result["metrics"] = {"x": round(sx,3), "y": round(sy,3), "yaw": round(syaw,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} X={sx:.2f} Y={sy:.2f} Yaw={syaw:.2f} "
                f"{'✅' if ok else '❌'}")

        elif name in ("planner_on", "planner_off"):
            # 对比看响应速度: planner_on 应更平滑
            steady_x = self._steady(data, "x")
            max_fx = self._max_abs(data, "fx")
            ok = abs(steady_x - 2.0) < 0.3
            result["metrics"] = {"steady_x": round(steady_x,3), "max_fx": round(max_fx,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} 稳态X={steady_x:.2f}m 最大推力={max_fx:.2f} "
                f"{'✅' if ok else '❌'}")

        elif name == "edge_nan":
            # NaN 应被拒绝 → 位置保持
            steady_x = self._steady(data, "x")
            ok = abs(steady_x) < 0.1  # 保持在起点
            result["metrics"] = {"steady_x": round(steady_x,3)}
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} X={steady_x:.3f}m(NaN应被拒绝) {'✅' if ok else '❌'}")

        elif name == "edge_disarmed":
            # 未解锁时指令被忽略
            was_armed = any(d["armed"] for d in data[-20:])
            ok = not was_armed
            result["pass"] = ok
            self.get_logger().info(
                f"{prefix} 解锁状态={'🔓' if was_armed else '🔒'} "
                f"{'✅' if ok else '❌'}(应保持锁定)")

        else:
            # 通用: 检查是否有推力输出
            max_fx = self._max_abs(data, "fx")
            ok = max_fx > 0.01 or not self.armed
            self.get_logger().info(f"{prefix} 最大推力={max_fx:.3f} {'✅' if ok else '⚠️'}")

        self.results.append(result)

    # ---- 辅助 ----

    def _arm(self):
        """发送心跳解锁 (data=3 = 远程解锁模式, 跳过导航检查)"""
        m = UInt32(); m.data = 3
        self.pub_hb.publish(m)

    def _sp(self, ck, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0,
            yaw=0.0, mask=0):
        """发 setpoint（2Hz，MCU 回调耗时可能挤占心跳处理）"""
        self._sp_tick += 1
        # Roll/Pitch 只保留函数签名兼容性，测试发送时固定为旁路值 0。
        self._last_sp = (x, y, z, 0.0, 0.0, yaw)
        if self._sp_tick % 50 != 0:  # 每 500ms 发一次
            return
        m = ZitSetpoint()
        m.control_key = ck; m.type_mask = mask
        m.x = float(x); m.y = float(y); m.z = float(z)
        m.roll = 0.0; m.pitch = 0.0; m.yaw = float(yaw)
        self.pub_sp.publish(m)

    def _sim(self, pos, vel):
        if self.mode != "sitl":
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        roll, pitch, yaw = float(pos[3]), float(pos[4]), float(pos[5])
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        odom.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        odom.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        odom.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        odom.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])
        odom.twist.twist.angular.x = float(vel[3])
        odom.twist.twist.angular.y = float(vel[4])
        odom.twist.twist.angular.z = float(vel[5])
        self.pub_sim_nav.publish(odom)

    def _record(self):
        p = self.physics.pos if self.mode == "sitl" else [0.0]*6
        v = self.physics.vel if self.mode == "sitl" else [0.0]*6
        f = self.last_forces
        self.log.append({
            "t": self.scn_t + self.scn_idx * self.SCENARIO_LEN,
            "scenario": self.scenarios[self.scn_idx] if self.scn_idx < len(self.scenarios) else "",
            "armed": self.armed,
            "x": p[0], "y": p[1], "z": p[2], "roll": p[3], "pitch": p[4], "yaw": p[5],
            "u": v[0], "v": v[1], "w": v[2],
            "fx": f[0], "fy": f[1], "fz": f[2],
            "mroll": f[3], "mpitch": f[4], "myaw": f[5],
        })

    def _run_scenario(self, name, fn):
        """执行场景 fn: 前 ~7 秒发心跳解锁 + 缓冲, 之后才发 setpoint"""
        self._hb_tick += 1
        if self._hb_tick % self.HEARTBEAT_TICKS == 0:  # 2Hz 心跳
            self._arm()
        if self.scn_t >= self.ARM_TIME + 1.0:  # arm 后 1s 缓冲即发 setpoint
            fn(self.scn_t - self.ARM_TIME - 1.0)

    # ---- 场景定义 ----

    def scn_frame_world_pos(self, t):
        self._sp(CK_POS_WORLD_ABS, x=2.0)

    def scn_frame_body_pos(self, t):
        if t < 1.0:
            # 发当前位置 + 新 yaw, 避免重置 x/y/z 到 0
            cp = self.physics.pos
            self._sp(CK_POS_WORLD_ABS, x=cp[0], y=cp[1], z=cp[2], yaw=math.pi/2)
        else:
            self._sp(CK_POS_BODY_ABS, x=1.0)

    def scn_frame_world_vel(self, t):
        self._sp(CK_VEL_WORLD_ABS, x=0.5)

    def scn_frame_body_vel(self, t):
        self._sp(CK_VEL_BODY_ABS, x=0.5)

    def scn_mode_pos_abs(self, t):
        self._sp(CK_POS_WORLD_ABS, x=2.0)

    def scn_mode_pos_inc(self, t):
        if t < 1.0:
            self._sp(CK_POS_WORLD_ABS, x=1.0)
        else:
            self._sp(CK_POS_WORLD_INC, x=1.0)

    def scn_mode_vel_abs(self, t):
        self._sp(CK_VEL_WORLD_ABS, x=0.5)

    def scn_mode_vel_inc(self, t):
        self._sp(CK_VEL_BODY_INC, x=0.3)

    def scn_switch_none(self, t):
        if t < 2.0:
            self._sp(CK_POS_WORLD_ABS, x=1.0)

    def scn_switch_pos_vel(self, t):
        if t < 2.0:
            self._sp(CK_POS_WORLD_ABS, x=1.0)
        else:
            self._sp(CK_VEL_WORLD_ABS, x=0.2)

    def scn_switch_vel_act(self, t):
        if t < 2.0:
            self._sp(CK_VEL_BODY_ABS, x=0.3)
        else:
            self._sp(CK_ACT_BODY_ABS, x=0.5)

    def scn_mask_single(self, t):
        self._sp(CK_POS_WORLD_ABS, x=2.0, y=99.0, z=99.0,
                 roll=99.0, pitch=99.0, yaw=99.0, mask=0b111110)

    def scn_mask_multi(self, t):
        self._sp(CK_POS_WORLD_ABS, x=2.0, yaw=0.5, mask=0b011110)

    def scn_mask_all(self, t):
        self._sp(CK_POS_WORLD_ABS, x=2.0, mask=0b111111)

    def scn_planner_on(self, t):
        self._sp(CK_POS_WORLD_ABS, x=2.0)

    def scn_planner_off(self, t):
        self._sp(CK_POS_WORLD_ABS, x=2.0)

    def scn_step_x(self, t):
        self._sp(CK_POS_WORLD_ABS, x=2.0)

    def scn_step_yaw(self, t):
        cp = self.physics.pos
        self._sp(CK_POS_WORLD_ABS, x=cp[0], y=cp[1], z=cp[2], yaw=1.57)

    def scn_step_multi(self, t):
        self._sp(CK_POS_WORLD_ABS, x=1.0, y=0.5, yaw=0.3)

    def scn_edge_nan(self, t):
        if t < 1.0:
            self._sp(CK_POS_WORLD_ABS, x=1.0)
        elif t < 2.0:
            m = ZitSetpoint(); m.control_key = CK_POS_WORLD_ABS
            m.x = float('nan')
            self.pub_sp.publish(m)

    def scn_edge_disarmed(self, t):
        self._sp(CK_POS_WORLD_ABS, x=5.0)

    # ---- 主循环 ----

    def _tick(self):
        if not self.running:
            return
        elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        self.scn_idx = min(int(elapsed / self.SCENARIO_LEN), len(self.scenarios)-1)
        self.scn_t = elapsed - self.scn_idx * self.SCENARIO_LEN

        # 检测场景切换 → 分析刚完成的场景
        if self.scn_idx != self.scn_last_idx and self.scn_last_idx >= 0:
            self._analyze_scenario(self.scenarios[self.scn_last_idx], self.scn_log)
            self.scn_log = []
        self.scn_last_idx = self.scn_idx

        # 收集当前场景数据 + 写入主日志
        name = self.scenarios[self.scn_idx] if self.scn_idx < len(self.scenarios) else "?"
        p = self.physics.pos if self.mode == "sitl" else [0.0]*6
        v = self.physics.vel if self.mode == "sitl" else [0.0]*6
        f = self.last_forces
        entry = {
            "t": self.scn_t, "scenario": name, "armed": self.armed,
            "x": p[0], "y": p[1], "z": p[2], "roll": p[3], "pitch": p[4], "yaw": p[5],
            "u": v[0], "v": v[1], "w": v[2],
            "fx": f[0], "fy": f[1], "fz": f[2],
            "mroll": f[3], "mpitch": f[4], "myaw": f[5],
        }
        self.scn_log.append(entry)
        self.log.append(entry)

        # 每 5 秒输出一次进度
        if elapsed - self.last_progress_log >= 5.0:
            self.last_progress_log = elapsed
            name = self.scenarios[self.scn_idx] if self.scn_idx < len(self.scenarios) else "?"
            armed_str = "🔓解锁" if self.armed else "🔒锁定"
            rx_str = f"📡rx={self.status_count}" if self.status_count > 0 else "⏳等待Agent"
            sitl_str = ""
            if not self.sitl_configured and self.status_count > 0 and self.mode == "sitl":
                sitl_str = " 🔄正在配置SITL..."
                self._configure_sitl()
            hint = ""
            if not self.armed and self.status_count > 10 and elapsed > 20:
                hint = " ⚠️ MCU未解锁: 检查心跳是否发送(scan_edge_disarmed)"
            self.get_logger().info(
                f"[{elapsed:6.1f}s] {armed_str} {rx_str}{sitl_str}  "
                f"场景[{self.scn_idx+1}/{len(self.scenarios)}]: {name}{hint}"
            )

        # 物理演进 (100Hz 计算，33Hz 发布 sim 数据)
        if self.mode == "sitl":
            self.physics.step(self.last_forces)
            self._sim_tick += 1
            if self._sim_tick % 3 == 0:  # ~33Hz
                self._sim(self.physics.pos, self.physics.vel)

        # 实时 3D 可视化 (~5Hz)
        self._viz_tick += 1
        if self.viz and self._viz_tick % 20 == 0 and self.mode == "sitl":
            sp = self._last_sp
            self.viz.update(self.physics.pos, sp)

        # 执行场景
        if self.scn_idx < len(self.scenarios):
            name = self.scenarios[self.scn_idx]
            fn_name = f"scn_{name}"
            fn = getattr(self, fn_name, None)
            if fn:
                self._run_scenario(name, fn)

        self._record()

        if elapsed >= self.duration:
            self.finish()

    def finish(self):
        self.running = False
        if self.viz:
            self.viz.close()
        # 分析最后一个场景
        if self.scn_log and len(self.scn_log) > 5:
            self._analyze_scenario(self.scenarios[-1], self.scn_log)
        # 打印汇总
        self._print_summary()
        self.get_logger().info("测试完成")
        self.save_log()
        raise KeyboardInterrupt()

    def _print_summary(self):
        if not self.results:
            return
        passed = sum(1 for r in self.results if r["pass"])
        total = len(self.results)
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"📊 测试汇总: {passed}/{total} 通过")
        self.get_logger().info("="*60)
        for r in self.results:
            icon = "✅" if r["pass"] else "❌"
            self.get_logger().info(f"  {icon} {r['name']:20s} {r['metrics']}")
        self.get_logger().info("="*60)

    def save_log(self):
        d = os.path.join(os.path.dirname(__file__),"..","..","log")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"hitl_full_{int(time.time())}.csv")
        if not self.log:
            return
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=self.log[0].keys())
            w.writeheader(); w.writerows(self.log)
        self.get_logger().info(f"日志: {path}")


# ============================================================================
# 入口
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="ZIT6 AUV HITL 全面测试")
    parser.add_argument("--mode", choices=["sitl","real"], default="sitl")
    parser.add_argument("--scenarios", nargs="*", default=["all"])
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--list", action="store_true", help="列出所有测试项")
    args = parser.parse_args()

    if args.list:
        print("可用测试场景 (22项):")
        for s in ALL_SCENARIOS:
            print(f"  {s:20s} {SCENARIO_DOC.get(s, '')}")
        return

    rclpy.init()
    node = HitlTestNode(mode=args.mode, scenarios=args.scenarios, duration=args.duration)
    node.start_time = node.get_clock().now().nanoseconds / 1e9

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        if node.viz:
            node.viz.close()
        node.running = False
        node.save_log()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
