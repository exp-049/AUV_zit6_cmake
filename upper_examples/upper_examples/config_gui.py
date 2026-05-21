#!/usr/bin/env python3
import sys
import json
import os
from typing import Dict, Any

# Qt & ROS
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                 QTableWidget, QTableWidgetItem, QPushButton, QLabel, QHeaderView, QMessageBox)
    from PyQt5.QtCore import Qt, QThread, pyqtSignal
except ImportError:
    from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                   QTableWidget, QTableWidgetItem, QPushButton, QLabel, QHeaderView, QMessageBox)
    from PySide6.QtCore import Qt, QThread, Signal as pyqtSignal

import rclpy
from rclpy.node import Node
from zit6_interfaces.srv import GetParams, UpdateParams

class RosWorker(QThread):
    get_signal = pyqtSignal(dict)
    update_signal = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.node = None
        self.get_client = None
        self.update_client = None

    def run(self):
        rclpy.init()
        self.node = Node('config_gui_node')
        self.get_client = self.node.create_client(GetParams, '/zit6/get_params')
        self.update_client = self.node.create_client(UpdateParams, '/zit6/update_params')
        rclpy.spin(self.node)

    def fetch_params(self, paths=[]):
        if not self.get_client.wait_for_service(timeout_sec=1.0):
            return
        req = GetParams.Request()
        req.paths = paths
        future = self.get_client.call_async(req)
        future.add_done_callback(self._fetch_done)

    def _fetch_done(self, future):
        try:
            res = future.result()
            if res.success:
                data = json.loads(res.config_json)
                self.get_signal.emit(data)
        except Exception as e:
            print(f"Fetch failed: {e}")

    def push_params(self, paths, values):
        if not self.update_client.wait_for_service(timeout_sec=1.0):
            return
        req = UpdateParams.Request()
        req.paths = paths
        req.values = [str(v) for v in values]
        future = self.update_client.call_async(req)
        future.add_done_callback(self._update_done)

    def _update_done(self, future):
        try:
            res = future.result()
            self.update_signal.emit(res.success, res.message)
        except Exception as e:
            self.update_signal.emit(False, str(e))

class ConfigApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Zit6 AUV Parameter Tuner")
        self.resize(900, 700)
        self.params_map = {} # path -> row_idx
        self.config_path = os.path.join(os.getcwd(), 'config.json')
        self._int_paths = {"soft_watchdog.timeout_ms"}
        self._bool_paths = {
            "soft_watchdog.check_microros",
            "soft_watchdog.check_ins",
            "soft_watchdog.check_depth",
            "chassis.planner_enabled",
            "simulation.hitl_enabled",
        }
        self._enum_paths = {"z_data_sourse"}
        self._enum_allowed = {
            "z_data_sourse": {
                "use_ins_integrated_z",
                "use_ms5837_z",
                "use_ins_pressure_z",
                "use_manometer_z",
            }
        }
        self._enum_aliases = {
            "use_manometer_z": "use_ins_pressure_z",
        }
        self._types_map = {}

        # UI Layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # Header
        header = QLabel("Zit6 System Configuration")
        header.setStyleSheet("font-size: 18px; font-weight: bold; margin: 10px;")
        layout.addWidget(header)

        # Table
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Parameter Path", "Current Value", "New Value"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setStyleSheet("gridline-color: #444; background-color: #2b2b2b; color: #eee;")
        layout.addWidget(self.table)

        # Buttons
        btn_layout = QHBoxLayout()
        self.btn_get = QPushButton("Fetch from AUV")
        self.btn_set = QPushButton("Apply to AUV")
        self.btn_save = QPushButton("Save to Local JSON")
        
        for btn in [self.btn_get, self.btn_set, self.btn_save]:
            btn.setFixedHeight(40)
            btn.setStyleSheet("font-weight: bold; border-radius: 5px;")
        
        self.btn_get.setStyleSheet("background-color: #0d47a1; color: white;")
        self.btn_set.setStyleSheet("background-color: #1b5e20; color: white;")
        self.btn_save.setStyleSheet("background-color: #455a64; color: white;")
        
        btn_layout.addWidget(self.btn_get)
        btn_layout.addWidget(self.btn_set)
        btn_layout.addWidget(self.btn_save)
        layout.addLayout(btn_layout)

        # Logic
        self.worker = RosWorker()
        self.worker.get_signal.connect(self.update_table_values)
        self.worker.update_signal.connect(self.show_update_result)
        self.worker.start()

        self.btn_get.clicked.connect(lambda: self.worker.fetch_params([]))
        self.btn_set.clicked.connect(self.on_apply)
        self.btn_save.clicked.connect(self.on_save_json)

        # 启动后自动执行一次全量获取
        try:
            from PyQt5.QtCore import QTimer
        except ImportError:
            from PySide6.QtCore import QTimer 
        QTimer.singleShot(1000, lambda: self.worker.fetch_params([]))

        self.load_structure()

    def flatten_json(self, data, prefix=""):
        res = {}
        for k, v in data.items():
            if not prefix and k == "types":
                continue
            path = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict):
                res.update(self.flatten_json(v, path))
            else:
                res[path] = v
        return res

    def _format_value(self, val):
        if isinstance(val, bool):
            return "true" if val else "false"
        return str(val)

    def _apply_types_map(self, types_map):
        for path, t in types_map.items():
            t_norm = str(t).strip().lower()
            if t_norm in ("uint32", "uint32_t", "int32", "int32_t"):
                self._int_paths.add(path)
            elif t_norm in ("bool", "boolean"):
                self._bool_paths.add(path)
            elif t_norm.startswith("enum"):
                self._enum_paths.add(path)

    def _normalize_enum_value(self, path, val_str):
        v = val_str.strip().lower()
        return self._enum_aliases.get(v, v)

    def _is_enum_value_valid(self, path, val_str):
        allowed = self._enum_allowed.get(path)
        if not allowed:
            return True
        return val_str in allowed

    def _normalize_value_for_send(self, path, val_str):
        s = val_str.strip()
        if path in self._bool_paths:
            low = s.lower()
            if low in ("true", "1"):
                return "true"
            if low in ("false", "0"):
                return "false"
        if path in self._int_paths:
            try:
                return str(int(float(s)))
            except ValueError:
                return s
        if path in self._enum_paths:
            return self._normalize_enum_value(path, s)
        return s

    def _infer_value_from_str(self, path, val_str):
        s = val_str.strip()
        low = s.lower()
        if path in self._bool_paths:
            if low in ("true", "1"):
                return True
            if low in ("false", "0"):
                return False
        if path in self._enum_paths:
            return self._normalize_enum_value(path, s)
        if path in self._int_paths:
            try:
                return int(float(s))
            except ValueError:
                return s
        try:
            return float(s)
        except ValueError:
            return s

    def load_structure(self):
        # 优化：优先使用 ROS 2 路径查找，其次是当前工作目录
        from ament_index_python.packages import get_package_share_directory
        search_paths = [
            os.path.join(os.getcwd(), 'config.json'),
            os.path.join(os.path.dirname(__file__), '../../config.json'), # 源码路径
            self.config_path # 默认路径
        ]
        
        # 如果在 ROS 环境运行，添加 share 目录（如果 config.json 被安装了）
        try:
            share_dir = get_package_share_directory('upper_examples')
            search_paths.insert(0, os.path.join(share_dir, 'config.json'))
        except:
            pass
        
        found = False
        for p in search_paths:
            if os.path.exists(p):
                self.config_path = p
                found = True
                break
        
        if not found:
            print("Warning: config.json not found in search paths.")
            return

        try:
            with open(self.config_path, 'r') as f:
                config = json.load(f)
                if isinstance(config.get("types"), dict):
                    self._types_map = config.get("types")
                    self._apply_types_map(self._types_map)
                flat = self.flatten_json(config)
                self.table.setRowCount(len(flat))
                for i, (path, val) in enumerate(flat.items()):
                    if path in self._enum_paths and isinstance(val, str):
                        val = self._normalize_enum_value(path, val)
                    self.table.setItem(i, 0, QTableWidgetItem(path))
                    self.table.item(i, 0).setFlags(Qt.ItemIsEnabled)
                    self.table.setItem(i, 1, QTableWidgetItem(self._format_value(val)))
                    self.table.item(i, 1).setFlags(Qt.ItemIsEnabled)
                    self.table.setItem(i, 2, QTableWidgetItem(self._format_value(val)))
                    self.params_map[path] = i
        except Exception as e:
            print(f"Failed to load config.json: {e}")

    def update_table_values(self, data):
        for path, val in data.items():
            if path in self.params_map:
                if path in self._enum_paths and isinstance(val, str):
                    val = self._normalize_enum_value(path, val)
                row = self.params_map[path]
                self.table.setItem(row, 1, QTableWidgetItem(self._format_value(val)))
                self.table.item(row, 1).setFlags(Qt.ItemIsEnabled)

    def on_apply(self):
        paths, values = [], []
        invalid_enums = []
        for i in range(self.table.rowCount()):
            path = self.table.item(i, 0).text()
            cur_val = self.table.item(i, 1).text()
            new_val = self.table.item(i, 2).text()
            if cur_val != new_val:
                normalized = self._normalize_value_for_send(path, new_val)
                if path in self._enum_paths and not self._is_enum_value_valid(path, normalized):
                    invalid_enums.append((path, new_val))
                    continue
                paths.append(path)
                values.append(normalized)

        if invalid_enums:
            lines = "\n".join([f"{p}: {v}" for p, v in invalid_enums])
            QMessageBox.warning(self, "Invalid Enum", f"Invalid enum values:\n{lines}")
        
        if paths:
            self.worker.push_params(paths, values)
        else:
            QMessageBox.information(self, "No Changes", "No parameters modified.")

    def on_save_json(self):
        # 收集当前表格中的所有“新值”并还原为 JSON
        flat_data = {}
        for i in range(self.table.rowCount()):
            path = self.table.item(i, 0).text()
            val_str = self.table.item(i, 2).text()
            flat_data[path] = self._infer_value_from_str(path, val_str)
        
        # 反展平
        nested_data = {}
        for path, val in flat_data.items():
            parts = path.split('.')
            d = nested_data
            for part in parts[:-1]:
                if part not in d: d[part] = {}
                d = d[part]
            d[parts[-1]] = val

        if self._types_map:
            nested_data["types"] = self._types_map
        
        try:
            with open(self.config_path, 'w') as f:
                json.dump(nested_data, f, indent=2)
            QMessageBox.information(self, "Saved", f"Configuration saved to:\n{self.config_path}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save JSON:\n{e}")

    def show_update_result(self, success, message):
        if success:
            QMessageBox.information(self, "Success", f"Parameters updated successfully!\n{message}")
            self.worker.fetch_params([]) # Refresh
        else:
            QMessageBox.critical(self, "Error", f"Failed to update parameters:\n{message}")

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = ConfigApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
