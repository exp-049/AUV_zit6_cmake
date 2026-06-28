#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import json
import threading

import rclpy
from rclpy.node import Node
from zit6_interfaces.srv import GetParams, UpdateParams

# 导入共享的浮动心跳面板
from .heartbeat import FloatingHeartbeatPanel

# Qt imports
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                 QTableWidget, QTableWidgetItem, QPushButton, QLabel, QHeaderView, QMessageBox)
    from PyQt5.QtCore import Qt, pyqtSignal, QTimer
except ImportError:
    from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                   QTableWidget, QTableWidgetItem, QPushButton, QLabel, QHeaderView, QMessageBox)
    from PySide6.QtCore import Qt, Signal as pyqtSignal, QTimer

class ConfigWidget(QWidget):
    """
    参数配置组件，支持在主控制台中嵌入或在独立窗口中显示
    """
    get_signal = pyqtSignal(dict)
    update_signal = pyqtSignal(bool, str)

    def __init__(self, node):
        super().__init__()
        self.node = node
        
        self.get_client = self.node.create_client(GetParams, '/zit6/get_params')
        self.update_client = self.node.create_client(UpdateParams, '/zit6/update_params')
        
        self.params_map = {}
        # 从脚本位置推导项目根
        self._project_root = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
        self.config_path = os.path.join(self._project_root, 'UserApp', 'Config', 'config.json')
        self._int_paths = {"soft_watchdog.timeout_ms"}
        self._bool_paths = {
            "soft_watchdog.check_microros",
            "soft_watchdog.check_ins",
            "soft_watchdog.check_depth",
            "chassis.planner_enabled",
            "simulation.hitl_enabled",
            "simulation.sitl_enabled",
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
        
        self.init_ui()
        self.get_signal.connect(self.update_table_values)
        self.update_signal.connect(self.show_update_result)
        
        self.load_structure()
        QTimer.singleShot(1000, lambda: self.fetch_params([]))

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["参数路径 (Path)", "当前值 (Current)", "新值 (New)"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setStyleSheet("""
            QTableWidget {
                gridline-color: #333333; 
                background-color: #1e1e1e; 
                color: #e0e0e0;
                border: 1px solid #333333;
                border-radius: 6px;
            }
            QHeaderView::section {
                background-color: #2b2b2b;
                color: #ffffff;
                padding: 6px;
                border: 1px solid #333333;
            }
        """)
        layout.addWidget(self.table)
        
        btn_layout = QHBoxLayout()
        self.btn_get = QPushButton("从 AUV 获取")
        self.btn_set = QPushButton("应用到 AUV")
        self.btn_save = QPushButton("保存到本地 JSON")
        
        for btn in [self.btn_get, self.btn_set, self.btn_save]:
            btn.setFixedHeight(38)
            btn.setStyleSheet("""
                QPushButton {
                    font-weight: bold; 
                    border-radius: 5px;
                    border: none;
                    padding: 0 15px;
                }
            """)
        
        self.btn_get.setStyleSheet("background-color: #0d47a1; color: white;")
        self.btn_set.setStyleSheet("background-color: #1b5e20; color: white;")
        self.btn_save.setStyleSheet("background-color: #455a64; color: white;")
        
        self.btn_get.clicked.connect(lambda: self.fetch_params([]))
        self.btn_set.clicked.connect(self.on_apply)
        self.btn_save.clicked.connect(self.on_save_json)
        
        btn_layout.addWidget(self.btn_get)
        btn_layout.addWidget(self.btn_set)
        btn_layout.addWidget(self.btn_save)
        layout.addLayout(btn_layout)

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
        from ament_index_python.packages import get_package_share_directory
        search_paths = [
            self.config_path,
            os.path.join(os.getcwd(), 'UserApp/Config/config.json'),
            os.path.join(os.getcwd(), 'config.json'),
        ]
        
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

    def fetch_params(self, paths=[]):
        if not self.get_client.wait_for_service(timeout_sec=1.0):
            print("GetParams service not available.")
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

    def update_table_values(self, data):
        for path, val in data.items():
            if path in self.params_map:
                if path in self._enum_paths and isinstance(val, str):
                    val = self._normalize_enum_value(path, val)
                row = self.params_map[path]
                self.table.setItem(row, 1, QTableWidgetItem(self._format_value(val)))
                self.table.item(row, 1).setFlags(Qt.ItemIsEnabled)
            else:
                # 固件中有但本地 config.json 没有的参数（如 firmware.version）
                row = self.table.rowCount()
                self.table.insertRow(row)
                self.table.setItem(row, 0, QTableWidgetItem(path))
                self.table.item(row, 0).setFlags(Qt.ItemIsEnabled)
                self.table.setItem(row, 1, QTableWidgetItem(self._format_value(val)))
                self.table.item(row, 1).setFlags(Qt.ItemIsEnabled)
                self.table.setItem(row, 2, QTableWidgetItem(self._format_value(val)))
                self.params_map[path] = row

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
            QMessageBox.warning(self, "非法Enum值", f"参数包含未被允许的Enum项:\n{lines}")
        
        if paths:
            self.push_params(paths, values)
        else:
            QMessageBox.information(self, "提示", "参数未发生任何修改。")

    def push_params(self, paths, values):
        if not self.update_client.wait_for_service(timeout_sec=1.0):
            QMessageBox.critical(self, "连接错误", "无法连接到参数更新服务！")
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

    def show_update_result(self, success, message):
        if success:
            QMessageBox.information(self, "成功", f"参数更新成功!\n{message}")
            self.fetch_params([])
        else:
            QMessageBox.critical(self, "失败", f"更新失败:\n{message}")

    def on_save_json(self):
        flat_data = {}
        for i in range(self.table.rowCount()):
            path = self.table.item(i, 0).text()
            val_str = self.table.item(i, 2).text()
            flat_data[path] = self._infer_value_from_str(path, val_str)
        
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
            QMessageBox.information(self, "保存成功", f"配置已保存到:\n{self.config_path}")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存 JSON 失败:\n{e}")

    def close(self):
        pass


class ConfigApp(QMainWindow):
    """
    配置参数独立 GUI 窗口
    """
    def __init__(self, node, spinner):
        super().__init__()
        self.node = node
        self.spinner = spinner
        self.setWindowTitle("Zit6 AUV 参数调试器")
        self.resize(900, 780)
        self.setStyleSheet("QMainWindow { background-color: #121212; }")
        
        self.widget = ConfigWidget(self.node)
        self.setCentralWidget(self.widget)
        
        # 统一的浮动心跳下发面板
        self.floating_hbt = FloatingHeartbeatPanel(self.node, self)
        self.floating_hbt.show()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.floating_hbt.setGeometry(self.width() - 280, 15, 260, 42)
        self.floating_hbt.raise_()

    def closeEvent(self, event):
        self.widget.close()
        self.floating_hbt.close()
        super().closeEvent(event)


def main(args=None):
    parser = argparse.ArgumentParser(description="Zit6 参数配置命令行工具")
    parser.add_argument("--get", nargs="*", metavar="PATH", help="从 AUV 中获取指定路径的参数，缺省表示获取所有")
    parser.add_argument("--set", nargs="+", metavar="PATH=VALUE", help="设置 AUV 的参数，格式为 path=value")
    
    parsed_args, unknown = parser.parse_known_args(args=args)

    if parsed_args.get is not None or parsed_args.set is not None:
        rclpy.init(args=args)
        node = Node('config_setter_cli')
        
        get_client = node.create_client(GetParams, '/zit6/get_params')
        update_client = node.create_client(UpdateParams, '/zit6/update_params')
        
        if parsed_args.get is not None:
            if not get_client.wait_for_service(timeout_sec=2.0):
                print("Error: /zit6/get_params service is not available.")
                sys.exit(1)
            req = GetParams.Request()
            req.paths = parsed_args.get
            future = get_client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            
            try:
                res = future.result()
                if res.success:
                    print(res.config_json)
                else:
                    print("Error: Failed to fetch parameters.")
            except Exception as e:
                print(f"Service call failed: {e}")
                
        elif parsed_args.set is not None:
            paths, values = [], []
            for pair in parsed_args.set:
                if '=' not in pair:
                    print(f"Error: Invalid set argument format '{pair}'. Expected path=value.")
                    sys.exit(1)
                p, v = pair.split('=', 1)
                paths.append(p.strip())
                values.append(v.strip())
                
            if not update_client.wait_for_service(timeout_sec=2.0):
                print("Error: /zit6/update_params service is not available.")
                sys.exit(1)
            req = UpdateParams.Request()
            req.paths = paths
            req.values = values
            future = update_client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            
            try:
                res = future.result()
                if res.success:
                    print(f"Success: {res.message}")
                else:
                    print(f"Failed: {res.message}")
            except Exception as e:
                print(f"Service call failed: {e}")
                
        node.destroy_node()
        rclpy.shutdown()
    else:
        rclpy.init(args=args)
        node = Node('config_gui_node')
        
        spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spinner.start()
        
        app = QApplication(sys.argv)
        app.setStyle("Fusion")
        
        window = ConfigApp(node, spinner)
        window.show()
        
        exit_code = app.exec_()
        
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)

if __name__ == '__main__':
    main()
