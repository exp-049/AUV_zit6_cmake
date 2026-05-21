import rclpy, json, os
from rclpy.node import Node
from zit6_interfaces.srv import UpdateParams

def main():
    rclpy.init()
    node = Node('upd_client')
    client = node.create_client(UpdateParams, 'zit6/update_params')
    if client.wait_for_service(timeout_sec=5.0):
        req = UpdateParams.Request()
        # 尝试从同目录的 setcfg.json 加载配置，找不到或解析失败时使用默认值
        cfg_path = os.path.join(os.path.dirname(__file__), 'setcfg.json')
        try:
            with open(cfg_path, 'r', encoding='utf-8') as f:
                cfg_text = f.read()
            # 验证 JSON 合法性
            json.loads(cfg_text)
        except Exception:
            cfg_text = json.dumps({
                "chassis": {
                    "x": {
                        "pos_kp": 0.02,
                        "pos_ki": 0.001,
                        "max_v": 0.6,
                        "max_a": 0.2
                    }
                }
            })
        req.json = cfg_text
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        print(future.result().success, future.result().message)
    rclpy.shutdown()

if __name__ == '__main__':
    main()