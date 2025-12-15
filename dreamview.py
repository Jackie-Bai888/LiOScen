from websocket import create_connection
import json
# import simulate as si

def connnect_dreaview():
    ip = '127.0.0.1'
    port = '8888'
    url = "ws://" + ip + ":" + port + "/websocket"
    ws = create_connection(url)
    return ws

def send_info(ws, start_location, end_location):
    json_data = json.dumps(
        {
            "type": "SendRoutingRequest",
            "start": {
                "x": start_location[0],
                "y": -start_location[1],
                "z": start_location[2],
            },
            "end": {
                "x": end_location[0],
                "y": -end_location[1],
                "z": end_location[2],
            },
            "waypoint": "[]"
        }

    )
    ws.send(json_data)

if __name__ == '__main__':
    ws = connnect_dreaview()
    s_x = 383.1048889160156
    s_y = -18.292522430419922
    s_z = 0
    e_x = 389.9281311035156
    e_y = -16.72943878173828
    e_z = 0
    send_info(ws, (s_x, s_y, s_z), (e_x, e_y, e_z))


# s = ws.send(
#             json.dumps(
#                 {"body":{"end":{"latitude":39.98,"longitude":116.33},"start":{"latitude":39.91,"longitude":116.4}},"header":{"id":"1234567890","timestamp":"2023-04-29T10:00:00.000Z","type":"RoutingRequest"}}
#             )
#         )
# print(s)
# # 接收返回数据
# response = ws.recv()
# print(response)
# 关闭WebSocket连接
# ws.close()



# import websocket
# import json
#
# # WebSocket连接配置
# websocket_url = "ws://127.0.0.1:8888/websocket"  # 替换成你的实际地址
# ws = websocket.create_connection(websocket_url)
#
# # 构建控制指令
# control_command = {
#     "type": "ControlCommand",
#     "command": "START",  # 替换成你需要的控制指令，比如START, STOP, STEER, etc.
#     "param1": 0,  # 根据具体指令可能需要提供的参数进行设置
#     "param2": 0
# }
#
# # 将控制指令转换为JSON格式并发送
# ws.send(json.dumps(control_command))
# # 接收返回数据
# response = ws.recv()
# print(response)
# # 关闭WebSocket连接
# ws.close()



# waypoint {
#   id: "road_4_lane_0_-1"
#   s: 118.71761810473971
#   pose {
#     x: 220.13681030273438
#     y: -133.24014282226562
#   }
# }
# waypoint {
#   id: "road_4_lane_0_-1"
#   s: 130.7117501971012
#   pose {
#     x: 232.13099613075892
#     y: -133.12255275830441
#   }
# }

# {"body":{"end":{"latitude":39.98,"longitude":116.33},"start":{"latitude":39.91,"longitude":116.4}},"header":{"id":"1234567890","timestamp":"2023-04-29T10:00:00.000Z","type":"RoutingRequest"}}

# waypoint {
#   id: "road_4_lane_0_-1"
#   s: 46.0739963707354
#   pose {
#     x: 147.49343872070312
#     y: -132.66403198242188
#   }
# }
# waypoint {
#   id: "road_4_lane_0_-1"
#   s: 63.644991008803295
#   pose {
#     x: 165.06419513194845
#     y: -133.20114926479704
#   }
# }
#
#
# Get new routing request:header {
#   timestamp_sec: 1702285292.6517327
#   module_name: "dreamview"
#   sequence_num: 5
# }
# waypoint {
#   id: "road_4_lane_0_-1"
#   s: 46.0739963707354
#   pose {
#     x: 147.49343872070312
#     y: -132.66403198242188
#   }
# }
# waypoint {
#   id: "road_4_lane_0_-1"
#   s: 63.644991008803295
#   pose {
#     x: 165.06419513194845
#     y: -133.20114926479704
#   }
# }
