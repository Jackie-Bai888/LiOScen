import carla
import time
import argparse
import math
from getting_stop_line import get_stop_line

def main(world, e_x, e_y, t_x, t_y, stop_event):
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    if not traffic_lights:
        print("No traffic lights found in the map.")
        return
    _, sl = get_stop_line(world, e_x, e_y)
    sl_lo = carla.Location(x=int(sl['x']), y=-int(sl['y']), z=0)
    e_tf, n_tf = None, None
    for tf in traffic_lights:
        lc = tf.get_location()
        print(int(lc.x), e_x, int(lc.y), e_y)
        if int(lc.x) == e_x and int(lc.y) == e_y:
            e_tf = tf
        if int(lc.x) == t_x and int(lc.y) == t_y:
            n_tf = tf
        if e_tf and n_tf:
            break

    if n_tf:
        # 冻结交通灯，防止自动切换
        group = n_tf.get_group_traffic_lights()
        for light in group:
            light.freeze(True)
        # n_tf.freeze(True)
        n_tf.set_state(carla.TrafficLightState.Green)
    if e_tf:
        group = e_tf.get_group_traffic_lights()  # 获取组内所有交通灯
        for light in group:
            light.freeze(True)  # 冻结整个组
        # e_tf.freeze(True)
        e_tf.set_state(carla.TrafficLightState.Green)

    # 获取所有车辆的列表
    vehicle_list = world.get_actors().filter('vehicle.*')
    ego_vehicle = None
    if len(vehicle_list) != 0:
        for actor in vehicle_list:
            if actor.type_id == 'vehicle.lincoln.mkz_2017':
                ego_vehicle = actor
                break

    while not stop_event.is_set():
        # # 绿灯
        # e_tf.set_state(carla.TrafficLightState.Green)
        # print(f"Traffic light {e_tf.id} set to Green")
        # time.sleep(0.5)  # 绿灯持续时间
        #
        # # 黄灯
        # e_tf.set_state(carla.TrafficLightState.Yellow)
        # print(f"Traffic light {e_tf.id} set to Yellow")
        # time.sleep(0.5)  # 黄灯持续时间
        #
        # # 红灯
        # e_tf.set_state(carla.TrafficLightState.Red)
        # print(f"Traffic light {e_tf.id} set to Red")
        # time.sleep(0.5)  # 红灯持续时间
        may_run_red, time_to_red = predict_red_light_violation(ego_vehicle, sl_lo, 1.5)
        if may_run_red:
            # 黄灯
            e_tf.set_state(carla.TrafficLightState.Yellow)
            print(f"Traffic light {e_tf.id} set to Yellow")
            # 黄灯持续时间/时间间隔
            time.sleep(0.2)
        #     # # 红灯
            print(f"Traffic light {e_tf.id} set to Red")
            while not stop_event.is_set():
                e_tf.set_state(carla.TrafficLightState.Red)
                time.sleep(0.1)


def init_tow_traffic_light(world, e_tf, t_x, t_y):
    traffic_lights = world.get_actors().filter('traffic.traffic_light')

    if not traffic_lights:
        print("No traffic lights found in the map.")
        return

    n_tf = None
    for tf in traffic_lights:
        lc = tf.get_location()
        if int(lc.x) == t_x and int(lc.y) == t_y:
            n_tf = tf
            break

    if n_tf:
        n_tf.set_state(carla.TrafficLightState.Green)
    if e_tf:
        e_tf.set_state(carla.TrafficLightState.Green)

def change_traffic_light(e_tf):

    # 黄灯
    e_tf.set_state(carla.TrafficLightState.Yellow)
    print(f"Traffic light {e_tf.id} set to Yellow")
    time.sleep(2)

    # 红灯
    e_tf.set_state(carla.TrafficLightState.Red)
    print(f"Traffic light {e_tf.id} set to Red")

def predict_red_light_violation(vehicle, sl_lo, threshold_time=3.0):
    vehicle_location = vehicle.get_location()
    vehicle_velocity = vehicle.get_velocity()
    vehicle_acceleration = vehicle.get_acceleration()

    # 计算距离
    # traffic_light_location = e_tf.get_location()
    distance_to_stop_line = vehicle_location.distance(sl_lo)

    # 计算当前速度和加速度
    speed_mps = math.sqrt(vehicle_velocity.x ** 2 + vehicle_velocity.y ** 2 + vehicle_velocity.z ** 2)
    acceleration_mps2 = math.sqrt(
        vehicle_acceleration.x ** 2 + vehicle_acceleration.y ** 2 + vehicle_acceleration.z ** 2)

    # 使用匀变速运动公式预测到达时间
    # s = v0 * t + 0.5 * a * t^1
    # 解方程求t
    a = acceleration_mps2
    v0 = speed_mps
    s = distance_to_stop_line

    # 计算到达时间（假设加速度方向与速度方向一致）
    discriminant = v0 ** 2 + 2 * a * s
    if discriminant < 0:
        time_to_stop_line = float('inf')
    else:
        t1 = (-v0 + math.sqrt(discriminant)) / a
        # t2 = (-v0 - math.sqrt(discriminant)) / a
        # time_to_stop_line = min(t1, t2) if a > 0 else max(t1, t2)
        time_to_stop_line = t1
    # print(time_to_stop_line, speed_mps)
    # 判断是否可能闯红灯
    if time_to_stop_line < threshold_time and speed_mps > 3.0:
        return True, time_to_stop_line
    return False, time_to_stop_line

if __name__ == '__main__':
    # 配置命令行参数解析
    parser = argparse.ArgumentParser(description='Control CARLA traffic light by coordinates')
    parser.add_argument('e_tf_x', type=int, help='X coordinate of the traffic light')
    parser.add_argument('e_tf_y', type=int, help='Y coordinate of the traffic light')
    parser.add_argument('n_tf_x', type=int, help='X coordinate of the traffic light')
    parser.add_argument('n_tf_y', type=int, help='Y coordinate of the traffic light')
    args = parser.parse_args()

    # 执行主程序
    main(args.e_tf_x, args.e_tf_y, args.n_tf_x, args.n_tf_y)