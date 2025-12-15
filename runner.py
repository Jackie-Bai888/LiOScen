import subprocess
import os
import sys
import carla
import simulator as sim
import dreamview as dv
import time
from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
from set_traffic_signal import main as set_ts_main
from mutation import mutate_direction, mutate
import threading
from getting_stop_line import get_stop_line
import math
import feedback as fd
import shutil
import scenario
import random
from sep_ego_npc_tl import modify_xml, extract_data
import xml.etree.ElementTree as ET
from pathlib import Path

# 全局变量
should_exit = False
scenario_process = None
collision_sensor = None  # 碰撞传感器
collision_event = threading.Event()  # 碰撞事件标志
npc_targe_speed = 5
# 初始交通管理器端口
traffic_manager_port = 8000
Max_Cycle_times = 10000
Max_Mutation_times = 5
# ORIGINAL_FILE = "/home/hahabai/PycharmProjects/ego_crime/generation_opensceanrio_files/have_temp/change_lane.xosc"
# NEW_FILE = "/home/hahabai/PycharmProjects/ego_crime/generation_opensceanrio_files/mutation/queue/change_lane.xosc"
ORIGINAL_FILE = "/home/hahabai/PycharmProjects/ego_crime/generation_opensceanrio_files/no_crash_information/new/intersection.xosc"
NEW_FILE = "/home/hahabai/PycharmProjects/ego_crime/generation_opensceanrio_files/mutation/queue/intersection.xosc"


def initialize_simulation():
    """初始化Carla连接并返回客户端、世界和地图对象"""
    client, world, carla_map = sim.inital_carla()
    return client, world, carla_map

def reset_simulation(world):
    """重置仿真环境，删除所有NPC车辆"""
    # 获取所有车辆的列表
    vehicle_list = world.get_actors().filter('vehicle.*')
    # 删除所有NPC车辆
    for actor in vehicle_list:
        if actor.type_id != 'vehicle.lincoln.mkz_2017':
            actor.destroy()
    pedestrians = world.get_actors().filter('walker.pedestrian.*')
    if pedestrians:
        for pedestrian in pedestrians:
            pedestrian.destroy()
    time.sleep(1)  # 等待车辆完全销毁

def create_ego(world, ego_ini_tf):
    # 获取所有车辆的列表
    vehicle_list = world.get_actors().filter('vehicle.*')
    if len(vehicle_list) != 0:
        for actor in vehicle_list:
            if actor.type_id == 'vehicle.lincoln.mkz_2017':
                ego_vehicle = actor
                ego_vehicle.set_transform(ego_ini_tf)
                return ego_vehicle

def sent_route(s_x, s_y, s_z, e_x, e_y, e_z):
    ws = dv.connnect_dreaview()
    dv.send_info(ws, (s_x, s_y, s_z), (e_x, e_y, e_z))
    ws.close()

def traffic_signal_stop_flag(stop_event):
    """设置信号停止标志，用于通知 set_traffic_signal.py 停止"""
    stop_event.set()

def red_cross_stop_line(world, e_tf_x, e_tf_y, ego_vehicle, stop_event):
    # global should_exit
    tl, sl = get_stop_line(world, e_tf_x, e_tf_y)
    print('******')
    print(sl)
    print('******')
    sl_lo = carla.Location(x=int(sl['x']), y=-int(sl['y']), z=0)
    # 记录函数开始时间
    start_time = time.time()
    while not stop_event.is_set():
        vehicle_transform = ego_vehicle.get_transform()
        forward_vector = vehicle_transform.get_forward_vector()
        bounding_box = ego_vehicle.bounding_box

        # 计算车辆前端位置
        front_location = vehicle_transform.location + forward_vector * bounding_box.extent.x

        # 计算前端到停止线的向量差
        diff = front_location - sl_lo
        # 判断前端是否超过停止线（点积大于等于0表示超过）
        if diff.dot(forward_vector) >= 0:
            cross_time = time.time()
            elapsed_time = cross_time - start_time
            print(f"Ego车辆越过停止线，用时: {elapsed_time:.2f}秒, 交通灯： {tl.get_state()}")
            return True, tl.get_state()
    return False, None

def ego_finish_change_lane(world, ego_vehicle, carla_map, stop_event):
    ego_loc = ego_vehicle.get_location()
    # 检查两者是否在同一车道
    ego_waypoint = carla_map.get_waypoint(ego_loc)
    start_wp = ego_waypoint
    finish = False
    while not stop_event.is_set():
        ego_loc = ego_vehicle.get_location()
        # 检查两者是否在同一车道
        ego_waypoint = carla_map.get_waypoint(ego_loc)
        if ego_waypoint.road_id == start_wp.road_id and ego_waypoint.lane_id != start_wp.lane_id:
            finish = True
            break

    actors = world.get_actors().filter('vehicle.*')
    npc_vehicle = None
    for actor in actors:
        if actor.type_id == 'vehicle.audi.tt':
            npc_vehicle = actor
    npc_loc = npc_vehicle.get_location()
    # 向量的指向是从npc到ego
    delta_x = ego_loc.x - npc_loc.x
    delta_y = ego_loc.y - npc_loc.y

    # 根据车辆的方向计算相对位置
    ego_transform = ego_vehicle.get_transform()
    ego_forward_vector = ego_transform.get_forward_vector()
    dot_product = ego_forward_vector.x * delta_x + ego_forward_vector.y * delta_y
    if dot_product > 0:
        print("ego优先完成行为时位于npc前方")
        direct = 'forward'
    else:
        print("npc优先完成行为时位于npc后方")
        direct = 'backward'
    return finish, direct

def npc_brake(ego_vehicle, world, carla_map, stop_event):
    # Function to apply brakes to NPC vehicle
    # Define a function called brake_npc which takes two parameters, vehicle and brake_value
    def get_distance(actor1, actor2):
        """计算两个actor之间的距离"""
        loc1 = actor1.get_location()
        loc2 = actor2.get_location()
        return math.sqrt((loc1.x - loc2.x) ** 2 + (loc1.y - loc2.y) ** 2 + (loc1.z - loc2.z) ** 2)

    def generate_target_waypoint_list_multilane(waypoint, change='left',
                                                distance_same_lane=10, distance_other_lane=25,
                                                total_lane_change_distance=25, check=True,
                                                lane_changes=1, step_distance=2):
        """
            生成从当前车道到目标车道的路点列表
            """
        plan = []
        plan.append((waypoint, 'follow'))  # 起点位置

        option = 'follow'

        # 在当前车道行驶一段距离
        distance = 0
        while distance < distance_same_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                break
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, option))

        if change == 'left':
            option = 'left'
        elif change == 'right':
            option = 'right'
        else:
            print(f"错误: 方向参数必须是'left'或'right'，但得到了{change}")
            return None, None

        lane_changes_done = 0
        lane_change_distance = total_lane_change_distance / lane_changes

        # 执行车道变更
        while lane_changes_done < lane_changes:
            # 向前移动一定距离
            next_wps = plan[-1][0].next(lane_change_distance)
            if not next_wps:
                break
            next_wp = next_wps[0]

            # 获取目标车道
            if change == 'left':
                if check and str(next_wp.lane_change) not in ['Left', 'Both']:
                    print(f"无法向左变道，车道规则不允许")
                    return None, None
                side_wp = next_wp.get_left_lane()
            else:
                if check and str(next_wp.lane_change) not in ['Right', 'Both']:
                    print(f"无法向右变道，车道规则不允许")
                    return None, None
                side_wp = next_wp.get_right_lane()

            if not side_wp or side_wp.lane_type != carla.LaneType.Driving:
                print(f"目标车道不存在或不可行驶")
                return None, None

            # 更新路径规划
            plan.append((side_wp, option))
            lane_changes_done += 1

        # 在目标车道继续行驶一段距离
        distance = 0
        while distance < distance_other_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                break
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, 'follow'))

        if plan:
            target_lane_id = plan[-1][0].lane_id
            return plan, target_lane_id
        else:
            return None, None

    def npc_change_lane(npc_vehicle, carla_map, world, direction='right'):
        """控制NPC车辆变道"""
        print(f'开始通过路点导航控制NPC车辆{direction}变道')

        try:
            # 获取当前位置和路点
            current_loc = npc_vehicle.get_location()
            current_wp = carla_map.get_waypoint(current_loc)

            # 检查当前车道是否允许变道
            if direction == 'right' and str(current_wp.lane_change) not in ['Right', 'Both']:
                print(f"当前车道不允许向右变道")
                return False
            elif direction == 'left' and str(current_wp.lane_change) not in ['Left', 'Both']:
                print(f"当前车道不允许向左变道")
                return False

            # 生成目标路点列表
            plan, target_lane_id = generate_target_waypoint_list_multilane(
                current_wp,
                change=direction,
                distance_same_lane=0,  # 变道前在当前车道行驶距离
                total_lane_change_distance=4,  # 变道过程距离
                distance_other_lane=8  # 变道后在目标车道行驶距离
            )

            # 如果没有可用的相邻车道，返回失败
            if plan is None:
                print(f"无法{direction}变道，路径生成失败")
                return False

            waypoints = [wp[0] for wp in plan]
            print(f"生成了{len(waypoints)}个路点用于变道")
            terminate_scenario_process()
            # 控制车辆驶向目标路点
            start_time = time.time()
            current_time = 0
            last_location = npc_vehicle.get_location()
            stuck_counter = 0
            while len(waypoints) > 0:
                # 获取当前路点作为目标
                target_wp = waypoints[0]
                target_loc = target_wp.transform.location

                # 计算与目标路点的距离
                current_location = npc_vehicle.get_location()
                distance_to_target = current_location.distance(target_loc)
                current_wp = carla_map.get_waypoint(current_location)
                # 如果距离目标很近，移除该路点
                if distance_to_target < 4.0:
                    waypoints.pop(0)
                    continue

                # 计算转向角度（改进版）
                vehicle_transform = npc_vehicle.get_transform()
                vehicle_location = vehicle_transform.location
                vehicle_forward = vehicle_transform.get_forward_vector()

                # 计算目标方向向量
                target_vector = carla.Vector3D(
                    x=target_loc.x - vehicle_location.x,
                    y=target_loc.y - vehicle_location.y,
                    z=0
                )

                # 计算向量点积，确定车辆是否朝向目标
                dot_product = vehicle_forward.x * target_vector.x + vehicle_forward.y * target_vector.y

                # 计算偏航角差
                target_yaw = math.degrees(math.atan2(target_vector.y, target_vector.x))
                vehicle_yaw = vehicle_transform.rotation.yaw
                yaw_diff = target_yaw - vehicle_yaw

                # 标准化偏航角到[-180, 180]
                while yaw_diff > 180:
                    yaw_diff -= 360
                while yaw_diff < -180:
                    yaw_diff += 360

                # 根据偏航角差计算转向（改进版）
                max_steering = 0.8  # 最大转向角度限制
                steering = yaw_diff / 90.0  # 基本转向计算
                steering = max(min(steering, max_steering), -max_steering)

                # 构建控制指令（改进版）
                control = carla.VehicleControl()
                control.steer = steering
                # 应用控制
                npc_vehicle.apply_control(control)

                # 推进仿真
                if world.get_settings().synchronous_mode:
                    world.tick()
                else:
                    time.sleep(0.05)

                # # 检测是否卡住
                if current_location.distance(last_location) < 0.1:
                    stuck_counter += 1
                    if stuck_counter > 20:  # 如果20帧内移动距离不足0.1米，认为卡住
                        print("车辆卡住，变道失败")
                        break
                else:
                    stuck_counter = 0
                    last_location = current_location

            # 检查是否成功变道
            final_wp = carla_map.get_waypoint(npc_vehicle.get_location())
            if final_wp.lane_id == target_lane_id:
                print(f"路点导航成功{direction}变道，目标车道ID: {target_lane_id}，当前车道ID: {final_wp.lane_id}")
                return True
            else:
                print(f"路点导航变道失败，目标车道ID: {target_lane_id}，当前车道ID: {final_wp.lane_id}")
                return False

        except Exception as e:
            print(f"变道过程中发生异常: {e}")
            return False
        finally:
            print("路点导航变道已完成")
        # npc_vehicle.set_autopilot()
        # 恢复异步模式

        # print('--lane_change_finish---')

    def brake(npc_vehicle, intensity=1.0):
        """控制NPC车辆刹车"""
        # 禁用自动驾驶
        npc_vehicle.set_autopilot(False)
        npc_vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))

        # 应用刹车
        control = carla.VehicleControl()
        control.brake = intensity
        control.throttle = 0
        npc_vehicle.apply_control(control)

    def ego_brake_distance(vehicle_speed):
        # 重力加速度g、平均摩擦系数mu_brake、坡度s
        g = 9.81  # m/s^1
        mu_brake = 0.62
        s = 0
        # 将速度从 km/h 转换为 m/s
        V_mps = vehicle_speed / 3.6

        # 计算制动距离
        l_brake = (V_mps ** 2) / (2 * g * (mu_brake + s))
        return l_brake

    actors = []
    while len(actors) < 2:
        actors = world.get_actors().filter('vehicle.*')
    npc_vehicle = None
    for actor in actors:
        if actor.type_id == 'vehicle.audi.tt':
            npc_vehicle = actor
    lane_changed = False
    prev_speed, acceleration = None, None
    time_interval = 0.1
    while not stop_event.is_set():
        # 获取车辆状态
        npc_transform = npc_vehicle.get_transform()
        ego_transform = ego_vehicle.get_transform()

        # 计算与ego车辆的距离
        distance = get_distance(npc_vehicle, ego_vehicle)

        # 计算相对位置和角度
        npc_location = npc_transform.location
        ego_location = ego_transform.location

        # 计算NPC相对于Ego的位置差
        delta_x = npc_location.x - ego_location.x
        delta_y = npc_location.y - ego_location.y

        # 计算Ego车辆的朝向角度(弧度)
        ego_yaw = math.radians(ego_transform.rotation.yaw)

        # 计算NPC相对于Ego朝向的相对位置
        relative_x = delta_x * math.cos(ego_yaw) + delta_y * math.sin(ego_yaw)
        relative_y = -delta_x * math.sin(ego_yaw) + delta_y * math.cos(ego_yaw)

        # 打印距离信息
        print(f"NPC与ego的距离: {distance:.2f}米")
        print(f"NPC相对Ego的位置: X={relative_x:.2f}米, Y={relative_y:.2f}米")

        ego_speed = ego_vehicle.get_velocity().length() * 3.6
        if prev_speed is None:
            prev_speed = ego_speed
            acceleration = 0
        else:
            acceleration = (ego_speed - prev_speed) / time_interval
            prev_speed = ego_speed
        brake_dis = ego_brake_distance(ego_speed)
        ego_bb = ego_vehicle.bounding_box.extent.x
        npc_bb = npc_vehicle.bounding_box.extent.x
        length = ego_bb + npc_bb
        relative_x = relative_x - length
        # print(f"ego的刹车距离: {brake_dis:.2f}米", length)
        if (relative_x > 0 and brake_dis+2> relative_x > brake_dis and brake_dis > 0 and  # 安全变道距离
                not lane_changed):
            print("NPC开始变道...")
            lane_result = npc_change_lane(npc_vehicle,carla_map, world)
            if lane_result:
                lane_changed = True
            else:
                return False, None
        if lane_changed:
            # 变道完成后刹车
            while True:
                ego_b_tf = ego_vehicle.get_transform()
                ego_b_wp = world.get_map().get_waypoint(ego_b_tf.location)
                npc_b_tf = npc_vehicle.get_transform()
                npc_b_wp = world.get_map().get_waypoint(npc_b_tf.location)
                print(ego_b_wp.lane_id, npc_b_wp.lane_id)
                if ego_b_wp.lane_id == npc_b_wp.lane_id:
                    break
            print("NPC开始刹车...")
            brake(npc_vehicle, 1)
            time.sleep(5)
            return True, acceleration

        time.sleep(time_interval)
    return False, None

def set_ts(world, e_x, e_y, t_x, t_y, stop_event):
    set_ts_main(world, int(e_x), int(e_y), int(t_x), int(t_y), stop_event)

def run_npc(xosc_file_name, port):
    global scenario_process
    # 扩展用户目录路径（处理波浪号）
    scenario_path = os.path.expanduser(xosc_file_name)

    # 检查文件是否存在
    if not os.path.exists(scenario_path):
        print(f"错误：OpenScenario文件未找到 - {scenario_path}")
        return False

    # 构造命令列表
    command = [
        "python",
        "/home/hahabai/code/scenario_runner-0.9.13/scenario_runner.py",
        "--openscenario",
        scenario_path,
        "--trafficManagerPort",
        str(port)
    ]

    try:
        scenario_process = subprocess.Popen(
            command,
            stdout=sys.stdout,  # 将标准输出重定向到当前终端
            stderr=subprocess.STDOUT,  # 合并标准错误到标准输出
            text=True
        )
        # 使用communicate()代替wait()，这样可以通过terminate()终止进程
        scenario_process.communicate()
        if scenario_process.returncode == 0:
            print("\n场景执行成功！")
            return True
        else:
            print(f"\n场景执行失败，错误码：{scenario_process.returncode}")
            return False
    except Exception as e:
        print(f"未预期的错误：{str(e)}")
    return False

def monitor_ego(ego_vehicle, category):
    speed_non_zero = False  # 标记是否存在非零速度
    start_time = time.time()
    while time.time() - start_time < 10:
        # 获取当前速度
        velocity = ego_vehicle.get_velocity()
        speed = 3.6 * (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5

        # 检查速度是否不为零
        if speed > 0.1:  # 添加小阈值以避免微小波动
            speed_non_zero = True
            print(f"检测到非零速度: {speed:.2f} km/h")
        else:
            print(f"检测到速度为零: {speed:.2f} km/h")

        # 等待一小段时间
        time.sleep(0.1)
    if category == 'rear_end':
        # 根据是否存在非零速度返回结果
        return "backward" if speed_non_zero else "forward"
    else:
        # 根据是否存在非零速度返回结果
        return "forward" if speed_non_zero else "backward"

def run_ego(world, carla_map, start_loc, end_loc, scenario_category, stop_event, tf_pos):
    s_x, s_y, s_z = start_loc
    s_lo = carla.Location(x=s_x, y=s_y, z=s_z)
    # s_lo = carla.Location(x=298.344360, y=13.401448, z=1.0)
    s_wp = carla_map.get_waypoint(s_lo)
    ego_vehicle = create_ego(world, s_wp.transform)
    # 设置碰撞传感器
    if ego_vehicle:
        setup_collision_sensor(world, ego_vehicle)
    e_x, e_y, e_z = end_loc
    sent_route(s_x, s_y, s_z, e_x, e_y, e_z)
    time.sleep(1)
    if scenario_category == 'intersection':
        passed, light_state = red_cross_stop_line(world, tf_pos[0], tf_pos[1], ego_vehicle, stop_event)
        return passed, light_state
    elif scenario_category == 'change_lane':
        finish_change, mudir = ego_finish_change_lane(world, ego_vehicle, carla_map, stop_event)
        print("是否ego完成变道",finish_change)
        return finish_change, mudir
    elif scenario_category == 'rear_end' or scenario_category == 'loss_control' or scenario_category == 'environment_factor':
        mudir = monitor_ego(ego_vehicle, scenario_category)
        return True, mudir
        # return True, 'forward'


def setup_collision_sensor(world, ego_vehicle):
    """设置碰撞传感器"""
    global collision_sensor

    # 创建碰撞传感器蓝图
    bp_library = world.get_blueprint_library()
    collision_bp = bp_library.find('sensor.other.collision')

    # 添加传感器到车辆
    collision_sensor = world.spawn_actor(
        collision_bp,
        carla.Transform(),
        attach_to=ego_vehicle
    )

    # 设置碰撞回调函数
    collision_sensor.listen(lambda event: on_collision(event))

def destroy_collision_sensor():
    """安全销毁碰撞传感器"""
    global collision_sensor
    if collision_sensor is not None:
        # 停止监听
        if collision_sensor.is_listening:
            collision_sensor.stop()
        # 销毁传感器
        collision_sensor.destroy()
        collision_sensor = None

def save_crash_scenario():
    # 1. 获取 queue 目录路径
    queue_dir = Path(NEW_FILE).parent  # 即 mutation/queue/

    # 1. 确保目录存在
    if not queue_dir.exists():
        raise FileNotFoundError(f"目录不存在: {queue_dir}")

    # 3. 获取 queue 目录下所有 .xosc 文件（或所有文件，根据需求）
    files = list(queue_dir.glob("*.xosc"))  # 只找 OpenSCENARIO 文件
    # 如果你想包括所有文件，可以用: list(queue_dir.iterdir())

    if not files:
        raise FileNotFoundError(f"在 {queue_dir} 中没有找到任何文件")

    # 4. 找到最新文件（按修改时间排序）
    latest_file = max(files, key=lambda f: f.stat().st_mtime)

    print(f"最新文件: {latest_file.name} (修改时间: {latest_file.stat().st_mtime})")

    # 5. 定义目标目录：mutation/crash/
    mutation_dir = queue_dir.parent  # 即 mutation/ 目录
    crash_dir = mutation_dir / "crash"

    # 创建 crash 目录（如果不存在）
    crash_dir.mkdir(exist_ok=True)

    # 6. 目标路径
    destination = crash_dir / latest_file.name

    # 7. 复制文件（保留原文件）
    shutil.copy2(latest_file, destination)  # copy2 会保留元数据（如修改时间）

    print(f"已保存crash场景到: {destination}")
def on_collision(event):
    """碰撞回调函数"""
    global collision_event

    # 打印碰撞信息
    print(f"发生碰撞！强度: {event.normal_impulse.length()}")

    # 设置碰撞事件标志
    collision_event.set()

    # 在终止场景前，先销毁碰撞传感器
    destroy_collision_sensor()

    # 终止场景进程
    terminate_scenario_process()

    # 保存crash场景
    save_crash_scenario()

def kill_traffic_signal_process():
    """终止所有 set_traffic_signal.py 进程"""
    try:
        # 使用 -f 参数匹配完整命令行
        subprocess.run(
            ["pkill", "-f", "set_traffic_signal.py"],
            check=True,
            stdout=subprocess.DEVNULL,  # 屏蔽输出
            stderr=subprocess.DEVNULL
        )
        print("已终止 set_traffic_signal.py 进程")
    except subprocess.CalledProcessError:
        print("无 set_traffic_signal.py 进程需要终止")

def terminate_scenario_process():
    """终止场景子进程"""
    global scenario_process
    if scenario_process is not None and scenario_process.poll() is None:
        print("正在终止场景进程...")
        scenario_process.terminate()
        try:
            # 等待进程终止，设置超时时间
            scenario_process.wait(timeout=1)
            print("场景进程已成功终止")
        except subprocess.TimeoutExpired:
            # 如果超时，强制杀死进程
            print("场景进程未响应，正在强制终止...")
            scenario_process.kill()
            scenario_process.wait()
            print("场景进程已被强制终止")
    scenario_process = None

def cleanup_simulation(client, world):
    """清理仿真资源"""
    global collision_sensor

    # 销毁碰撞传感器
    if collision_sensor is not None:
        collision_sensor.destroy()
        collision_sensor = None

    # 重置仿真环境
    reset_simulation(world)

    # 关闭Carla客户端连接
    client.apply_batch([carla.command.DestroyActor(x) for x in world.get_actors()])
    client = None

def monitor_ttc(world, stop_event, category, min_ttc):
    min_ttc_value = float('inf')
    while not stop_event.is_set():
        try:
            ttc = fd.calculate_ttc(world, category)
            if ttc < min_ttc_value:
                min_ttc_value = ttc
        except Exception as e:
            print(f"计算TTC时出错: {e}")
        time.sleep(0.1)
    min_ttc[0] = min_ttc_value

def run_scenario(ego_s, ego_e, scenario_category, file_name, ego_tl, npc_tl, cross_point, npc_max_speed):
    global should_exit, scenario_process, traffic_manager_port
    # 初始化Carla连接
    client, world, carla_map = initialize_simulation()
    scene = scenario.Scene()
    scene.ego_s, scene.ego_e, scene.category, scene.file_name, scene.max_speed = ego_s, ego_e, scenario_category, file_name, npc_max_speed
    # intersection_accident

    # change_line
    # ego_s = (383.1048889160156, -18.292522430419922, 0.0)
    # carla_location = carla.Location(x=ego_s[0], y=ego_s[1], z=ego_s[1])
    # ego_sta_wp = carla_map.get_waypoint(carla_location, project_to_road=True)
    # ego_end_wp = carla_map.get_waypoint(carla_location, project_to_road=True).next(100)[0].get_left_lane()
    # ego_e = (ego_end_wp.transform.location.x, ego_end_wp.transform.location.y, ego_end_wp.transform.location.z)
    # scenario_category = 'change_lane'
    # scenario_file_name = "/home/hahabai/PycharmProjects/ego_crime/llm_generate_openscenario/Change_Lane_Conflicts.xosc"

    # read_end_collision
    # ego_s = (308.22137451171875, 13.57004451751709, 1.2320460081100464)
    # carla_location = carla.Location(x=ego_s[0], y=ego_s[1], z=ego_s[1])
    # ego_sta_wp = carla_map.get_waypoint(carla_location, project_to_road=True)
    # ego_end_wp = ego_sta_wp.next(100)[0]
    # ego_e = (ego_end_wp.transform.location.x, ego_end_wp.transform.location.y, ego_end_wp.transform.location.z)
    # scenario_category = 'rear_end'
    # scenario_file_name = "/home/hahabai/PycharmProjects/ego_crime/llm_generate_openscenario/Rear_End_Conflicts_changeLane_brake.xosc"

    cycle_count = 0
    sce_min_ttc = float('inf')
    start_time = time.time()
    duration = 5 * 60  # 5分钟
    should_stop = False  # 控制外层循环的标志
    # 外层循环继续前检查标志
    while cycle_count < Max_Cycle_times:
        if should_stop:
            break  # 跳出外层循环
        cycle_count += 1
        next_scene = None
        mut_count = 0
        try:
            while mut_count < Max_Mutation_times:
                # 检查是否超时
                if time.time() - start_time > duration:
                    print("已运行%d秒，停止所有循环。" % duration)
                    should_stop = True  # 设置标志
                    break  # 跳出内层循环
                mut_count += 1
                skip_this_mutation = False
                print(f"\n===== 开始第 {cycle_count} 轮循环, 第 {mut_count} 轮变异 =====")

                # 重置碰撞事件标志
                collision_event.clear()

                # 重置仿真环境
                reset_simulation(world)

                mutate_scene = mutate(carla_map, scene, cycle_count, mut_count)
                ego_s = mutate_scene.ego_s
                carla_location = carla.Location(x=ego_s[0], y=ego_s[1], z=ego_s[2])
                ego_sta_wp = carla_map.get_waypoint(carla_location, project_to_road=True)
                ego_e = mutate_scene.ego_e
                scenario_category = mutate_scene.category
                scenario_file_name = mutate_scene.file_name
                with ThreadPoolExecutor(max_workers=4) as executor:
                    # 提交任务到线程池，使用不同的交通管理器端口
                    future1 = executor.submit(run_npc, scenario_file_name, traffic_manager_port)

                    time.sleep(2) #rear_end 4s，other 2s
                    # 在 main() 函数中创建 stop_event
                    stop_event = threading.Event()

                    # 提交 set_ts 作为第三个线程
                    if scenario_category == 'intersection':
                        # 提交 run_ego 作为第二个线程
                        future2 = executor.submit(run_ego, world, carla_map, ego_s, ego_e, scenario_category,
                                                  stop_event, [ego_tl[0], ego_tl[1]])
                        future3 = executor.submit(set_ts, world, ego_tl[0], ego_tl[1], npc_tl[0], npc_tl[1], stop_event)

                        # 定义目标点
                        target_location = carla.Location(x=cross_point[0], y=cross_point[1], z=0)
                        mut_dir_stop_event = threading.Event()
                        future4 = executor.submit(mutate_direction, carla_map, True, world, target_location,
                                                  collision_event, mut_dir_stop_event)

                    else:
                        # 提交 run_ego 作为第二个线程
                        future2 = executor.submit(run_ego, world, carla_map, ego_s, ego_e, scenario_category,
                                                  stop_event, None)
                        future3 = None

                    # 启动TTC监控线程（使用独立的stop_event）
                    min_ttc = [float('inf')]
                    ttc_stop_event = threading.Event()
                    ttc_monitor_thread = threading.Thread(
                        target=monitor_ttc,
                        args=(world, ttc_stop_event, scenario_category, min_ttc)
                    )
                    ttc_monitor_thread.daemon = True  # 设置为守护线程，主线程结束时自动终止
                    ttc_monitor_thread.start()

                    done, not_done = wait([future1, future2], return_when=FIRST_COMPLETED)
                    # 检查完成的任务
                    for future in done:
                        print("进入for循环")
                        if future == future1:  # 场景任务完成
                            stop_event.set()
                            print("场景执行完成，进入下一轮循环")
                            # 取消未完成的任务
                            # if not future2.done():
                            # future2.cancel()
                            if scenario_category == 'intersection':
                                mut_dir_stop_event.set()
                                mudir = future4.result()
                                future4.cancel()
                                future2.cancel()
                                # if passed:
                                #     # 终止场景子进程
                                #     print("NPC完成刹车，终止场景进程", acceleration)
                                #     mudir = mutate_direction(ego_sta_wp, carla_map,
                                #                                scenario_file_name,
                                #                                False, None, acceleration)
                            else:
                                # mudir = random.choice(['forward', 'backward'])
                                passed, mudir = future2.result()
                            # 等待TTC监控线程结束
                            ttc_stop_event.set()
                            ttc_monitor_thread.join()
                            print(f"场景运行过程中最小的TTC: {min_ttc[0]}")
                            break
                        elif future == future2:  # Ego任务完成
                            passed, f2_result = future.result()
                            if scenario_category == 'intersection':
                                stop_event.set()
                                light_state = f2_result
                                print(passed, light_state)
                                if passed and light_state != carla.TrafficLightState.Red:
                                    # 属于无效场景，不需要变化npc的起始点
                                    print("Ego车辆通过停止线时，红绿灯不是红色，进入下一轮循环")
                                    # 终止场景子进程
                                    terminate_scenario_process()
                                    # 取消未完成的场景任务
                                    # if not future1.done():
                                    mut_dir_stop_event.set()
                                    mudir = future4.result()
                                    future4.cancel()
                                    future1.cancel()
                                    print("场景任务已取消")
                                    # 将mut_count减1，以便重新尝试该轮变异
                                    mut_count -= 1
                                    # 清理资源
                                    ttc_stop_event.set()
                                    ttc_monitor_thread.join()
                                    # 设置标志并跳出循环
                                    skip_this_mutation = True
                                    break
                                elif passed and light_state == carla.TrafficLightState.Red:
                                    print("Ego车辆通过停止线时，红绿灯为红色")
                                    # 直接等待 future1 完成（无超时）
                                    if future1 in not_done:
                                        print("等待NPC场景完成")
                                        future1_result = future1.result(timeout=10)  # 无超时等待
                                        print("场景任务完成")
                                    else:
                                        print("NPC场景已完成")
                                    ttc_stop_event.set()
                                    ttc_monitor_thread.join()
                                    mut_dir_stop_event.set()
                                    mudir = future4.result()
                                    print(f"场景运行过程中最小的TTC: {min_ttc[0]}")
                                    break
                            elif scenario_category == 'change_lane':
                                if passed:
                                    # 终止场景子进程
                                    print("Ego完成变道，终止场景进程")
                                    mudir = f2_result                                    # 等待TTC监控线程结束
                                    ttc_stop_event.set()
                                    ttc_monitor_thread.join()
                                    print(f"场景运行过程中最小的TTC: {min_ttc[0]}")
                                    terminate_scenario_process()
                                break
                            else:
                                mudir = f2_result
                                # 等待TTC监控线程结束
                                ttc_stop_event.set()
                                ttc_monitor_thread.join()
                                print(f"场景运行过程中最小的TTC: {min_ttc[0]}")
                                terminate_scenario_process()
                                break
                    if scenario_category == 'intersection':
                        future3.cancel()
                    destroy_collision_sensor()
                    # 如果需要跳过当前变异，直接进入下一次循环
                if skip_this_mutation:
                    print(f"跳过第 {mut_count + 1} 轮变异的后续处理")
                    continue
                else:
                    print(f"变异场景的: {min_ttc[0]}; 上一场景的ttc：{sce_min_ttc}")
                    if sce_min_ttc >= min_ttc[0]:
                        sce_min_ttc = min_ttc[0]
                        mutate_scene.direct = mudir
                        next_scene = mutate_scene
                print(f"=====  结束{cycle_count} 轮循环, 第 {mut_count} 轮变异 =====")
                time.sleep(1)  # 循环间短暂等待
                # 增加交通管理器端口
                traffic_manager_port += 1
            if next_scene is not None:
                scene = next_scene

        except KeyboardInterrupt:
            print("\n程序被用户中断")
    # 清理资源
    should_exit = True
    terminate_scenario_process()
    cleanup_simulation(client, world)
    print("程序已安全退出")

def parse_file(original_file_path, new_file_path, category):
    try:
        extracted_data = extract_data(original_file_path, category)
        # 格式化输出结果
        print("=" * 50)
        print("数据提取结果")
        print("=" * 50)
        print("\n1. EgoRoute（自我车辆路径组）:")
        print(f"   - Route坐标点（2个）: {extracted_data['EgoRoute']['Route_Coordinates']}")
        print(f"   - 交通信号灯坐标: {extracted_data['EgoRoute']['TrafficSignal_Coordinate']}")

        print("\n1. NPCRoute（NPC车辆路径组）:")
        print(f"   - 交通信号灯坐标: {extracted_data['NPCRoute']['TrafficSignal_Coordinate']}")
        print("=" * 50)
    except FileNotFoundError:
        print(f"错误：文件未找到，请检查路径是否正确！路径：{original_file_path}")
    except ET.ParseError:
        print("错误：XML文件解析失败，可能文件格式损坏或非标准XML")
    cross_point = None
    if category == 'intersection':
        # 提取两个列表
        ego_coords = extracted_data["EgoRoute"]["Route_Coordinates"]
        npc_coords = extracted_data["NPCRoute"]["Route_Coordinates"]

        # 使用集合推导式，提取 (int(x), int(y)) 作为键来匹配
        ego_set = {(int(x), int(y)) for x, y, z in ego_coords}
        npc_set = {(int(x), int(y)) for x, y, z in npc_coords}
        # 找出交集，即相同的 (x, y) 坐标点（整型）
        common_xy = ego_set & npc_set
        cross_point = next(iter(common_xy))
    modify_xml(original_path=original_file_path, new_path=new_file_path)
    return extracted_data['EgoRoute']['Route_Coordinates'], extracted_data['EgoRoute']['TrafficSignal_Coordinate'], extracted_data['NPCRoute']['TrafficSignal_Coordinate'], extracted_data['NPCRoute']['MaxSpeed'], cross_point
if __name__ == "__main__":
    category = ORIGINAL_FILE.split('/')[-1].split('.xosc')[0]
    ego_pos, ego_tl, npc_tl, npc_max_speed, cross_point = parse_file(ORIGINAL_FILE, NEW_FILE, category)
    run_scenario(ego_pos[0], ego_pos[-1], category, NEW_FILE, ego_tl, npc_tl, cross_point, npc_max_speed)