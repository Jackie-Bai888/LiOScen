import xml.etree.ElementTree as ET
import carla
import time
import carla
import math
import random
import scenario
import copy
import json
import os

# 创建Carla位置对象列表
# npc_juc_wps_loc = [
#     carla.Location(x=154.035950, y=36.197544, z=0.000000),
#     carla.Location(x=154.0347442626953, y=37.19754409790039, z=0.0),
#     carla.Location(x=154.0335235595703, y=38.197540283203125, z=0.0),
#     carla.Location(x=154.03231811523438, y=39.197540283203125, z=0.0),
#     carla.Location(x=154.03109741210938, y=40.197540283203125, z=0.0),
#     carla.Location(x=154.02987670898438, y=41.197540283203125, z=0.0),
#     carla.Location(x=154.02867126464844, y=42.197540283203125, z=0.0),
#     carla.Location(x=154.02745056152344, y=43.197540283203125, z=0.0),
#     carla.Location(x=154.02622985839844, y=44.197540283203125, z=0.0),
#     carla.Location(x=154.0250244140625, y=45.19753646850586, z=0.0),
#     carla.Location(x=154.0225830078125, y=47.19753646850586, z=0.0),
#     carla.Location(x=154.02137756347656, y=48.19753646850586, z=0.0),
#     carla.Location(x=154.0251007080078, y=49.260074615478516, z=0.0),
#     carla.Location(x=154.13739013671875, y=50.49626922607422, z=0.0),
#     carla.Location(x=154.3981170654297, y=51.709869384765625, z=0.0),
#     carla.Location(x=154.80343627929688, y=52.883113861083984, z=0.0),
#     carla.Location(x=155.34742736816406, y=53.99884796142578, z=0.0),
#     carla.Location(x=156.02215576171875, y=55.040740966796875, z=0.0),
#     carla.Location(x=156.81771850585938, y=55.993560791015625, z=0.0),
#     carla.Location(x=157.71307373046875, y=56.837005615234375, z=0.0),
#     carla.Location(x=158.68299865722656, y=57.56748962402344, z=0.0),
#     carla.Location(x=159.72564697265625, y=58.189788818359375, z=0.0),
#     carla.Location(x=160.82901000976562, y=58.69672393798828, z=0.0),
#     carla.Location(x=161.98033142089844, y=59.082454681396484, z=0.0),
#     carla.Location(x=163.16639709472656, y=59.342533111572266, z=0.0),
#     carla.Location(x=164.37350463867188, y=59.473960876464844, z=0.0),
#     carla.Location(x=165.48294067382812, y=59.49085235595703, z=0.0),
#     carla.Location(x=166.48294067382812, y=59.49073028564453, z=0.0),
#     carla.Location(x=167.48329162597656, y=59.49060821533203, z=0.0),
#     carla.Location(x=168.48329162597656, y=59.49048614501953, z=0.0),
#     carla.Location(x=169.48329162597656, y=59.49036407470703, z=0.0),
#     carla.Location(x=170.48329162597656, y=59.49024200439453, z=0.0),
#     carla.Location(x=171.48329162597656, y=59.49011993408203, z=0.0),
#     carla.Location(x=172.48329162597656, y=59.48999786376953, z=0.0),
#     carla.Location(x=173.48329162597656, y=59.48987579345703, z=0.0),
#     carla.Location(x=174.48329162597656, y=59.48975372314453, z=0.0),
#     carla.Location(x=175.48329162597656, y=59.48963165283203, z=0.0),
#     carla.Location(x=176.48329162597656, y=59.4895133972168, z=0.0),
#     carla.Location(x=177.1702423095703, y=59.48942947387695, z=0.0)
# ]
def connect_carla():
    """连接到CARLA服务器并加载地图"""
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        return world.get_map()
    except RuntimeError as e:
        print(f"连接CARLA失败: {e}")
        raise


def get_new_waypoint(carla_map, original_location, direct, distance=10.0):
    """获取前或后指定距离的路径点"""
    # 创建CARLA坐标变换
    location = carla.Location(x=original_location['x'],
                       y=original_location['y'],  # CARLA使用左手坐标系Y轴反向
                       z=original_location['z'])
    # 获取最近的路径点
    wp = carla_map.get_waypoint(location)
    if direct == 'forward':
        new_wp = wp.next(distance)[0]
    else:
        new_wp = wp.previous(distance)[0]
    return new_wp.transform

def normalize_angle(angle_degrees):
    """将角度规范到(-180, 180]范围"""
    angle_degrees = angle_degrees % 360
    if angle_degrees > 180:
        angle_degrees = angle_degrees - 360
    if angle_degrees < -180:
        angle_degrees = angle_degrees + 360
    return angle_degrees


def carla_to_openscenario_rotation(carla_rotation):
    """
    处理CARLA到OpenSCENARIO的旋转转换：
    1. 角度规范化到(-180,180]
    1. 反转yaw/h的正负关系
    3. 转换为弧度制
    """
    return {
        "h": math.radians(normalize_angle(carla_rotation.yaw)),  # 反转yaw并转换
        "p": math.radians(normalize_angle(carla_rotation.pitch)),  # 保持pitch方向
        "r": math.radians(normalize_angle(carla_rotation.roll))  # 保持roll方向
    }

def distance_between_locations(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x) ** 2 + (loc1.y - loc2.y) ** 2 + (loc1.z - loc2.z) ** 2)


def find_closest_location_index(npc_wp_loc, npc_location):
    closest_distance = float('inf')
    closest_index = -1

    for i, location in enumerate(npc_wp_loc):
        dist = distance_between_locations(location, npc_location)
        if dist < closest_distance:
            closest_distance = dist
            closest_index = i

    return closest_index


def compare_indices(collision_index, npc_index):
    if collision_index == -1 or npc_index == -1:
        return "Reference or target index not found"

    if npc_index < collision_index:
        return "before"
    elif npc_index > collision_index:
        return "after"
    else:
        return "at the same position"

def is_ego_prior_pass_colli_point(world, colli_point_locat, collision_event, mut_dir_stop_event):
    # 判断和npc车辆比是否ego车辆优先通过碰撞点
    # 获取所有车辆actor
    actors = []
    while len(actors) < 2:
        actors = world.get_actors().filter('vehicle.*')
    ego_vehicle = None
    npc_vehicle = None

    for actor in actors:
        if actor.type_id == 'vehicle.lincoln.mkz_2017':
            ego_vehicle = actor
        elif actor.type_id == 'vehicle.audi.tt':
            npc_vehicle = actor
    # 找到最近的waypoint到目标点
    npc_vehicle_location = npc_vehicle.get_location()
    # reference_index = find_closest_location_index(npc_juc_wps_loc, colli_point_locat)
    # closest_index = find_closest_location_index(npc_juc_wps_loc, npc_vehicle_location)
    # position_relation = compare_indices(reference_index, closest_index)
    #
    # if position_relation == 'after':
    #     return False
    # else:
    while not collision_event.is_set() and not mut_dir_stop_event.is_set():
        ego_loc = ego_vehicle.get_location()
        npc_loc = npc_vehicle.get_location()
        ego_col_dis = distance_between_locations(ego_loc, colli_point_locat)
        npc_col_dis = distance_between_locations(npc_loc, colli_point_locat)
        print(ego_col_dis, npc_col_dis)
        if ego_col_dis < npc_col_dis and ego_col_dis < 2:
            return True
        elif npc_col_dis < ego_col_dis and npc_col_dis < 2:
            return False
        # # 等待一段时间后再次检查位置
        time.sleep(0.1)

def finish_behav_ego_with_npc(world, carla_map):
    # 判断ego或npc完成行为时，ego是否位于npc前方
    # 获取所有车辆actor
    actors = []
    while len(actors) <2:
        actors = world.get_actors().filter('vehicle.*')
    ego_vehicle = None
    npc_vehicle = None

    for actor in actors:
        if actor.type_id == 'vehicle.lincoln.mkz_2017':
            ego_vehicle = actor
        elif actor.type_id == 'vehicle.audi.tt':
            npc_vehicle = actor
    ego_loc = ego_vehicle.get_location()
    npc_loc = npc_vehicle.get_location()
    # 向量的指向是从npc到ego
    delta_x = ego_loc.x - npc_loc.x
    delta_y = ego_loc.y - npc_loc.y

    # 根据车辆的方向计算相对位置
    ego_transform = ego_vehicle.get_transform()
    ego_forward_vector = ego_transform.get_forward_vector()
    dot_product = ego_forward_vector.x * delta_x + ego_forward_vector.y * delta_y
    return dot_product > 0

def filter_curvature_paths(data, current_curvature, direct):
    """根据方向和当前曲率筛选路径"""
    if direct == 'forward':
        candidates = [p for p in data if p['curvature'] > current_curvature]
        # return min(candidates, key=lambda x: x['curvature'], default=None)
        return random.choice(candidates) if candidates else None
    elif direct == 'backward':
        candidates = [p for p in data if p['curvature'] < current_curvature]
        # return max(candidates, key=lambda x: x['curvature'], default=None)
        return random.choice(candidates) if candidates else None
    return None


def update_change_range(scene, direct, change_amount=1.0):
    """更新场景的变化范围"""
    if direct == 'forward':
        scene.forward_change_range = max(0, scene.forward_change_range - change_amount)
    else:
        scene.backward_change_range = max(0, scene.backward_change_range - change_amount)
    return scene

def update_openscenario_with_coordinates(input_file, output_file, coordinates):
    """
    依据提供的坐标更新OpenSCENARIO文件

    参数:
    input_file (str): 输入的OpenSCENARIO文件路径
    output_file (str): 输出的OpenSCENARIO文件路径
    coordinates (list): 行人坐标列表，每个坐标是一个字典，包含x、y、z和h值
    """
    # 解析XML文件
    tree = ET.parse(input_file)
    root = tree.getroot()

    # 清除现有的Entities和相关动作
    entities = root.find('Entities')
    if entities is not None:
        entities.clear()

    storyboard = root.find('Storyboard')
    if storyboard is not None:
        init = storyboard.find('Init')
        if init is not None:
            actions = init.find('Actions')
            if actions is not None:
                # 保留GlobalAction，清除Private动作
                global_action = actions.find('GlobalAction')
                actions.clear()
                if global_action is not None:
                    actions.append(global_action)

        # 更新Story中的Actors
        story = storyboard.find('Story')
        if story is not None:
            act = story.find('Act')
            if act is not None:
                maneuver_group = act.find('ManeuverGroup')
                if maneuver_group is not None:
                    actors = maneuver_group.find('Actors')
                    if actors is None:
                        # 创建Actors元素并添加selectTriggeringEntities属性
                        actors = ET.SubElement(maneuver_group, 'Actors', {'selectTriggeringEntities': 'true'})
                    else:
                        actors.clear()
                        # 确保现有Actors元素有selectTriggeringEntities属性
                        if 'selectTriggeringEntities' not in actors.attrib:
                            actors.set('selectTriggeringEntities', 'true')

    # 添加新的Entities
    if entities is None:
        entities = ET.SubElement(root, 'Entities')

    for i, coord in enumerate(coordinates, 1):
        name = f"adversary{i}"
        scenario_object = ET.SubElement(entities, 'ScenarioObject', {'name': name})
        pedestrian = ET.SubElement(scenario_object, 'Pedestrian', {
            'model': 'walker.pedestrian.0001',
            'mass': '90.0',
            'name': 'walker.pedestrian.0001',
            'pedestrianCategory': 'pedestrian'
        })
        param_decl = ET.SubElement(pedestrian, 'ParameterDeclarations')
        bounding_box = ET.SubElement(pedestrian, 'BoundingBox')
        center = ET.SubElement(bounding_box, 'Center', {'x': '1.5', 'y': '0.0', 'z': '0.9'})
        dimensions = ET.SubElement(bounding_box, 'Dimensions', {'width': '1.1', 'length': '4.5', 'height': '1.8'})
        properties = ET.SubElement(pedestrian, 'Properties')
        prop = ET.SubElement(properties, 'Property', {'name': 'type', 'value': 'simulation'})

    # 添加新的Private动作
    if storyboard is not None and init is not None and actions is not None:
        for i, coord in enumerate(coordinates, 1):
            name = f"adversary{i}"
            private = ET.SubElement(actions, 'Private', {'entityRef': name})
            private_action = ET.SubElement(private, 'PrivateAction')
            teleport_action = ET.SubElement(private_action, 'TeleportAction')
            position = ET.SubElement(teleport_action, 'Position')
            world_position = ET.SubElement(position, 'WorldPosition', {
                'x': str(round(coord['location']['x'], 8)),
                'y': str(round(coord['location']['y'], 8)),
                'z': str(round(coord['location']['z'], 8)),
                'h': str(round(coord['orientation']['h'], 8)),
                'p': str(round(coord['orientation']['p'], 8)),
                'r': str(round(coord['orientation']['r'], 8))
            })

    # 更新Actors
    if storyboard is not None and story is not None and act is not None and maneuver_group is not None and actors is not None:
        for i in range(1, len(coordinates) + 1):
            name = f"adversary{i}"
            entity_ref = ET.SubElement(actors, 'EntityRef', {'entityRef': name})

    # 保存更新后的XML文件
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"已成功生成新的OpenSCENARIO文件: {output_file}")


def extract_coordinates(json_file, target_curvature):
    """
    从JSON文件中提取指定curvature的坐标数据并转换格式

    参数:
    json_file (str): JSON文件路径
    target_curvature (float): 目标曲率值

    返回:
    list: 转换后的坐标列表
    """
    # 检查文件是否存在
    if not os.path.exists(json_file):
        raise FileNotFoundError(f"文件 {json_file} 不存在")

    # 读取JSON文件
    with open(json_file, 'r') as f:
        data = json.load(f)

    # 查找匹配curvature的字典
    matched_entry = None
    for entry in data:
        if entry.get('curvature') == target_curvature:
            matched_entry = entry
            break

    if not matched_entry:
        raise ValueError(f"未找到曲率为 {target_curvature} 的数据")

    # 提取并转换坐标
    coordinates = []
    for shoulder_point in matched_entry.get('shoulder', []):
        location = shoulder_point.get('location', {})
        orientation = shoulder_point.get('orientation', {})

        # 转换坐标格式，z值设为1
        coord = {
            'x': location.get('x', 0),
            'y': location.get('y', 0),
            'z': 1,  # 固定为1
            'h': orientation.get('h', 0)
        }
        coordinates.append(coord)

    return coordinates
def mutate_direction(carla_map, is_need_judge_cl, world, colli_point_locat, collision_event, mut_dir_stop_event):
    # 是否需要判断是否ego优先通过碰撞点或完成行为时位于npc前方
    if is_need_judge_cl:
        if colli_point_locat:
            is_ego_prior = is_ego_prior_pass_colli_point(world, colli_point_locat, collision_event, mut_dir_stop_event)
        else:
            is_ego_prior = finish_behav_ego_with_npc(world, carla_map)
        if is_ego_prior:
            print("ego优先到达碰撞点或完成行为时位于npc前方")
            direct = 'forward'
        else:
            print("npc优先到达碰撞点或完成行为时位于npc后方")
            direct = 'backward'
    else:
        if colli_point_locat:
            if colli_point_locat > 0:
                # 前后碰撞场景，npc变道时，如果ego加速，则将npc往前放或者不变
                direct = 'forward'
            elif colli_point_locat < 0:
                # 前后碰撞场景，npc变道时，如果ego减速，则将npc往后拉或者不变
                direct = 'backward'
            else:
                direct = random.choice(['forward', 'backward'])

        else:
            direct = 'backward'
    return direct

def add_waypoint_to_route(carla_map, ori_loc, distance):
    backward_waypoints = []  # 存储向后采样的 waypoint 坐标
    accumulated_distance = 0.0
    step = 2.0  # 每 1 米采样一个点
    current_loc = carla.Location(
        x=ori_loc['x'],
        y=ori_loc['y'],
        z=ori_loc['z']
    )
    current_wp = carla_map.get_waypoint(current_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    while accumulated_distance < distance:
        # 获取当前 location 对应的 road waypoint
        if current_wp is None:
            print("无法获取当前 waypoint，中断向后采样")
            break

        # 获取前一个 waypoint（沿道路反向）
        prev_wps = current_wp.previous(step)
        if not prev_wps:
            print("无更多前向（backward）路径点")
            break

        # 通常 previous() 返回一个列表，取第一个
        prev_wp = prev_wps[0]
        current_wp = prev_wp
        prev_loc = prev_wp.transform.location

        # 计算移动距离
        segment_distance = current_loc.distance(prev_loc)
        if segment_distance < 0.1:  # 防止卡住
            break

        backward_waypoints.append(prev_wp)
        accumulated_distance += segment_distance
    return backward_waypoints




def mutate(carla_map, scene, cycle_count, mut_count):
    (direct, min_speed, max_speed, forward_change_range, backward_change_range, category) \
        = (scene.direct, scene.min_speed,
           scene.max_speed, scene.forward_change_range, scene.backward_change_range, scene.category)
    mutate_scene = copy.deepcopy(scene)
    input_file = scene.file_name
    # 处理loss_control类型场景（JSON文件）
    if category == 'loss_control':
        try:
            # 读取曲率数据
            curve_coordinates_file = "/home/hahabai/PycharmProjects/ego_crime/simulation/carla_town04_curves_data.json"
            with open(curve_coordinates_file, 'r') as f:
                curve_data = json.load(f)

            # 直接使用场景中的当前曲率
            current_curvature = scene.curvature

            # 筛选符合条件的路径
            new_path = filter_curvature_paths(curve_data, current_curvature, direct)
            if not new_path:
                print(f"未找到曲率{'更大' if direct == 'forward' else '更小'}的路径")
                return scene

            # 随机选择新路径并更新变量
            mutate_scene.ego_s = (new_path['start']['x'], new_path['start']['y'], new_path['start']['z'])
            mutate_scene.ego_e = (new_path['end']['x'], new_path['end']['y'], new_path['end']['z'])
            mutate_scene.curvature = new_path['curvature']  # 新增：更新曲率变量

            # # 更新变化范围并保存文件
            # update_change_range(mutate_scene, direct)
            # 执行更新
            output_file = input_file.replace('.xosc', f'_mutation_{cycle_count}_{mut_count}.xosc')
            update_openscenario_with_coordinates(input_file, output_file, new_path["shoulder"])
            print(f"成功更新：曲率从{current_curvature:.4f}变为{mutate_scene.curvature:.4f}")
            mutate_scene.file_name = output_file
            return mutate_scene

        except Exception as e:
            print(f"处理loss_control场景时出错: {e}")
            return scene
    elif category == "environment_factor":
        # 解析XML文件
        tree = ET.parse(input_file)
        root = tree.getroot()

        # 查找frictionScaleFactor元素
        friction_element = root.find('.//RoadCondition[@frictionScaleFactor]')
        # 修改摩擦系数值
        # 获取旧的摩擦系数值
        old_value = float(friction_element.get('frictionScaleFactor'))
        print(f"旧的摩擦系数值为 {old_value}")
        if direct == 'forward':
            # 减少摩擦系数
            t = random.uniform(-0.1, -0.2)
        else:
            # 增加摩擦系数
            t = random.uniform(0.1, 0.2)

        # 计算新值
        new_value = old_value + t

        # 确保新值不超过1
        new_value = max(0.1, min(new_value, 1.0))
        print(f"新的摩擦系数值为 {new_value}")

        # 修改摩擦系数值
        friction_element.set('frictionScaleFactor', str(new_value))
        output_file = input_file.replace('.xosc', f'_mutation_{cycle_count}_{mut_count}.xosc')
        tree.write(output_file, encoding='utf-8', xml_declaration=True)

        mutate_scene.file_name = output_file
        return mutate_scene
    else:

        # 解析XML文件
        tree = ET.parse(input_file)
        root = tree.getroot()

        # 获取npc车辆的初始位置
        init_xpath = './/Init/Actions/Private[@entityRef="npc"]/PrivateAction/TeleportAction/Position/WorldPosition'
        init_wp = root.find(init_xpath)
        if init_wp is None:
            print("未找到初始位置节点")
            return scene

        # 从XML读取原始坐标
        original_location = {
            'x': float(init_wp.get('x')),
            'y': float(init_wp.get('y')),
            'z': float(init_wp.get('z'))
        }

        # 随机选择改变速度还是位置
        if category != 'rear_end':
            change_type = random.choice(['speed', 'location'])
        else:
            change_type = 'location'

        if change_type == 'speed':
            # 获取速度元素
            speed_elements = root.findall(".//AbsoluteTargetSpeed")
            if not speed_elements:
                print("未找到速度元素")
                return scene
            element = speed_elements[0]

            current_speed = float(element.get('value', '0'))

            # 根据direct决定是增加还是减少速度
            if direct == 'forward':
                # 增加速度，但不超过最大值
                speed_change = random.uniform(1.0, 5.0)  # 随机增加1-5 m/s
                new_speed = min(current_speed + speed_change, max_speed)
            else:  # backward
                # 减少速度，但不低于最小值
                speed_change = random.uniform(1, 5)  # 随机减少1-5 m/s
                new_speed = max(current_speed - speed_change, min_speed)

            # 更新速度值
            element.set('value', f"{new_speed:.2f}")
            print(f"成功更新速度从 {current_speed:.2f} 到 {new_speed:.2f}")

        else:  # change_type == 'location'
            if category == 'rear_end':
                # 后向碰撞场景特殊处理
                if direct == 'forward':
                    # 在前向范围内随机生成距离
                    if forward_change_range <= 0:
                        print("前向变化范围不足")
                        return scene
                    distance = random.uniform(2, forward_change_range)  # 随机距离范围0.1到forward_change_range米
                    actual_change = distance
                else:  # backward
                    # 在后向范围内随机生成距离
                    if backward_change_range <= 0:
                        print("后向变化范围不足")
                        return scene
                    distance = random.uniform(2, backward_change_range)  # 随机距离范围0.1到backward_change_range米
                    actual_change = distance

                # 获取新的路径点
                new_transform = get_new_waypoint(
                    carla_map, original_location, direct, distance)

                if new_transform:
                    # 转换坐标系：CARLA Y轴反向
                    new_location = new_transform.location

                    # 更新XML节点
                    init_wp.set('x', f"{new_location.x:.6f}")
                    init_wp.set('y', f"{new_location.y:.6f}")
                    init_wp.set('z', f"{new_location.z:.6f}")
                    print(f"成功更新初始位置坐标，方向: {direct}, 变化量: {actual_change:.2f}")
                else:
                    print("未找到有效的路径点")
                    return scene

                # 更新变化范围
                if direct == 'forward':
                    new_forward_range = forward_change_range - actual_change
                    new_backward_range = backward_change_range
                else:
                    new_forward_range = forward_change_range
                    new_backward_range = backward_change_range - actual_change

                print(f"更新变化范围: forward={new_forward_range:.2f}, backward={new_backward_range:.2f}")
                # 更新场景的变化范围
                mutate_scene.forward_change_range = new_forward_range
                mutate_scene.backward_change_range = new_backward_range
            else:
                # 非后向碰撞场景的原始处理逻辑
                # 获取所有Waypoint节点
                waypoints = root.findall('.//AssignRouteAction/Route/Waypoint/Position/WorldPosition')
                if not waypoints:
                    print("未找到路径点")
                    return scene
                first_wp = waypoints[0]

                if direct == 'forward':
                    # 删除first_wp节点
                    if len(waypoints) > 1:
                        # 创建一个字典来存储每个元素及其父元素的关系
                        parent_map = {c: p for p in tree.iter() for c in p}

                        # 获取Waypoint节点（first_wp的父节点的父节点）
                        waypoint_node = parent_map[parent_map[first_wp]]
                        # 删除第一个路径点
                        first_wp_parent = parent_map[waypoint_node]
                        first_wp_parent.remove(waypoint_node)

                        # 获取新的第一个路径点
                        new_wp = waypoints[1]
                        new_location = {
                            'x': float(new_wp.get('x')),
                            'y': float(new_wp.get('y')),
                            'z': float(new_wp.get('z'))
                        }

                        # 计算与原位置的距离作为变化量
                        dx = new_location['x'] - original_location['x']
                        dy = new_location['y'] - original_location['y']
                        dz = new_location['z'] - original_location['z']
                        distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                        # 确保变化量不超过forward_change_range
                        if distance > forward_change_range:
                            # 按比例缩放新位置
                            scale = forward_change_range / distance
                            new_location['x'] = original_location['x'] + dx * scale
                            new_location['y'] = original_location['y'] + dy * scale
                            new_location['z'] = original_location['z'] + dz * scale
                            actual_change = forward_change_range
                        else:
                            actual_change = distance

                        # 更新init_wp为新的第一个路径点坐标
                        init_wp.set('x', f"{new_location['x']:.6f}")
                        init_wp.set('y', f"{new_location['y']:.6f}")
                        init_wp.set('z', f"{new_location['z']:.6f}")
                        print(f"成功删除第一个路径点并更新初始位置，变化量: {actual_change:.2f}")
                    else:
                        print("路径点数量不足，无法删除")
                        return scene
                elif direct == 'backward':
                    # 随机生成距离，但不超过backward_change_range
                    if backward_change_range <= 0:
                        print("向后变化范围不足")
                        return scene
                    distance = random.uniform(40, min(50.0, backward_change_range))  # 随机距离范围5-20米或剩余变化范围
                    actual_change = distance

                    # 获取向后的路径点
                    # new_transform = get_new_waypoint(
                    #     carla_map, original_location, direct, distance)
                    need_add_wp = add_waypoint_to_route(carla_map, original_location, distance)
                    if need_add_wp:
                        # 最远的点（即最后一个采样点）作为新的起点
                        farthest_loc = need_add_wp[-1].transform.location

                        # 更新 init_wp（ego 初始位置）
                        init_wp.set('x', f"{farthest_loc.x:.6f}")
                        init_wp.set('y', f"{farthest_loc.y:.6f}")
                        init_wp.set('z', f"{farthest_loc.z:.6f}")

                        # # 更新 first_wp（路线的第一个点）
                        # first_wp.set('x', f"{farthest_loc.x:.6f}")
                        # first_wp.set('y', f"{farthest_loc.y:.6f}")
                        # first_wp.set('z', f"{farthest_loc.z:.6f}")
                        # print(f"成功更新第一个路径点坐标，变化量: {actual_change:.2f}")
                        # 获取 Route 的父节点，用于插入新 Waypoint
                        parent_map = {c: p for p in tree.iter() for c in p}
                        route_node = None
                        for waypoint in waypoints:
                            parent = parent_map[waypoint]
                            grandparent = parent_map[parent]
                            g_grandparent = parent_map[grandparent]
                            if g_grandparent.tag == 'Route':
                                route_node = g_grandparent
                                break

                        if route_node is None:
                            print("未找到 Route 节点")
                            return scene
                        # 插入所有 backward_waypoints 到原 route 的最前面
                        # 逆序插入，因为 backward_waypoints 是从近到远，我们要从远到近加入路线
                        for wp in need_add_wp:
                            # 创建新的 Waypoint 元素
                            t = wp.transform
                            op_sce_ro = carla_to_openscenario_rotation(t.rotation)
                            new_waypoint = ET.Element('Waypoint')
                            new_waypoint.set('routeStrategy', 'shortest')  # 可选：估算 s 值

                            position = ET.SubElement(new_waypoint, 'Position')
                            world_pos = ET.SubElement(position, 'WorldPosition')
                            world_pos.set('x', f"{t.location.x:.6f}")
                            world_pos.set('y', f"{t.location.y:.6f}")
                            world_pos.set('z', f"{t.location.z:.6f}")
                            world_pos.set('h', f"{op_sce_ro['h']:.6f}")
                            world_pos.set('p', f"{op_sce_ro['p']:.6f}")
                            world_pos.set('r', f"{op_sce_ro['r']:.6f}")

                            # 插入到 route 的最前面
                            route_node.insert(0, new_waypoint)

                    else:
                        print("未找到有效的后向路径点")
                        return scene

                # 更新变化范围
                if direct == 'forward':
                    new_forward_range = forward_change_range - actual_change
                    new_backward_range = backward_change_range
                else:
                    new_forward_range = forward_change_range
                    new_backward_range = backward_change_range - actual_change

                print(f"更新变化范围: forward={new_forward_range:.2f}, backward={new_backward_range:.2f}")
                # 更新场景的变化范围
                mutate_scene.forward_change_range = new_forward_range
                mutate_scene.backward_change_range = new_backward_range

        output_file = input_file.replace('.xosc', f'_mutation_{cycle_count}_{mut_count}.xosc')
        tree.write(output_file, encoding='utf-8', xml_declaration=True)

        mutate_scene.file_name = output_file

    return mutate_scene
# carla_map = connect_carla()
# modify_waypoint_with_carla(carla_map,
#                             '/home/hahabai/PycharmProjects/ego_crime/llm_generate_openscenario/change.xosc',
#                             '/home/hahabai/PycharmProjects/ego_crime/llm_generate_openscenario/change.xosc')
