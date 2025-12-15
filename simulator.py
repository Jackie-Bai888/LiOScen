import os
import time

import carla
import random
import math
import numpy as np

MAP_DIR = '/home/hahabai/code/apollo-8.0.0/modules/map/data/'
MAP_NAMES = ['carla_town01', 'carla_town02', 'carla_town04', 'carla_town07', 'carla_town10hd']
def inital_carla():
    # 连接到CARLA服务器
    client = carla.Client('localhost', 2000)
    client.set_timeout(3.0)
    # 获取世界对象和地图
    world = client.get_world()
    carla_map = world.get_map()
    return client, world, carla_map

def set_road(world, carla_map):
    #目前没有找到设置道路积水、积雪和积冰的方法
    random_waypoint = random.choice(carla_map.generate_waypoints(1))
    #获取道路材质
    materials = world.get_map().get_waypoint_materials(random_waypoint)

    # 修改道路材质
    for material in materials:
        material.road_friction = 0.2

    # 应用道路材质修改
    world.get_map().set_waypoint_materials(random_waypoint, materials)


def judge_lanes_direct(ve_waypoint, adjacent_waypoint):
    # 计算两条车道是否是同向车道
    # 计算邻近车道与ego车道的方向向量
    ve_direction = ve_waypoint.transform.rotation.get_forward_vector()
    adjacent_directions = adjacent_waypoint.transform.rotation.get_forward_vector()
    ve_length = ve_direction.length()
    ad_length = adjacent_directions.length()
    inner = ve_direction.dot(adjacent_directions)
    cos_va = ve_direction.dot(adjacent_directions)/(ve_length*ad_length)
    if cos_va > 1:
        cos_va = 1
    elif cos_va < -1:
        cos_va = -1
    is_same_direction = abs(math.degrees(math.acos(cos_va)))<90
    print("邻近车道与ego车道是否同向: {}".format(is_same_direction))
    return is_same_direction

def straight_line_ana(ini_wp, mid_wp, des_wp):
    k1 = (mid_wp.transform.location.y - ini_wp.transform.location.y)/(mid_wp.transform.location.x - ini_wp.transform.location.x)
    k2 = (des_wp.transform.location.y - ini_wp.transform.location.y)/(des_wp.transform.location.x - ini_wp.transform.location.x)
    if k1 == k2:
        print('start waypoint and destination waypoint on the same straight line.')
        return True
    else:
        print('start waypoint and destination waypoint not on the same straight line.')
        return False

def get_junction_wp(map, behavior):
    wp = map.generate_waypoints(1.0)   #按照距离1生成waypoints
    for point in wp:
        if point.lane_type == carla.LaneType.Driving and behavior == 'cross':
            junc = point.is_junction
            if junc == True:
                get_junction = point.get_junction()
                wp_dr = get_junction.get_waypoints(carla.LaneType.Driving)
                return wp_dr[0][0], wp_dr[0][1], wp_dr[3][0], wp_dr[3][1]
    return None

def get_ve_start_wp(ego_waypoint, position, driving_distance, judge_param):
    # NPC的坐标需要在行驶路线中而不是在起点和终点
    location_num = random.randint(5, driving_distance-5)
    print('--随机npc location的值--',location_num)
    if ('front' == position) or ('rear' == position):
        npc_waypoint = ego_waypoint.next(location_num)[0]
        return npc_waypoint
    else:
        other_lane_wp = None  # 获取与ego车辆邻近的车道
        if 'left' in position:
            other_lane_wp = ego_waypoint.get_left_lane()
        if 'right' in position:
            other_lane_wp = ego_waypoint.get_right_lane()
        if other_lane_wp.lane_type == carla.LaneType.Driving:
            if judge_param == judge_lanes_direct(ego_waypoint, other_lane_wp):
                npc_waypoint = other_lane_wp.next(location_num)[0]
                return npc_waypoint
        return None


def get_pedestrian_start_wp(carla_map, ego_waypoint, position, driving_distance):
    # pedestrian的坐标需要在行驶路线中而不是在起点和终点
    location_num = random.randint(1, driving_distance - 1)
    print('--随机npc location的值--', location_num)
    next_waypoint = ego_waypoint.next(location_num)[0]
    if position == 'left front':
        pe_ini_wp = next_waypoint.get_left_lane()
    if position == 'right front':
        pe_ini_wp = next_waypoint.get_right_lane()
    # 让行人从路边开始过马路
    if pe_ini_wp.lane_type == carla.LaneType.Shoulder:
        return pe_ini_wp
    else:
        return None



def get_driving_location(world, danger_analysis_result, driving_distance, not_need_wp):
    if danger_analysis_result[1] == 'cut in':
        judge_param = True
    else:
        judge_param = False
    position = danger_analysis_result[2]
    behavior = danger_analysis_result[1]
    # # 获取当前的地图对象
    map = world.get_map()
    # if behavior == 'cross':
    #     driving_wp = get_junction_wp(map, behavior)
    # else:
    spawn_points = map.get_spawn_points()
    for point in spawn_points:
        if point in not_need_wp:
            continue
        ego_waypoint = map.get_waypoint(point.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        npc_driving_wp = get_ve_start_wp(ego_waypoint, position, driving_distance, judge_param)
        if npc_driving_wp is not None:
            break
    return ego_waypoint, npc_driving_wp



def get_walk_location(world, danger_analysis_result, driving_distance, not_need_wp):
    #目前只是找车道紧挨着的一个人行道，而且占时将人出现的位置确定在车辆前方
    position = danger_analysis_result[2]
    # # 获取当前的地图对象
    carla_map = world.get_map()
    # recommended spawn points for the creation of vehicles
    spawn_points = carla_map.get_spawn_points()
    for point in spawn_points:
        if point in not_need_wp:
            continue
        ego_waypoint = carla_map.get_waypoint(point.location,project_to_road=True, lane_type=carla.LaneType.Driving)
        pedestrian_wp = get_pedestrian_start_wp(carla_map, ego_waypoint, position, driving_distance)
        if pedestrian_wp is not None:
            break
    return ego_waypoint, pedestrian_wp


def load_map(danger_info):
    # 加载地图
    sub, pos = danger_info[0], danger_info[-1]
    find_lane_type = None
    # CITY_DRIVING, SIDEWALK
    # left_neighbor_forward_lane_id  right_neighbor_forward_lane_id
    # type SIDEWALK
    if 'pedestrian' in sub:
        find_lane_type = 'SIDEWALK'
    else:
        if pos == 'left front':
            find_lane_type = 'left_neighbor_forward_lane_id'
        elif pos == 'right front':
            find_lane_type = 'right_neighbor_forward_lane_id'
    for m in MAP_NAMES:
        map_path = os.path.join(MAP_DIR, m)
        sim_map_path = os.path.join(map_path, 'sim_map.txt')
        with open(sim_map_path, 'r') as file:
            content = file.read()
        if find_lane_type in content:
            if find_lane_type == 'SIDEWALK':
                return m
            else:
                con_ls = content.split('\n}\n')
                for tag in con_ls:
                    if find_lane_type in tag and 'type: CITY_DRIVING' in tag:
                        find_id = tag.split(find_lane_type+' {\n')[-1].split('\n')[0]
                        for new_tag in con_ls:
                            if 'lane' in new_tag and find_id in new_tag and 'type: CITY_DRIVING' in tag:
                                return m
    return None

def reset_ego_ini_position(carla_map):
    check_distance = 15.0
    spawn_points = carla_map.get_spawn_points()

    # 优先筛选十字路口入口附近的候选点
    candidates = []
    for point in spawn_points:
        # 获取车道信息并过滤非驾驶车道
        ego_waypoint = carla_map.get_waypoint(
            point.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        if not ego_waypoint:
            continue

        # 关键条件判断：当前不在路口，但前方即将进入路口
        if not ego_waypoint.is_intersection:
            # 检查前方 check_distance 米内的路径点
            next_waypoints = ego_waypoint.next(check_distance)
            for next_wp in next_waypoints:
                if next_wp.is_intersection:
                    # 调整生成点方向与车道方向对齐
                    new_point = carla.Transform(
                        location=point.location,
                        rotation=ego_waypoint.transform.rotation  # 使用车道方向而非原始生成点方向
                    )
                    candidates.append(new_point)
                    break  # 找到即跳出循环

    # 优先返回符合条件的候选点
    if candidates:
        return candidates[2]  # 返回第一个候选点，可改为随机选择


if __name__ == '__main__':
    client, world, carla_map = inital_carla()
    danger_analysis_result = ['pedestrian', 'cross', 'front']
    # danger_analysis_result = ['car', 'insert', 'right front']
    # m = load_map(danger_analysis_result)
    # _, pe_wp = get_walk_location(world, danger_analysis_result)
    vehicle_list = world.get_actors().filter('vehicle.*')
    ego_vehicle = vehicle_list[0]
    # 获取当前车辆的位置
    ego_location = ego_vehicle.get_location()
    # 获取车辆当前所在的道路点
    ego_waypoint = carla_map.get_waypoint(ego_location).next(10)[0]
    lane_wp = ego_waypoint.get_right_lane()
    ve_wp, pre_wp = get_walk_location(world, danger_analysis_result, 5)
    # set_pedestrian(world, lane_wp.transform)
    # reset_scene()
    # reload_map(None, 'Town03')
    # world, carla_map = inital_carla()
    # ego_waypoint = get_ego_waypoint(world, carla_map)
    # npc_ve = get_npc(world, ego_waypoint, 'front')
    # set_autopilot(npc_ve)
    # set_brake(npc_ve)
    # behavior_function = globals()['set_brake']
    # behavior_function(npc_ve)
    # generate_npc(world, carla_map, ego_waypoint)
    # add_water(world, carla_map, ego_waypoint)
    # add_ice(carla_map, ego_waypoint)
    # set_weather(world)
    # set_road(world, carla_map)