import random
class Scene:
    def __init__(self):
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
        self.direct = random.choice(['forward', 'backward'])
        self.min_speed = 5
        self.max_speed = 100
        self.forward_change_range = 10
        self.backward_change_range = 50
        # intersection
        # self.ego_s = (177.1697540283203, 55.48942947387695, 0.0)
        # self.ego_e = (134.98974609375, 55.49456787109375, 0.0)
        # self.category = 'intersection'
        # self.file_name = "/home/hahabai/PycharmProjects/ego_crime/llm_generate_openscenario/Intersection_Conflicts.xosc"
        # self.curvature = None
        # change_line
        # self.ego_s = (383.1048889160156, -18.292522430419922, 0.0)
        # self.ego_e = (315.2305908203125, 17.19019889831543, 1.068198800086975)
        # self.category = 'change_lane'
        # self.file_name = "/home/hahabai/PycharmProjects/ego_crime/llm_generate_openscenario/Change_Lane_Conflicts.xosc"
        # self.curvature = None
        # rear_end
        # self.ego_s = (308.22137451171875, 13.57004451751709, 1.2320460081100464)
        # self.ego_e = (62.671043395996094, -67.92257690429688, 0.0)
        # self.category = 'rear_end'
        # self.file_name = "/home/hahabai/PycharmProjects/ego_crime/llm_generate_openscenario/Rear_End_Conflicts.xosc"
        # self.curvature = None
        # loss_control
        # self.ego_s = (379.6932373046875, -19.074064254760742, 0.0)
        # self.ego_e = (357.9949951171875, 7.322314262390137, 0.07823474705219269)
        # self.category = 'loss_control'
        # self.file_name = "/home/hahabai/PycharmProjects/ego_crime/llm_generate_openscenario/Loss_Control_Conflicts.xosc"
        # self.curvature = 0.018512264495997445
        # environment_factor
        self.ego_s = (-16.194501876831055, -33.84495544433594, 0.0)
        self.ego_e = (-15.751138687133789, 66.17171478271484, 0.07823474705219269)
        self.category = 'environment_factor'
        self.file_name = "/home/hahabai/PycharmProjects/ego_crime/llm_generate_openscenario/Environment_Conflicts_ori.xosc"
        self.curvature = 0.48