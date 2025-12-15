import xml.etree.ElementTree as ET
from xml.dom import minidom

def extract_data(file_path, category):
    # 解析XML文件
    tree = ET.parse(file_path)
    root = tree.getroot()

    # 初始化结果存储字典
    result = {
        "EgoRoute": {
            "Route_Coordinates": [],  # EgoRoute的Route坐标点
            "TrafficSignal_Coordinate": None  # EgoRoute的交通信号灯坐标
        },
        "NPCRoute": {
            "Route_Coordinates": [],  # NPCRoute的Route坐标点
            "TrafficSignal_Coordinate": None,  # NPCRoute的交通信号灯坐标
            "MaxSpeed": None
        }
    }

    # 第一步：定位核心节点（Storyboard→Story→Act，原文件中Act名为"Behavior"）
    # 先找Storyboard下的Story（原文件只有1个Story）
    story = root.find("Storyboard/Story")
    if not story:
        print("错误：未找到Story节点（路径：Storyboard→Story）")
        return result

    act = story.find(".//Act[@name='Behavior']")
    if not act:
        act = story.find(".//Act[@name='EgoBehavior']")
        if not act:
            print("错误：未找到Act节点（期望名称：'Behavior' 或 'EgoBehavior'）")
            return result
    # 第二步：提取EgoRoute相关数据（ManeuverGroup[@name='EgoRoute']）
    ego_maneuver_group = act.find(".//ManeuverGroup[@name='EgoRoute']")
    if ego_maneuver_group:
        # 1.1 提取EgoRoute的Route坐标（嵌套在Maneuver→Event→Action→PrivateAction→RoutingAction）
        # 先找EgoRoute下的AssignRouteAction（Route的父节点）
        ego_assign_route = ego_maneuver_group.find(
            ".//Maneuver/Event/Action/PrivateAction/RoutingAction/AssignRouteAction/Route"
        )
        if ego_assign_route:
            # 提取Route下的所有Waypoint坐标（原文件固定2个）
            waypoints = ego_assign_route.findall(".//Waypoint/Position/WorldPosition")
            for wp in waypoints:
                x = wp.get("x")
                y = wp.get("y")
                z = wp.get("z")
                if x and y and z:  # 确保坐标值存在（避免空值）
                    result["EgoRoute"]["Route_Coordinates"].append((float(x), float(y), float(z)))

        # 1.1 提取EgoRoute的TrafficSignal坐标（嵌套在Maneuver→Event→Action→GlobalAction→...）
        # 先找EgoRoute下的TrafficSignalStateAction（精准匹配完整层级）
        if category=='intersection':
            ego_traffic_signal = ego_maneuver_group.findall(
                ".//GlobalAction/InfrastructureAction/TrafficSignalAction/TrafficSignalStateAction"
            )
            # ego_traffic_signal = ego_maneuver_group.find(
            #     ".//GlobalAction/InfrastructureAction/TrafficSignalAction/TrafficSignalStateAction"
            # )
            if ego_traffic_signal:
                # 从name属性解析坐标（格式：pos=xxx.xxx,xxx.xxx）
                signal_name = ego_traffic_signal[0].get("name")
                if signal_name and "pos=" in signal_name:
                    # 截取"pos="后的坐标部分，再分割x和y
                    pos_part = signal_name.split("pos=")[1]
                    x_str, y_str = pos_part.split(",")
                    result["EgoRoute"]["TrafficSignal_Coordinate"] = (float(x_str), float(y_str))
        else:
            result["EgoRoute"]["TrafficSignal_Coordinate"] = (None, None)

    # 第三步：提取NPCRoute相关数据（ManeuverGroup[@name='NPCRoute']）
    npc_maneuver_group = act.find(".//ManeuverGroup[@name='NPCRoute']")
    if npc_maneuver_group:
        npc_assign_route = npc_maneuver_group.find(
            ".//Maneuver/Event/Action/PrivateAction/RoutingAction/AssignRouteAction/Route"
        )
        if npc_assign_route:
            # 提取Route下的所有Waypoint坐标（原文件固定2个）
            waypoints = npc_assign_route.findall(".//Waypoint/Position/WorldPosition")
            for wp in waypoints:
                x = wp.get("x")
                y = wp.get("y")
                z = wp.get("z")
                if x and y and z:  # 确保坐标值存在（避免空值）
                    result["NPCRoute"]["Route_Coordinates"].append((float(x), float(y), float(z)))
        if category == 'intersection':
            # 提取NPCRoute的TrafficSignal坐标（层级与EgoRoute一致）
            npc_traffic_signal = npc_maneuver_group.findall(
                ".//GlobalAction/InfrastructureAction/TrafficSignalAction/TrafficSignalStateAction"
            )
            if npc_traffic_signal:
                signal_name = npc_traffic_signal[0].get("name")
                if signal_name and "pos=" in signal_name:
                    pos_part = signal_name.split("pos=")[1]
                    x_str, y_str = pos_part.split(",")
                    result["NPCRoute"]["TrafficSignal_Coordinate"] = (float(x_str), float(y_str))
        else:
            result["EgoRoute"]["TrafficSignal_Coordinate"] = (None, None)
    # 查找 name="npc" 的 ScenarioObject，然后找到其 Performance 的 maxSpeed
    for obj in root.findall('.//ScenarioObject'):
        if obj.get('name') == 'npc':
            performance = obj.find('.//Performance')
            if performance is not None:
                max_speed = performance.get('maxSpeed')
                result["NPCRoute"]["MaxSpeed"] = float(max_speed)
            break
    # 第四步：验证提取结果（若有缺失打印提示）
    if not result["EgoRoute"]["Route_Coordinates"]:
        print("警告：未提取到EgoRoute的Route坐标")
    if not result["EgoRoute"]["TrafficSignal_Coordinate"]:
        print("警告：未提取到EgoRoute的交通信号灯坐标")
    if not result["NPCRoute"]["TrafficSignal_Coordinate"]:
        print("警告：未提取到NPCRoute的交通信号灯坐标")

    return result

def modify_xml(original_path, new_path):
    """
    处理XML文件：
    1. 删除Entities下name="ego_vehicle"的ScenarioObject
    1. 删除Storyboard→Init→Actions下Private entityRef="ego_vehicle"的节点
    3. 删除ManeuverGroup[@name='EgoRoute']节点
    4. 删除ManeuverGroup[@name='NPCRoute']下的Event[@name='GreenEventNPC']节点
    5. 保存到新文件intersection_new.txt
    """
    # --------------------------
    # 1. 解析原始XML文件
    # --------------------------
    try:
        tree = ET.parse(original_path)
        root = tree.getroot()
        print(f"✅ 成功解析原始文件：{original_path}")
    except FileNotFoundError:
        print(f"❌ 错误：未找到原始文件！路径：{original_path}")
        return
    except ET.ParseError as e:
        print(f"❌ 错误：XML格式解析失败！原因：{str(e)}")
        return

    # --------------------------
    # 新增任务1：删除Entities下 name="ego_vehicle" 的 ScenarioObject
    # 逻辑：定位Entities父节点 → 遍历子节点找到目标ScenarioObject → 删除
    # --------------------------
    # 定位Entities父节点（根节点下的Entities标签）
    entities_parent = root.find("Entities")
    if not entities_parent:
        print("⚠️ 警告：未找到Entities节点，跳过ego_vehicle删除")
    else:
        # 遍历Entities的子节点，找到name="ego_vehicle"的ScenarioObject
        ego_scenario_to_delete = None
        for child in entities_parent:
            if child.tag == "ScenarioObject" and child.get("name") == "ego_vehicle":
                ego_scenario_to_delete = child
                break
        # 执行删除
        if ego_scenario_to_delete:
            entities_parent.remove(ego_scenario_to_delete)
            print("✅ 成功删除 Entities 下 name='ego_vehicle' 的 ScenarioObject")
        else:
            print("⚠️ 警告：Entities下未找到name='ego_vehicle'的ScenarioObject（可能已删除）")

    # --------------------------
    # 新增任务2：删除Storyboard→Init→Actions下 Private entityRef="ego_vehicle" 的节点
    # 逻辑：定位Init→Actions父节点 → 遍历子节点找到目标Private → 删除
    # --------------------------
    # 定位Init→Actions父节点（Storyboard→Init→Actions）
    init_actions_parent = root.find("Storyboard/Init/Actions")
    if not init_actions_parent:
        print("⚠️ 警告：未找到Storyboard→Init→Actions节点，跳过ego_vehicle的Private删除")
    else:
        # 遍历Actions的子节点，找到entityRef="ego_vehicle"的Private节点
        ego_private_to_delete = None
        for child in init_actions_parent:
            if child.tag == "Private" and child.get("entityRef") == "ego_vehicle":
                ego_private_to_delete = child
                break
        # 执行删除
        if ego_private_to_delete:
            init_actions_parent.remove(ego_private_to_delete)
            print("✅ 成功删除 Storyboard→Init→Actions 下 entityRef='ego_vehicle' 的 Private 节点")
        else:
            print("⚠️ 警告：Init→Actions下未找到entityRef='ego_vehicle'的Private节点（可能已删除）")

    # --------------------------
    # 原有任务1：删除 ManeuverGroup[@name='EgoRoute']
    # --------------------------
    act_parent = root.find("Storyboard/Story/Act[@name='Behavior']")
    if not act_parent:
        act_parent = root.find("Storyboard/Story/Act[@name='EgoBehavior']")
        if not act_parent:
            print("❌ 错误：未找到核心父节点 Act[@name='Behavior']，跳过EgoRoute删除")
    ego_mg_to_delete = None
    for child in act_parent:
        if child.tag == "ManeuverGroup" and child.get("name") == "EgoRoute":
            ego_mg_to_delete = child
            break
    if ego_mg_to_delete:
        act_parent.remove(ego_mg_to_delete)
        print("✅ 成功删除 ManeuverGroup[@name='EgoRoute'] 节点")
    else:
        print("⚠️ 警告：未找到 ManeuverGroup[@name='EgoRoute'] 节点（可能已删除）")

    # --------------------------
    # 原有任务2：删除 NPCRoute 下的 Event[@name='GreenEventNPC']
    # --------------------------
    if act_parent:  # 确保Act节点存在
        # 找NPCRoute节点
        npc_mg = None
        for child in act_parent:
            if child.tag == "ManeuverGroup" and child.get("name") == "NPCRoute":
                npc_mg = child
                break
        if not npc_mg:
            print("⚠️ 警告：未找到 ManeuverGroup[@name='NPCRoute'] 节点，跳过GreenEventNPC删除")
        else:
            # 找NPCRoute下的Maneuver节点
            npc_maneuver = None
            for child in npc_mg:
                if child.tag == "Maneuver":
                    npc_maneuver = child
                    break
            if not npc_maneuver:
                print("⚠️ 警告：NPCRoute下未找到 Maneuver 节点，跳过GreenEventNPC删除")
            else:
                # 找Maneuver下的GreenEventNPC
                green_event_to_delete = None
                for child in npc_maneuver:
                    if child.tag == "Event" and child.get("name") == "GreenEventNPC":
                        green_event_to_delete = child
                        break
                if green_event_to_delete:
                    npc_maneuver.remove(green_event_to_delete)
                    print("✅ 成功删除 NPCRoute 下的 Event[@name='GreenEventNPC'] 节点")
                else:
                    print("⚠️ 警告：NPCRoute下未找到 Event[@name='GreenEventNPC'] 节点（可能已删除）")

    # --------------------------
    # 3. 格式化并保存新文件
    # --------------------------
    try:
        rough_xml = ET.tostring(root, encoding="utf-8", method="xml")
        pretty_xml = minidom.parseString(rough_xml).toprettyxml(indent="  ", newl="\n")
        pretty_xml = "\n".join([line for line in pretty_xml.split("\n") if line.strip()])  # 去空行

        with open(new_path, "w", encoding="utf-8") as f:
            f.write(pretty_xml)
        print(f"\n✅ 所有修改完成！新文件已保存至：{new_path}")
    except Exception as e:
        print(f"❌ 错误：保存新文件失败！原因：{str(e)}")
        return


# 执行提取并打印结果
if __name__ == "__main__":
    # ！！！请替换为你的intersection.txt实际路径（相对路径/绝对路径均可）
    ORIGINAL_FILE = "../generation_opensceanrio_files/intersection.txt"  # 原始文件路径（例："C:/Users/XXX/Desktop/intersection.txt"）
    NEW_FILE = "../generation_opensceanrio_files/intersection_new.txt"  # 新文件保存路径（例："C:/Users/XXX/Desktop/intersection_new.txt"）

    try:
        extracted_data = extract_data(ORIGINAL_FILE)
        # 格式化输出结果
        print("=" * 50)
        print("Intersection.txt 数据提取结果")
        print("=" * 50)
        print("\n1. EgoRoute（自我车辆路径组）:")
        print(f"   - Route坐标点（2个）: {extracted_data['EgoRoute']['Route_Coordinates']}")
        print(f"   - 交通信号灯坐标: {extracted_data['EgoRoute']['TrafficSignal_Coordinate']}")

        print("\n1. NPCRoute（NPC车辆路径组）:")
        print(f"   - 交通信号灯坐标: {extracted_data['NPCRoute']['TrafficSignal_Coordinate']}")
        print("=" * 50)
    except FileNotFoundError:
        print(f"错误：文件未找到，请检查路径是否正确！路径：{ORIGINAL_FILE}")
    except ET.ParseError:
        print("错误：XML文件解析失败，可能文件格式损坏或非标准XML")

    modify_xml(original_path=ORIGINAL_FILE, new_path=NEW_FILE)