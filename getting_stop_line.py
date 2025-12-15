import extract_signal_info as esi
def get_stop_line(world, x, y):
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    if not traffic_lights:
        print("No traffic lights found in the map.")
        return
    traffic_light = None
    for tf in traffic_lights:
        lc = tf.get_location()
        if int(lc.x) == int(x) and int(lc.y) == int(y):
            traffic_light = tf
    tf_open_id = traffic_light.get_opendrive_id()
    stop_line_location = esi.get_signal_stop_line_point("carla_town01_signals.json", tf_open_id)
    return traffic_light, stop_line_location