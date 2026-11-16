from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('num', 1)


def launch_setup():

    bridges = [GazeboBridge.clock()]

    name = 'heron' + str(sl.arg('num'))

    # # turbine bridges
    # bridges.append(('/vrx/windturbinesinspection/windturbine_checkup',
    #                 '/vrx/windturbinesinspection/windturbine_checkup',
    #                 'std_msgs/msg/String', GazeboBridge.ros2gz))
    # # bridges.append(('/aquabot/ais_sensor/windturbines_positions',
    # #                 '/aquabot/ais_sensor/windturbines_positions',
    # #                 'geometry_msgs/msg/Pose', GazeboBridge.gz2ros))
    # bridges.append(('/aquabot/ais_sensor/windturbines',
    #                 '/aquabot/ais_sensor/windturbines',
    #                 'sensor_msgs/NavSatFix', GazeboBridge.gz2ros))
    #
    # sl.create_gz_bridge(bridges)

    # spawn aquabot robot
    with sl.group(ns = name):

        sl.robot_state_publisher('heron_description', 'heron.xacro',
                                 xacro_args={'num': sl.arg('num')})
        sl.spawn_gz_model(name, spawn_args=['-y', str(5*sl.arg('num'))])

        # add bridges for this model
        bridges = []
        bridges.append((f'/world/aquabot/model/{name}/joint_state',
                        'joint_states',
                        'sensor_msgs/msg/JointState', GazeboBridge.gz2ros))
        # command
        for side in ('left','right'):
            bridges.append((f'/{name}/thrusters/{side}/thrust',
                            f'thrusters/{side}/thrust',
                            'std_msgs/Float64', GazeboBridge.ros2gz))

        # sensors
        bridges.append((f'/{name}/navsat',
                        'sensors/gps_raw',
                        'sensor_msgs/NavSatFix', GazeboBridge.gz2ros))
        bridges.append((f'/{name}/imu',
                        'sensors/imu_raw',
                        'sensor_msgs/Imu', GazeboBridge.gz2ros))
        bridges.append(GazeboBridge(f'/{name}/camera',
                                    'sensors/image_raw',
                                    'sensor_msgs/Image',
                                    GazeboBridge.gz2ros))

        sl.create_gz_bridge(bridges)

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
