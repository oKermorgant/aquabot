from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)
    sl.declare_arg('rviz', True)

    with sl.group(if_arg = 'rviz'):
        sl.rviz(sl.find('aquabot_ekf', 'ekf.rviz'))

    # TODO check namespaces and run EKF for gps_position and imu
    sl.node('aquabot_ekf','gps2pose')

    return sl.launch_description()
