from simple_launch import SimpleLauncher
from launch.actions import ExecuteProcess
import os

sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('world', 'easy')
sl.declare_arg('gui', True)

sl.declare_arg('n', 2)


buoys = {'red': ((0, 1),),
         'green': ((0, 2),)}


def launch_setup():

    # launch world
    world = sl.arg('world')
    if 'aquabot' not in world:
        world = f'aquabot_{world}'
    world.strip('.sdf')

    gz_args = '-r'
    if not sl.arg('gui'):
        gz_args += ' -s'
    sl.gz_launch(sl.find('aquabot_gz', world+'.sdf'), gz_args=gz_args)

    spawn_script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'spawn.bash')
    for i in range(sl.arg('n')):
        sl.add_action(ExecuteProcess(cmd=['bash', spawn_script, str(i+1)]))

    for color, coords in buoys.items():
        for i, (x,y) in enumerate(coords):
            sl.spawn_gz_model(f'buoy_{color}{i}', model_file=sl.find('asv_race_gz', f'buoy_{color}.sdf'))

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
