from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', 
                 '/barge_cmd@nav_msgs/msg/Odometry@gz.msgs.Odometry', 
                 '/barge/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry', 
                 '--ros-args', '-r', '__node:=barge_bridge'],
            name='barge_pos_cmd_bridge',
            output='screen',
        ),

        # Node(
        #     package='pohang_barge',
        #     executable='pohang_barge_node',
        #     name='pohang_barge_node',
        #     output='log'
        # ),
    ])