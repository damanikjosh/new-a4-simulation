from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch three UAV vehicle nodes (instances).
    # for i in range(3):
    #     node = Node(
    #         package='mission_planner',
    #         executable='vehicle',
    #         name=f'uav_{i}',
    #         output='screen',
    #         parameters=[{'vehicle_name': f'px4_{i}'}]
    #     )

    uav1_node = Node(
        package='mission_planner',
        executable='vehicle',
        name='uav_0',
        output='screen',
        parameters=[{'vehicle_name': 'uav1',
                     'vehicle_type': 0,
                     'port': 14541}]  # 0 for UAV
    )

    uav2_node = Node(
        package='mission_planner',
        executable='vehicle',
        name='uav_1',
        output='screen',
        parameters=[{'vehicle_name': 'uav2',
                     'vehicle_type': 0,
                     'port': 14542}]  # 0 for UAV
    )

    uav3_node = Node(
        package='mission_planner',
        executable='vehicle',
        name='uav_2',
        output='screen',
        parameters=[{'vehicle_name': 'uav3',
                     'vehicle_type': 0,
                     'port': 14543}]  # 0 for UAV
    )

    usv4_node = Node(
        package='mission_planner',
        executable='vehicle',
        name='usv_4',
        output='screen',
        parameters=[{'vehicle_name': 'usv4',
                     'vehicle_type': 1,
                     'port': 14544}]  # 1 for USV
    )

    usv5_node = Node(
        package='mission_planner',
        executable='vehicle',
        name='usv_5',
        output='screen',
        parameters=[{'vehicle_name': 'usv5',
                     'vehicle_type': 1,
                     'port': 14545}]  # 1 for USV
    )

    enemy8_node = Node(
        package='mission_planner',
        executable='vehicle',
        name='enemy_8',
        output='screen',
        parameters=[{'vehicle_name': 'enemy8',
                     'vehicle_type': 2,
                     'port': 14548}]  # 2 for enemy
    )

    enemy9_node = Node(
        package='mission_planner',
        executable='vehicle',
        name='enemy_9',
        output='screen',
        parameters=[{'vehicle_name': 'enemy9',
                     'vehicle_type': 2,
                     'port': 14549}]  # 2 for enemy
    )

    mission_planner_node = Node(
        package='mission_planner',
        executable='mission_planner_node.py',
        name='mission_planner',
        output='screen'
    )

    ld.add_action(uav1_node)
    ld.add_action(uav2_node)
    ld.add_action(uav3_node)
    ld.add_action(usv4_node)
    ld.add_action(usv5_node)
    ld.add_action(enemy8_node)
    ld.add_action(enemy9_node)
    ld.add_action(mission_planner_node)

    return ld
