import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, RosTimer, SetUseSimTime


def generate_launch_description():
    use_slam_arg = DeclareLaunchArgument(
        "use_slam", 
        default_value="false")
    
    use_slam = LaunchConfiguration("use_slam")
    

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py")
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        # arguments=["-d", os.path.join(
        #     get_package_share_directory("nav2_bringup"),
        #     "rviz",
        #     "nav2_default_view.rviz"
        # )],

        arguments=["-d", os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "rviz",
            "global_localization.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # rviz_localization = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", os.path.join(
    #         get_package_share_directory("bumperbot_localization"),
    #         "rviz",
    #         "global_localization.rviz"
    #     )],
    #     output="screen",
    #     parameters=[{"use_sim_time": True}],
    #     condition=UnlessCondition(use_slam)
    # )

    # rviz_slam = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", os.path.join(
    #         get_package_share_directory("bumperbot_mapping"),
    #         "rviz",
    #         "slam.rviz"
    #     )],
    #     output="screen",
    #     parameters=[{"use_sim_time": True}],
    #     condition=IfCondition(use_slam)
    # )


    
    # Gazebo phải xuất /clock ổn định trước khi các node dùng simulation time
    # tạo TF buffer. Nếu khởi động đồng thời, clock chuyển từ wall time về gần
    # zero sẽ làm RViz reset và xóa toàn bộ display.
    start_after_clock = RosTimer(
        period=2.0,
        actions=[
            controller,
            joystick,
            safety_stop,
            localization,
            slam,
            navigation,
            rviz,
        ]
    )

    return LaunchDescription([
        use_slam_arg,
        SetUseSimTime(True),
        gazebo,
        start_after_clock,
        # rviz_localization,
        # rviz_slam,
    ])
