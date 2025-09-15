from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import OpaqueFunction

def _set_tb3_env(context, *args, **kwargs):
# LaunchConfiguration 값을 문자열로 풀어서 즉시 환경변수에 주입
    os.environ['TURTLEBOT3_MODEL'] = context.perform_substitution(LaunchConfiguration('tb3_model'))
    return []

def generate_launch_description():
    # === Arguments ===
    tb3_model = DeclareLaunchArgument(
        'tb3_model', default_value='waffle',
        description='TurtleBot3 model: burger | waffle | waffle_pi'
    )
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    start_tb3 = DeclareLaunchArgument('start_tb3', default_value='True',
                                    description='Start turtlebot3_bringup/robot.launch.py')
    start_realsense = DeclareLaunchArgument('start_realsense', default_value='True')
    use_slam = DeclareLaunchArgument('use_slam', default_value='True',
                                    description='Use slam_toolbox via Nav2 bringup')
    map_yaml = DeclareLaunchArgument('map', default_value='',
                                    description='Map yaml path (set when use_slam=false)')
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('yolo_people_obstacles'),
            'config', 'nav2_params.yaml'
        ]),
        description='Nav2 parameter file with obstacle_layer.people_cloud configured'
    )
    # YOLO node params
    model_path = DeclareLaunchArgument('model_path', default_value='yolo11n-seg.engine')
    yolo_conf = DeclareLaunchArgument('yolo_conf', default_value='0.5')
    yolo_stride = DeclareLaunchArgument('yolo_stride', default_value='4')
    yolo_use_seg = DeclareLaunchArgument('yolo_use_seg', default_value='True')
    max_depth_m = DeclareLaunchArgument('max_depth_m', default_value='5.0')
    min_depth_m = DeclareLaunchArgument('min_depth_m', default_value='0.3')

    # Frames / static TF (base_link -> camera_link)
    parent_frame = DeclareLaunchArgument('parent_frame', default_value='base_link')
    camera_link = DeclareLaunchArgument('camera_link', default_value='camera_link')
    # 카메라 장착 위치(예시): x=0.12m, y=0.0m, z=0.20m, 회전 없음
    cam_tx = DeclareLaunchArgument('cam_tx', default_value='0.12')
    cam_ty = DeclareLaunchArgument('cam_ty', default_value='0.0')
    cam_tz = DeclareLaunchArgument('cam_tz', default_value='0.20')
    cam_qx = DeclareLaunchArgument('cam_qx', default_value='0.0')
    cam_qy = DeclareLaunchArgument('cam_qy', default_value='0.0')
    cam_qz = DeclareLaunchArgument('cam_qz', default_value='0.0')
    cam_qw = DeclareLaunchArgument('cam_qw', default_value='1.0')

    # === Paths ===
    pkg_tb3_bringup = get_package_share_directory('turtlebot3_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_realsense = get_package_share_directory('realsense2_camera')

    # === Environment ===
    set_tb3_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=LaunchConfiguration('tb3_model')
    )

    # === Nodes / Includes ===

    # 1) TurtleBot3 bringup (OpenCR, 로봇 상태 등)
    tb3_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_bringup, 'launch', 'robot.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_tb3')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 2) RealSense D435i
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_realsense, 'launch', 'rs_launch.py')),
        condition=IfCondition(LaunchConfiguration('start_realsense')),
        launch_arguments={
            'align_depth': 'True',
            'pointcloud.enable': 'True',
            # 필요시 fps/해상도 조절:
            # 'color_width': '640', 'color_height': '480', 'color_fps': '30',
            # 'depth_width': '640', 'depth_height': '480', 'depth_fps': '30'
        }.items()
    )

    # 3) base_link -> camera_link 고정 TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        condition=IfCondition(LaunchConfiguration('start_realsense')),
        # x y z qx qy qz qw parent child
        arguments=[
            LaunchConfiguration('cam_tx'),
            LaunchConfiguration('cam_ty'),
            LaunchConfiguration('cam_tz'),
            LaunchConfiguration('cam_qx'),
            LaunchConfiguration('cam_qy'),
            LaunchConfiguration('cam_qz'),
            LaunchConfiguration('cam_qw'),
            LaunchConfiguration('parent_frame'),
            LaunchConfiguration('camera_link'),
        ]
    )

    # 4) YOLOv11 사람→PointCloud2 노드
    yolo_people_node = Node(
        package='yolo_people_obstacles',
        executable='yolo_people_to_cloud_node',
        name='yolo_people_to_cloud',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'conf': LaunchConfiguration('yolo_conf'),
            'stride': LaunchConfiguration('yolo_stride'),
            'use_segmentation': LaunchConfiguration('yolo_use_seg'),
            'max_depth_m': LaunchConfiguration('max_depth_m'),
            'min_depth_m': LaunchConfiguration('min_depth_m'),
        }]
        # 필요 시 remappings 추가 가능:
        # remappings=[
        #   ('/camera/color/image_raw', '/your/camera/topic'),
        #   ('/camera/aligned_depth_to_color/image_raw', '/your/depth/topic'),
        #   ('/camera/color/camera_info', '/your/camera_info/topic'),
        # ]
    )

    # 5) Nav2 bringup (slam 또는 map 사용)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'slam': LaunchConfiguration('use_slam'),
            'map': LaunchConfiguration('map'),
        }.items()
    )

    return LaunchDescription([
        tb3_model, use_sim_time, start_tb3, start_realsense,
        use_slam, map_yaml, nav2_params_arg,
        model_path, yolo_conf, yolo_stride, yolo_use_seg, max_depth_m, min_depth_m,
        parent_frame, camera_link, cam_tx, cam_ty, cam_tz, cam_qx, cam_qy, cam_qz, cam_qw,
        OpaqueFunction(function=_set_tb3_env),
        set_tb3_env,
        tb3_robot,
        realsense,
        static_tf,
        yolo_people_node,
        nav2
    ])
