# Belum dicoba

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

from ament_index_python.packages import get_package_share_directory

path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"

rtabmap_params = [
            {
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "subscribe_depth": True,
                "subscribe_rgb": False,
                "subscribe_scan": False,
                "subscribe_scan_cloud": False,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_odom": False,
                "subscribe_odom_info": False,
                "qos_scan": 1,
                "wait_for_transform": 2.0,
                "odom_topic": "",

                # "odom_tf_linear_variance": 0.0001,
                # "odom_tf_angular_variance": 0.0001,
                "publish_tf": True,
                "publish_map": True,
                "approx_sync": True,
                "use_saved_map": False,
                "sync_queue_size": 30,
                "topic_queue_size": 10,
                "visual_odometry": True,
                "icp_odometry": False,

                "Rtabmap/DetectionRate": "5.0", # Added by Azzam
                "Rtabmap/CreateIntermediateNodes": "True",
                "Rtabmap/LoopThr": "0.11", # Routine period untuk cek loop closure

                "Mem/STMSize": "50",  # Short-term memory size
                "Mem/IncrementalMemory": "True",  # 
                "Mem/InitWMWithAllNodes": "True",  # 
                "Mem/RehearsalSimilaritys": "0.9",  #
                "Mem/UseOdomFeatures": "True",  #
                'Mem/NotLinkedNodesKept': "False",  # Keep unlinked nodes
                # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure

                "Kp/MaxFeatures": "2000",

                "RGBD/Enabled": "True",
                "RGBD/OptimizeFromGraphEnd": "False", # True agar robot tidak lompat 
                "RGBD/NeighborLinkRefining": "True",  # Added from documentation
                "RGBD/AngularUpdate": "0.01",  # Added from documentation
                "RGBD/LinearUpdate": "0.01",  # Added from documentation
                "RGBD/OptimizeMaxError": "3.0",  # Added from documentation
                "RGBD/InvertedReg": "False",  # Added from documentation
                "RGBD/ProximityPathMaxNeighbors": "20",
                "RGBD/ProximityMaxGraphDepth": "50",
                "RGBD/ProximityByTime": "False",
                "RGBD/ProximityBySpace": "True",  # Added from documentation

                "Optimizer/Strategy": "2",  # Added by Azzam
                "Optimizer/Iterations": "100",  # Added by Azzam
                "Optimizer/Epsilon": "0.00001",  # Added by Azzam
                "Optimizer/Robust": "False",  # Added by Azzam
                'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
                "Optimizer/VarianceIgnored": "True",
                "GTSAM/Incremental": "True",

                "Bayes/PredictionMargin": "0", # Added by Azzam
                "Bayes/FullPredictionUpdate": "False", # Added by Azzam
                "Bayes/PredictionLC": "0.1", # Added by Azzam

                "Odom/Strategy": "3",  # Added by Azzam
                "Odom/ResetCountdown": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
                "Odom/AlignWithGround": "True",  # Added by Azzam

                "Reg/Strategy": "0",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam

                "Icp/Strategy": "1",  # Added by Azzam
                "Icp/MaxTranslation": "1.0", # Added by Azzam
                "Icp/MaxRotation": "0.1", # Added by Azzam
                "Icp/RangeMin": "0.0", # Added by Azzam
                "Icp/RangeMax": "25.0", # Added by Azzam
                "Icp/MaxCorrespondenceDistance": "1.0", # Added by Azzam
                "Icp/Iterations": "30", # Added by Azzam
                "Icp/PointToPlane": "True", # Added by Azzam
                "Icp/VoxelSize": "0.05", # Added by Azzam
                'Icp/PointToPlaneMinComplexity':'0.23', # to be more robust to long corridors with low geometry
                'Icp/PointToPlaneLowComplexityStrategy':'1', # to be more robust to long corridors with low geometry

                "Vis/MaxDepth": "20.0",
                "Vis/MinInliers": "20",

                "Grid/Sensor": "2",  # Added to suppress warning
                "Grid/RangeMin": "0.0",  # Added by Azzam
                "Grid/RangeMax": "100.0",  # Added by Azzam
                'Grid/UpdateRate': "1.0",  # Update map every 1 second (default is often higher)
                'Grid/CellSize': "0.1",  # Increase cell size to reduce map density
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/IncrementalMapping": "True",  # Added from documentation
                "Grid/Scan2dUnknownSpaceFilled": "False",  # Added by Azzam
                "GridGlobal/UpdateError": "0.04", # Added by Azzam
                "Grid/RayTracing": "False", # Added by Azzam

                "use_sim_time": False,
                "Threads": 10, # Added by Azzam
            }
        ]

def generate_launch_description():
    
    # SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
    # SetEnvironmentVariable(name='CYCLONEDDS_URI', value='file://' + path_config + 'cyclonedds.xml'),

    tf_base_link_to_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_camera_link",
        # fmt: off
        arguments=["0.00","0.00","0.00","0.00","0.06","0.00","base_link","camera_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    rs2_cam_main = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="rs2_cam_main",
        parameters=[
            {
                "camera_name": "camera",
                "camera_namespace": "",
                "enable_accel": False,
                "enable_gyro": False,
                "enable_depth": True,
                "enable_color": True,
                "enable_sync": True,
                "unite_imu_method": 2,
                "align_depth.enable": True,
                "pointcloud.enable": False,
                "rgb_camera.profile": "640x360x30",
            }
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
        prefix='nice -n -20 chrt -f 96',
    )

    rtabmap_slam_rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        parameters=rtabmap_params,
        remappings=[
            ('rgb/image', '/camera/rs2_cam_main/color/image_raw'),
            ('rgb/camera_info', '/camera/rs2_cam_main/color/camera_info'),
            ('depth/image', '/camera/rs2_cam_main/aligned_depth_to_color/image_raw')
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap",
        parameters=rtabmap_params,
        remappings=[
            ('rgb/image', '/camera/rs2_cam_main/color/image_raw'),
            ('rgb/camera_info', '/camera/rs2_cam_main/color/camera_info'),
            ('depth/image', '/camera/rs2_cam_main/aligned_depth_to_color/image_raw')
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )


    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # fmt: off
        arguments=["-d",os.path.join(path_config,"robot.rviz"),
                   "--ros-args","--log-level","error",]
        # fmt: on
    )



    return LaunchDescription(
        [
            rs2_cam_main,
            rtabmap_slam_rtabmap,
            rviz2,
            # rtabmap_viz
        ]
    )
