import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"name": "local_xy_frame", "value": "map"},
                {"name": "local_xy_origin", "value": "swri"},
                {"name": "local_xy_origins", "value": """[
                    {"name": "swri",
                        "latitude": -6.139845,
                        "longitude": 106.725369,
                        "altitude": 233.719,
                        "heading": 0.0},
                    {"name": "back_40",
                        "latitude": -6.139845,
                        "longitude": 106.725369,
                        "altitude": 200.0,
                        "heading": 0.0}
                ]"""}
            ]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        )
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "wgs84"]
        )
    ])
