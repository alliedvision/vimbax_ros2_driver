import rclpy
from rclpy.node import Node
import vimbax_camera_msgs.srv
import argparse
from .helper import single_service_call


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_name")
    parser.add_argument("feature_name")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_feature_command_execute")

    feature_service_type = vimbax_camera_msgs.srv.FeatureCommandRun

    request = feature_service_type.Request()
    request.feature_name = args.feature_name
    response = single_service_call(node, feature_service_type,
                                   f"{args.node_name}/features/command_run", request)

    if response.error == 0:
        print(f"Successfully executed command {args.feature_name}")
    else:
        print(f"Gettings feature {args.feature_name} value failed with {response.error}")
