import rclpy
from rclpy.node import Node
import argparse
from .helper import single_service_call, feature_type_dict, print_feature_info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_name")
    parser.add_argument("feature_type", choices=['Int', 'Float', 'String', 'Raw', 'Bool', 'Enum'])
    parser.add_argument("feature_name")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_feature_get")

    feature_type = feature_type_dict[args.feature_type]
    feature_service_type = feature_type.info_service_type

    if feature_service_type is None:
        print(f"Feature type {args.feature_type} does not support info query")
        exit(1)

    request = feature_service_type.Request()
    request.feature_name = args.feature_name
    response = single_service_call(
        node, feature_service_type,
        f"{args.node_name}/{feature_type.service_base_path}_info_get", request)

    if response.error == 0:
        print_feature_info(response)
    else:
        print(f"Gettings feature {args.feature_name} value failed with {response.error}")
