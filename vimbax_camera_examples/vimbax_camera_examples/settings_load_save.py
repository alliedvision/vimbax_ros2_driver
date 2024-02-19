import rclpy
from rclpy.node import Node
import vimbax_camera_msgs.srv
import argparse
from .helper import *

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_name")
    parser.add_argument("operation", choices=["load", "save"])
    parser.add_argument("filename")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_settings_load_save")

    service_type = vimbax_camera_msgs.srv.SettingsLoadSave
    
    request = service_type.Request()
    request.filename = args.filename
    response = single_service_call(node,service_type,f"{args.node_name}/settings/{args.operation}",request)

    if response.error == 0:
        print(f"Settings {args.operation} was successfull")
    else:
        print(f"Settings {args.operation} failed with {response.error}")
        

