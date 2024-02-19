import rclpy
from rclpy.node import Node
import vimbax_camera_msgs.srv
import argparse
from .helper import *

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_name")
    
    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_status_get")

    service_type = vimbax_camera_msgs.srv.Status
    
    request = service_type.Request()
    response = single_service_call(node,service_type,f"{args.node_name}/status",request)

    if response.error == 0:
        print(f"Received status {response}")
    else:
        print(f"Settings {args.operation} failed with {response.error}")
        

