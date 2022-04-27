#!/usr/bin/env python3
import sys
from typing import Callable
import rclpy
import argparse
from std_msgs.msg import String, Float64
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import subprocess

def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'


class CameraNode(Node):
    def __init__(self, robot):
        super().__init__('camera_node')
        self.name = 'camera_node'
        self.robot = robot
        self.status = 0
        self.declare_parameter('id')
        self.id = self.get_parameter('id').value
        self.declare_parameter('udp/ip')
        self.ip = self.get_parameter('udp/ip').value
        self.proc = None

        # Subscribers
        sub_camera = self.create_subscription(String, 'handle_camera_' + str(self.id), self.handle_camera, 10)
        sub_status_check = self.create_subscription(String, 'status_check', self.status_callback, 10)

        # Publishers
        self.camera_status_publisher = self.create_publisher(String, 'camera_status', 10)

        self.publish_status()

    def status_callback(self, data):
        self.publish_status()
    
    def handle_camera(self, data):
        if data.data.lower() == "start" and self.status == 0:
            print(cl_green("Starting camera"))
            self.proc = subprocess.Popen(["/bin/bash", "kmr_communication/kmr_communication/script/startcamera.sh", self.ip])
            self.status = 1
        elif data.data.lower() == "stop":
            try:
                self.status = 0
                self.proc.terminate()
                self.proc = None
                print(cl_green("Stopping camera"))  
            except AttributeError:
                print(cl_red("Camera was never started, therefore never stopped")) 

        self.publish_status()


    def publish_status(self):
        msg = String()
        msg.data = self.id + ":" + self.robot + ":camera:" + str(self.status) + ":" + str(self.ip) #ip = ip:port
        self.camera_status_publisher.publish(msg)
    
    def tear_down(self):
        try:
            self.destroy_node()
            rclpy.shutdown()
            print(cl_green("Successfully tore down camera node"))
        except:
            print(cl_red('Error: ') + "rclpy shutdown failed")


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    while True:
        rclpy.init(args=argv)
        camera_node = CameraNode(args.robot)
        rclpy.spin(camera_node)
        
if __name__ == '__main__':
    main()