#!/usr/bin/python3

import os
import sys
import time
from datetime import datetime 
import psutil

import influxdb_client
import rclpy
from rclpy.node import Node

from influxdb_client import Point 
from influxdb_client.client.write_api import SYNCHRONOUS
from loguru import logger

from io import StringIO
from threading import Lock
import numpy as np

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16

class Telemetry(Node):
    def __init__(self):
        self.has_init = False
        self.lock = Lock()

        super().__init__("telemetry")

        # Get config from ROS2 parameters
        # -------------------------------
        self.get_logger().info("Getting config from ROS2 parameters")
        self.declare_parameter("INFLUXDB_URL", "http://x.x.x.x:8086")
        self.declare_parameter("INFLUXDB_USERNAME", "x")
        self.declare_parameter("INFLUXDB_PASSWORD", "xx")
        self.declare_parameter("INFLUXDB_ORG", "xxx")
        self.declare_parameter("INFLUXDB_BUCKET", "xxxx")
        self.declare_parameter("ROBOT_NAME", "xxxxx")
        self.declare_parameter("publish_period", 10) # in ms

        self.INFLUXDB_URL = self.get_parameter("INFLUXDB_URL").get_parameter_value().string_value
        self.INFLUXDB_USERNAME = self.get_parameter("INFLUXDB_USERNAME").get_parameter_value().string_value
        self.INFLUXDB_PASSWORD = self.get_parameter("INFLUXDB_PASSWORD").get_parameter_value().string_value
        self.INFLUXDB_ORG = self.get_parameter("INFLUXDB_ORG").get_parameter_value().string_value
        self.INFLUXDB_BUCKET = self.get_parameter("INFLUXDB_BUCKET").get_parameter_value().string_value
        self.ROBOT_NAME = self.get_parameter("ROBOT_NAME").get_parameter_value().string_value
        self.publish_period = self.get_parameter("publish_period").get_parameter_value().integer_value

        # Subscriber
        # ----------
        self.sub_distance_travelled = self.create_subscription(Float32, "/distance_travelled", self.callback_sub_distance_travelled, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.callback_sub_odom, 10)
        self.sub_battery = self.create_subscription(Int16, "/can/battery", self.callback_sub_battery, 10)

        # ROS2 Timer
        # ----------
        self.timer_routine = self.create_timer(self.publish_period, self.timer_routine_callback)

        # Logger
        # ------
        logger.remove()
        logger.add(
            sys.stdout,
            colorize=True,
            format="<green>{time:HH:mm:ss.SSS}</green> | <level>{level:^6}</level> | <cyan>{function}</cyan>:<cyan>{line}</cyan> - {message}",
            enqueue=True,
        )

        logger.info("Node Telemetry node has been initialized")
        logger.info("INFLUXDB_URL: {}".format(self.INFLUXDB_URL))
        logger.info("INFLUXDB_USERNAME: {}".format(self.INFLUXDB_USERNAME))
        logger.info("INFLUXDB_PASSWORD: {}".format(self.INFLUXDB_PASSWORD))
        logger.info("INFLUXDB_ORG: {}".format(self.INFLUXDB_ORG))
        logger.info("INFLUXDB_BUCKET: {}".format(self.INFLUXDB_BUCKET)) 
        logger.info("ROBOT_NAME: {}".format(self.ROBOT_NAME))
        logger.info("publish_period: {}".format(self.publish_period))

        logger.info("Attempting to connect to InfluxDB...")

        # Database
        # --------
        try:
            self.db_client = influxdb_client.InfluxDBClient(
                url=self.INFLUXDB_URL,
                org=self.INFLUXDB_ORG,
                username=self.INFLUXDB_USERNAME,
                password=self.INFLUXDB_PASSWORD,
                timeout=30000,
                read_timeout=30000,
                query_timeout=30000,
            )
            self.db_write_api = self.db_client.write_api(write_options=SYNCHRONOUS)
        except Exception as e:
            logger.error(e)
            sys.exit()

        self.has_init = True
        self.error_counter = 0
        self.time_robot_started = datetime.utcnow()
        self.distance_travelled = 0.0
        self.prev_distance_travelled = 0.0
        self.battery = 0.0
        self.odom = Odometry()
        self.cpu_usage_percent = 0.0
        self.ram_usage_percent = 0.0
        
        logger.info("Connected to InfluxDB")

    def write_once(self, fields, values):
        try:
            # Start with the base point (measurement name)
            point = Point(self.ROBOT_NAME).tag("robot_name", self.ROBOT_NAME)

            # Add fields dynamically from the lists
            for field, value in zip(fields, values):
                point = point.field(field, value)
            
            point = point.time(self.time_robot_started)

            self.db_write_api.write(bucket=self.INFLUXDB_BUCKET, org=self.INFLUXDB_ORG, record=point)
        except Exception as e:
            logger.error(e)
            self.error_counter += 1
    
    def write_sequently(self, fields, values):
        try:
            # Start with the base point (measurement name)
            point = Point(self.ROBOT_NAME).tag("robot_name", self.ROBOT_NAME)

            # Add fields dynamically from the lists
            for field, value in zip(fields, values):
                point = point.field(field, value)
            
            point = point.time(datetime.utcnow())

            self.db_write_api.write(bucket=self.INFLUXDB_BUCKET, org=self.INFLUXDB_ORG, record=point)
        except Exception as e:
            logger.error(e)
            self.error_counter += 1
    
    # ======================================================================================= 

    def callback_sub_distance_travelled(self, msg):
        self.lock.acquire()
        self.distance_travelled = msg.data
        self.lock.release()
    
    def callback_sub_battery(self, msg):
        self.lock.acquire()
        self.battery = float(msg.data)
        self.lock.release()
    
    def callback_sub_odom(self, msg):
        self.lock.acquire()
        self.odom = msg
        self.lock.release()
    
    def timer_routine_callback(self):
        if not self.has_init:
            return
        
        self.cpu_usage_percent = psutil.cpu_percent(interval=1)
        self.ram_usage_percent = psutil.virtual_memory().percent
        
        odom_pose_x = self.odom.pose.pose.position.x
        odom_pose_y = self.odom.pose.pose.position.y
        odom_pose_theta = np.arctan2(2*(self.odom.pose.pose.orientation.w*self.odom.pose.pose.orientation.z), 1-2*(self.odom.pose.pose.orientation.z**2))

        delta_distance_travelled = self.distance_travelled - self.prev_distance_travelled

        self.lock.acquire()
        self.write_sequently(["battery", "pose_x", "pose_y", "pose_theta", "cpu_usage", "ram_usage", "current_distance_travalled"], [self.battery, odom_pose_x, odom_pose_y, odom_pose_theta, self.cpu_usage_percent, self.ram_usage_percent, delta_distance_travelled])
        self.write_once(["distance_travelled"], [self.distance_travelled])
        self.lock.release()

        self.prev_distance_travelled = self.distance_travelled
        
def main(args=None):
    rclpy.init(args=args)

    node_telemetry = Telemetry()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_telemetry)
    executor.spin()


if __name__ == "__main__":
    main(sys.argv)