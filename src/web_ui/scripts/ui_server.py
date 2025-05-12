#!/usr/bin/python3

import subprocess
import os
import signal
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

http_server_process = None

class HttpServerNode(Node):
    def __init__(self):
        super().__init__("http_server_node")
        self.http_server_process = None

        self.declare_parameter("ui_root_path", "")

        # Signal handling for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        # Start the HTTP server
        self.start_http_server()

    def start_http_server(self):
        global http_server_process
        try:
            # Get the package path
            ui_root_path = self.get_parameter("ui_root_path").get_parameter_value().string_value
            os.chdir(ui_root_path)

            # Run the python -m http.server command as a subprocess
            self.http_server_process = subprocess.Popen(["python3", "-m", "http.server"])
            self.get_logger().info("HTTP server started.")

            # Wait for the subprocess to complete
            self.http_server_process.wait()
        except Exception as e:
            self.get_logger().error(f"Error starting HTTP server: {e}")

    def signal_handler(self, signum, frame):
        global http_server_process
        if self.http_server_process is not None:
            self.get_logger().info("Terminating HTTP server.")
            self.http_server_process.terminate()
            self.http_server_process = None

def main(args=None):
    rclpy.init(args=args)

    try:
        node = HttpServerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        rclpy.shutdown()
        if node.http_server_process:
            node.http_server_process.terminate()

if __name__ == "__main__":
    main()
