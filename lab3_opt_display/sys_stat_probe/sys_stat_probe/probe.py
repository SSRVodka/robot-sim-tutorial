
import rclpy
from rclpy.node import Node

from sys_stat_if.msg import SystemStat

import psutil
import platform


class SystemStatPublisher(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Lab3 TODO
        # self._pub = None
        self._timer = self.create_timer(1, self.on_timer_timeout)

    def get_msg_pkt(self) -> SystemStat:

        mem = psutil.virtual_memory()
        net = psutil.net_io_counters()

        sys_stat = SystemStat()
        sys_stat.stamp = self.get_clock().now().to_msg()
        sys_stat.hostname = platform.node()
        sys_stat.cpu_percent = psutil.cpu_percent()
        sys_stat.mem_percent = mem.percent
        sys_stat.mem_total = mem.total / (1024*1024)
        sys_stat.mem_available = mem.available / (1024*1024)
        sys_stat.net_sent = net.bytes_sent / (1024)
        sys_stat.net_recv = net.bytes_recv / (1024)
        sys_stat.disk_percent = psutil.disk_usage("/").percent

        self.get_logger().info(
            "%s's sys stat: CPU %.2f%%, MEM %.2f%%, DISK %.2f%%" % (
            sys_stat.hostname, sys_stat.cpu_percent, sys_stat.mem_percent, sys_stat.disk_percent))

        return sys_stat

    def on_timer_timeout(self):
        self._pub.publish(self.get_msg_pkt())


def main():
    rclpy.init()
    _node_name = "sys_stat_prob_node"
    pub = SystemStatPublisher(_node_name)
    _logger = pub.get_logger()
    _logger.info(f"{_node_name} initialized")
    rclpy.spin(pub)

    _logger.info(f"{_node_name} is shutting down")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

