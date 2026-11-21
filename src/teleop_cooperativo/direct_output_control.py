"""Local ROS client for the direct bridge output switch."""
import time


class DirectOutputControl:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool
        from std_srvs.srv import SetBool

        self.rclpy = rclpy
        self.service_type = SetBool
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init()
        self.node = Node('direct_output_console')
        self.client = self.node.create_client(SetBool, '/kinova_direct/enable_output')
        self.node.create_subscription(Bool, '/kinova_direct/output_enabled', self._on_state, 10)
        self.enabled = None
        self.updated = 0.0
        self.future = None
        self.message = ''

    def _on_state(self, msg):
        self.enabled = bool(msg.data)
        self.updated = time.monotonic()

    def poll(self):
        if not self.rclpy.ok():
            return None
        self.rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                self.message = result.message if result.success else 'Error: ' + result.message
            except Exception as error:
                self.message = 'Error: ' + str(error)
            self.future = None
        return self.enabled if time.monotonic() - self.updated < 0.5 else None

    def request(self, enabled):
        if self.future is not None or not self.client.service_is_ready():
            self.message = 'Servicio de armado no disponible o solicitud pendiente'
            return
        request = self.service_type.Request()
        request.data = bool(enabled)
        self.future = self.client.call_async(request)
        self.message = 'Solicitud en curso…'

    def close(self):
        if self.rclpy.ok():
            self.node.destroy_node()
            if self.owns_context:
                self.rclpy.shutdown()
