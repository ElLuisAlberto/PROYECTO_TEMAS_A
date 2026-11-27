"""Local ROS client for the direct bridge output switch."""
import time


class DirectOutputControl:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool, Float64MultiArray, Int32
        from std_srvs.srv import SetBool, Trigger

        self.rclpy = rclpy
        self.service_type = SetBool
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init()
        self.node = Node('direct_output_console')
        self.bool_type = Bool
        self.array_type = Float64MultiArray
        self.int_type = Int32
        self.recovery_client = self.node.create_client(SetBool, '/kinova_direct/recovery_mode')
        self.recovery_pub = self.node.create_publisher(Float64MultiArray, '/kinova_direct/recovery_velocity', 10)
        self.heartbeat_pub = self.node.create_publisher(Bool, '/kinova_direct/recovery_heartbeat', 10)
        self.game_reset_pub = self.node.create_publisher(Bool, '/teleop/game_reset', 10)
        self.object_reset_pub = self.node.create_publisher(Int32, '/teleop/game_reset_object', 10)
        self.node.create_subscription(Bool, '/kinova_direct/recovery_active', self._on_recovery, 10)
        self.recovery_active = None
        self.recovery_updated = 0.0
        self.recovery_future = None
        self.recovery_returning = False
        self.recovery_message = ''
        self.client = self.node.create_client(SetBool, '/kinova_direct/enable_output')
        self.node.create_subscription(Bool, '/kinova_direct/output_enabled', self._on_state, 10)
        self.node.create_subscription(Bool, '/kinova_direct/usb_connected', self._on_usb, 10)
        self.reconnect_client = self.node.create_client(Trigger, '/kinova_direct/reconnect_usb')
        self.disconnect_client = self.node.create_client(Trigger, '/kinova_direct/disconnect_usb')
        self.shutdown_client = self.node.create_client(Trigger, '/kinova_direct/prepare_shutdown')
        self.shutdown_future = None
        self.shutdown_result = None
        self.shutdown_message = ''
        self.disconnect_future = None
        self.reconnect_type = Trigger
        self.usb_connected = None
        self.usb_updated = 0.0
        self.reconnect_future = None
        self.reconnect_at = 0.0
        self.reconnect_message = ''
        self.enabled = None
        self.updated = 0.0
        self.future = None
        self.message = ''

    def _on_recovery(self, msg):
        self.recovery_active = bool(msg.data)
        if not self.recovery_active and self.recovery_returning:
            self.recovery_returning = False
            self.recovery_message = 'Control web habilitado'
        self.recovery_updated = time.monotonic()

    def recovery_state(self):
        return self.recovery_active if time.monotonic() - self.recovery_updated < 0.5 else None

    def set_recovery(self, active):
        if self.recovery_future is not None or not self.recovery_client.service_is_ready():
            self.recovery_message = 'Servicio de recuperación no disponible o solicitud pendiente'
            return
        request = self.service_type.Request()
        request.data = bool(active)
        if not active:
            self.recovery_returning = True
        self.recovery_future = self.recovery_client.call_async(request)
        self.recovery_message = 'Cambiando modo…'

    def reset_game(self):
        msg = self.bool_type()
        msg.data = True
        self.game_reset_pub.publish(msg)

    def reset_object(self, index):
        msg = self.int_type()
        msg.data = index
        self.object_reset_pub.publish(msg)

    def heartbeat(self):
        msg = self.bool_type()
        msg.data = True
        self.heartbeat_pub.publish(msg)

    def joint_velocity(self, values):
        msg = self.array_type()
        msg.data = list(values)
        self.recovery_pub.publish(msg)

    def _on_usb(self, msg):
        self.usb_connected = bool(msg.data)
        self.usb_updated = time.monotonic()

    def _on_state(self, msg):
        if self.enabled is not None and self.enabled != bool(msg.data):
            self.message = ''
        self.enabled = bool(msg.data)
        self.updated = time.monotonic()

    def poll(self):
        if not self.rclpy.ok():
            return None
        # Drain both 20 Hz status streams and service replies on every GUI tick.
        for _ in range(6):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                self.message = result.message if result.success else 'Error: ' + result.message
            except Exception as error:
                self.message = 'Error: ' + str(error)
            self.future = None
        if self.reconnect_future is not None and self.reconnect_future.done():
            try:
                self.reconnect_message = self.reconnect_future.result().message
            except Exception as error:
                self.reconnect_message = 'Error: ' + str(error)
            self.reconnect_future = None
        if self.disconnect_future is not None and self.disconnect_future.done():
            try:
                result = self.disconnect_future.result()
                self.reconnect_message = result.message if result.success else 'Error: ' + result.message
            except Exception as error:
                self.reconnect_message = 'Error: ' + str(error)
            self.disconnect_future = None
        if self.recovery_future is not None and self.recovery_future.done():
            try:
                result = self.recovery_future.result()
                self.recovery_message = result.message if result.success else 'Error: ' + result.message
                if not result.success:
                    self.recovery_returning = False
            except Exception as error:
                self.recovery_returning = False
                self.recovery_message = 'Error: ' + str(error)
            self.recovery_future = None
        if self.shutdown_future is not None and self.shutdown_future.done():
            try:
                result = self.shutdown_future.result()
                self.shutdown_result = bool(result.success)
                self.shutdown_message = result.message
            except Exception as error:
                self.shutdown_result = False
                self.shutdown_message = 'Error: ' + str(error)
            self.shutdown_future = None
        return self.enabled if time.monotonic() - self.updated < 0.5 else None

    def request(self, enabled):
        if self.future is not None or not self.client.service_is_ready():
            self.message = 'Servicio de armado no disponible o solicitud pendiente'
            return
        request = self.service_type.Request()
        request.data = bool(enabled)
        self.future = self.client.call_async(request)
        self.message = 'Solicitud en curso…'

    def usb_state(self):
        return self.usb_connected if time.monotonic() - self.usb_updated < 0.5 else None

    def reconnect(self):
        now = time.monotonic()
        if self.usb_state() is not False or self.reconnect_future is not None or self.disconnect_future is not None or now - self.reconnect_at < 1.0:
            return
        if not self.reconnect_client.service_is_ready():
            self.reconnect_message = 'Servicio de reconexión no disponible'
            return
        self.reconnect_at = now
        self.reconnect_future = self.reconnect_client.call_async(self.reconnect_type.Request())
        self.reconnect_message = 'Intentando conectar…'

    def disconnect(self):
        if self.usb_state() is not True or self.disconnect_future is not None or self.reconnect_future is not None:
            return
        if not self.disconnect_client.service_is_ready():
            self.reconnect_message = 'Servicio de desconexión no disponible'
            return
        self.disconnect_future = self.disconnect_client.call_async(self.reconnect_type.Request())
        self.reconnect_message = 'Deteniendo y cerrando conexión USB…'

    def prepare_shutdown(self):
        if self.shutdown_future is not None:
            return False
        if not self.shutdown_client.service_is_ready():
            self.shutdown_result = False
            self.shutdown_message = 'Servicio de cierre no disponible; revise el bridge'
            return False
        self.shutdown_result = None
        self.shutdown_message = 'Deteniendo la salida y cerrando el USB…'
        self.shutdown_future = self.shutdown_client.call_async(self.reconnect_type.Request())
        return True

    def close(self):
        if self.rclpy.ok():
            self.node.destroy_node()
            if self.owns_context:
                self.rclpy.shutdown()
