import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from px4_msgs.msg import ManualControlSetpoint, VehicleCommand, OffboardControlMode

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            durability=DurabilityPolicy.VOLATILE, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )

        # PX4 Yayıncıları
        self.cmd_pub = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', qos)
        self.off_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.v_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        
        # Vision Subscriber
        self.target_sub = self.create_subscription(Point, '/vision/target_info', self.target_callback, 10)
        
        # Değişkenler
        self.counter = 0
        self.target_visible = False
        self.target_x = 0.0
        self.last_steer = 0.5
        
        # Ana Döngü (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Controller Node Hazir!")

    def target_callback(self, msg):
        # Gelen veriyi değişkenlere aktar
        if msg.z == 1.0:
            self.target_visible = True
            self.target_x = msg.x
        else:
            self.target_visible = False

    def send_cmd(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.v_cmd_pub.publish(msg)

    def timer_callback(self):
        # PX4 Başlatma Prosedürü (Arm & Offboard)
        if self.counter < 10:
            self.counter += 1
        elif self.counter == 10:
            self.send_cmd(400, 1.0) # ARM
            self.send_cmd(176, 1.0, 6.0) # OFFBOARD
            self.counter += 1
        
        # Offboard Heartbeat
        off_msg = OffboardControlMode()
        off_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        off_msg.position = False
        off_msg.velocity = False
        off_msg.acceleration = False
        off_msg.attitude = False
        off_msg.body_rate = False
        self.off_pub.publish(off_msg)

        # Hareket Mesajı Hazırlama
        ctrl_msg = ManualControlSetpoint()
        ctrl_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ctrl_msg.valid = True
        ctrl_msg.data_source = 2

        if self.target_visible:
            error = self.target_x
            ctrl_msg.roll = float(error) # Direksiyon (Kp=1.0)
            
            if abs(error) < 0.15: # Merkezdeyse yardır
                ctrl_msg.throttle = 0.8
                self.get_logger().info("SALDIR!")
            else: # Merkezleniyorsa yavaş ilerle
                ctrl_msg.throttle = 0.2
                self.get_logger().info(f"Hizalanıyor: {error:.2f}")
            
            # Hafıza: En son görülen yön
            self.last_steer = 0.6 if error > 0 else -0.6
        else:
            # Hedef yoksa arama yap
            ctrl_msg.throttle = 0.0
            ctrl_msg.roll = float(self.last_steer)
            self.get_logger().info("Hedef yok, araniyor...")

        self.cmd_pub.publish(ctrl_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
