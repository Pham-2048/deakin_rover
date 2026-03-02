import rclpy
from rclpy.node import Node

from arm_interfaces.msg import MotorStat, MotorMove
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

from motor_node.iksolve import IKSolver

class Controller(Node):

    def __init__(self):
        super().__init__('Controller')
        self.declare_parameter('mode', 'fk')
        self.mode = self.get_parameter('mode').value
        self.declare_parameter('rate', 1)
        self.get_logger().info(f"Controller started in {self.mode} mode")

        self.ik_solver = IKSolver()
        # Position (x, y, z)
        self.joints = [0.0, 0.0, 0.0,0.0, 0.0, 0.0]
        self.pos = Point() #for Ik
        # Orientation (yaw, pitch, roll)
        #self.orientation = [0.0, 0.0, 0.0]
        self.motor_move_publisher = self.create_publisher(MotorMove, '/motor_move', 15)
        # Create 6 motor clients
        #self.move_clients = []
        #for i in range(6):
         #   srv_name = f"motor_move_{i+1}"
          #  client = self.create_client(MotorMove, srv_name)
           # self.move_clients.append(client)

        # Subscribe to joystick
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 30)
        #self.latest_msg = Joy()
        #timer_period = self.get_parameter('rate').value
        #self.motor_move_timer = self.create_timer(timer_period, self.publish_motor_move)

    def safe_axis(self, axes, index, default=0.0):
        """Safely access axes array with default value"""
        if index < len(axes):
            value = axes[index]
            # Check for NaN or Inf
            if not (float('-inf') < value < float('inf')):
                return default
            return value
        return default

    def safe_button(self, buttons, index, default=0):
        """Safely access buttons array with default value"""
        return buttons[index] if index < len(buttons) else default

    def joy_callback(self, joy_msg):
        """Process joystick input with defensive error handling"""
        try:
            # Validate minimum message structure
            if not hasattr(joy_msg, 'axes') or not hasattr(joy_msg, 'buttons'):
                self.get_logger().error('Invalid Joy message: missing axes or buttons')
                return

            # IK mode: requires axes[0], [1], [4]
            if (self.get_parameter('mode').value == 'ik'):
                self.pos.x += self.safe_axis(joy_msg.axes, 0) / 500
                self.pos.y += self.safe_axis(joy_msg.axes, 1) / 500
                self.pos.z += self.safe_axis(joy_msg.axes, 4) / 500
                self.joints[0:3] = self.ik_solver.solve(self.pos)

            # FK mode: requires axes[0], [1], [2]
            elif (self.get_parameter('mode').value == 'fk'):
                self.joints[0] = self.safe_axis(joy_msg.axes, 0) * 5
                self.joints[1] = self.safe_axis(joy_msg.axes, 1) * 5
                self.joints[2] = self.safe_axis(joy_msg.axes, 2) * 5

            # Orientation controls: requires axes[3], [6], [7]
            self.joints[3] = self.safe_axis(joy_msg.axes, 3) * 10
            self.joints[4] = self.safe_axis(joy_msg.axes, 6) * 10
            self.joints[5] = self.safe_axis(joy_msg.axes, 7) * 10

            # Button logging (optional)
            if self.safe_button(joy_msg.buttons, 0) == 1:
                self.get_logger().info(
                    f"going to x:{(self.pos.x * 1000):.2f} and y:{(self.pos.y * 1000):.2f} and z:{(self.pos.z * 1000):.2f}"
                )
                self.get_logger().info("sending joints: [" + ", ".join(f"{j:.2f}" for j in self.joints) + "]")

            # Publish motor commands
            self.publish_motor_move()

        except Exception as e:
            self.get_logger().error(f'Error processing Joy message: {e}')
            # Don't crash - just log and continue

        # dx = joy_msg.axes[0]
        # dy = joy_msg.axes[1]
        # dz = joy_msg.axes[2]

        # if dx != 0 or dy != 0 or dz != 0:
        #     self.pos[0] += dx
        #     self.pos[1] += dy
        #     self.pos[2] += dz

        #     # Solve IK
        #     #joints = IkSolver(self.pos)   # return [j1, j2, j3]

        #     # Send to first 3 motors
        #     for i in range(3):
        #         req = MotorMove.Request()
        #         req.angle = self.pos[i] #temp
        #         self.get_logger().info(f"sending motor {i + 1} angle {self.pos[i]}")
        #         self.move_clients[i].call_async(req)

        # # ---------------------------
        # #  2) ORIENTATION (buttons)
        # # ---------------------------
        # orientation_buttons = [
        #     (0, 1),  # yaw:  +btn0, -btn1
        #     (2, 3),  # pitch: +btn2, -btn3
        #     (4, 5)   # roll: +btn4, -btn5
        # ]

        # for j, (btn_plus, btn_minus) in enumerate(orientation_buttons):
        #     delta = joy_msg.buttons[btn_plus] - joy_msg.buttons[btn_minus]
        #     if delta != 0:
        #         self.orientation[j] += delta
        #         req = MotorMove.Request()
        #         req.angle = self.orientation[j]
        #         self.get_logger().info(f"sending motor {j + 4} angle {self.orientation[j]}")
        #         self.move_clients[j+3].call_async(req)  

    def publish_motor_move(self):
        for i in range (6):
            command = MotorMove()
            command.id = i + 1
            command.angle = self.joints[i]
            if (self.get_parameter('mode').value == 'ik' and i < 3):
                command.mode = True
            else:
                #self.get_logger().info(f"Controller sending fk mode")
                command.mode = False
            self.motor_move_publisher.publish(command)

def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()