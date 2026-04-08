#!/usr/bin/env python3
import rclpy, BLD_305s
from rclpy.node import Node
from dcr_interfaces.srv import MotorMovementCommand


class MotorService(Node):

    def __init__(self):
        super().__init__("motor_rs485_service")
        self.motor = BLD_305s.Motor("/dev/ttyACM0", 0)

        self.srv = self.create_service(
            MotorMovementCommand, 
            '/motor_rs485_service/issue_motor_command', 
            self.callback
        )       

    def callback(self, request, response):
        self.get_logger().info(f"Issuing Motor Command: Address: {request.address}, Direction: {request.direction}, Speed: {request.speed}")

        speed = request.speed
        if speed > 3500:
            speed = 3500
        elif speed < 0:
            speed = 0
        
        self.motor.Start(address=request.address, direction=request.direction)
        self.motor.SetSpeed(speed)

        response.result = 1 # TODO: return motor response

        return response

    def stop(self):
        self.motor.BroadcastSTOP()
        self.get_logger().error("Broadcasting STOP signal to all motors")
    

def main(args=None):
    rclpy.init(args=args)
    motorService = MotorService()
    
    try:
        rclpy.spin(motorService)
    except Exception as e:
        motorService.stop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()