#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
from datetime import datetime

class ArduinoComms(Node):
    def __init__(self):
        super().__init__("arduino_comms")
        #subscribe to servo_positions node
        self.servo_command_subscriber = self.create_subscription(
            Int32MultiArray, "servo_positions", self.serial_callback, 10)
        
        #setup serial
        self.ser = serial.Serial('/dev/ttyACM0', 9600)

        self.get_logger().info("ArduinoComms node is started.")

    def serial_callback(self, msg):
        #Convert the integer array to a comma-separated string and add a newline
        data_str = ",".join(map(str, msg.data)) + "\n"
        #Write the data to the serial port
        self.ser.write(data_str.encode())
        print(f"Callback called at {datetime.now().strftime('%H:%M:%S.%f')}")


    def on_exit(self):
        self.ser.close()

def main(args=None):    
    rclpy.init(args=args)
    node = ArduinoComms()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Handle other exceptions
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.on_exit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()