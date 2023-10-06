#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from datetime import datetime

class servo_controller(Node):
    def __init__(self):
        super().__init__("servo_controller")
        #subsribe to joy node
        self.ps4_subscriber = self.create_subscription(
            Joy, "joy", self.ps4_callback, 10)
        #create publisher for servos
        self.servo_publisher = self.create_publisher(
            Int32MultiArray, 'servo_positions', 10)
        
        
        #initially assign control of servos 2 and 3
        self.left_servo = 2
        self.right_servo = 3

        #min and max positions for each servo
        self.gripper_closed = 134
        self.gripper_opened = 85
        self.servo1_pos_min = 0
        self.servo1_pos_max = 180
        self.servo2_pos_min = 0
        self.servo2_pos_max = 175
        self.servo3_pos_min = 0
        self.servo3_pos_max = 165
        self.servo4_pos_min = 0
        self.servo4_pos_max = 175

        #setting intial servo positions so that arm is fulling extended up with open gripper
        self.gripper_position = self.gripper_opened
        self.servo1_position = 0
        self.servo2_position = 71
        self.servo3_position = 36
        self.servo4_position = 71

        self.servo_positions = [self.servo1_position, self.servo2_position, 
                                self.servo3_position, self.servo4_position]
        self.servo_min = [self.servo1_pos_min, self.servo2_pos_min,
                          self.servo3_pos_min, self.servo4_pos_max]
        self.servo_max = [self.servo1_pos_max, self.servo2_pos_max,
                          self.servo3_pos_max, self.servo4_pos_max]
    
        self.current_joy_msg = None
        
        #timer to update servo positions at 5Hz
        self.create_timer(0.2, self.update_servo_positions)

    def ps4_callback(self, msg):
        self.current_joy_msg = msg
        x = msg.buttons[0]
        o = msg.buttons[1]
        triangle = msg.buttons[2]
        square = msg.buttons[3]
        R2 = msg.buttons[7]
        LR_pad = msg.axes[6]
        UD_pad = msg.axes[7]

        self.right_stick([x, o, triangle, square])
        self.left_stick(LR_pad, UD_pad)
        self.gripper(R2)
        print(f"Callback called at {datetime.now().strftime('%H:%M:%S.%f')}")
    
    def right_stick(self, right_assignment):
        servos = [1, 2, 3, 4]

        if 1 in right_assignment:
            potential_servo = servos[right_assignment.index(1)]
            
            if potential_servo != self.left_servo:
                self.right_servo = potential_servo

    def left_stick(self, LR_pad, UD_pad):
        if (LR_pad == 0) and (UD_pad == 0):
            return
        else:
            potential_servo = None

            if LR_pad == -1:
                potential_servo = 2
            elif LR_pad == 1:
                potential_servo = 4
            elif UD_pad == 1:
                potential_servo = 3
            elif UD_pad == -1:
                potential_servo = 1
            
            if potential_servo and potential_servo != self.right_servo:
                self.left_servo = potential_servo

    def gripper(self, R2):
        if R2 == 1:
            if self.gripper_position == self.gripper_opened:
                self.gripper_position = self.gripper_closed

            else:
                self.gripper_position = self.gripper_opened
        else:
            return
        
    def publish_servo_positions(self):
        positions_msg = Int32MultiArray()
        positions_msg.data = [self.servo1_position,
                              self.servo2_position,
                              self.servo3_position,
                              self.servo4_position,
                              self.gripper_position]
        self.servo_publisher.publish(positions_msg)
        
    def update_servo_positions(self):
        if self.current_joy_msg is not None:

            #left servo
            if self.current_joy_msg.axes[1] > .05: #positive left stick with buffer
                self.servo_positions[self.left_servo - 1] += 1
            elif self.current_joy_msg.axes[1] < -.05: #negative left stick with buffer
                self.servo_positions[self.left_servo - 1] -= 1

            #make sure value is within servo bounds
            self.servo_positions[self.left_servo - 1] = self.clamp(
                self.servo_positions[self.left_servo - 1], 
                self.servo_min[self.left_servo - 1], 
                self.servo_max[self.left_servo - 1])

            #right servo
            if self.current_joy_msg.axes[4] > .05: #positive right stick with buffer
                self.servo_positions[self.right_servo - 1] += 1
            elif self.current_joy_msg.axes[4] < -.05: #negative right stick with buffer
                self.servo_positions[self.right_servo - 1] -= 1

            #make sure value is within servo bounds
            self.servo_positions[self.right_servo - 1] = self.clamp(
                self.servo_positions[self.right_servo - 1], 
                self.servo_min[self.right_servo - 1], 
                self.servo_max[self.right_servo - 1])
            
            #publish positions
            self.publish_servo_positions()


    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    
def main(args=None):
    rclpy.init(args=args)
    node = servo_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()