#include <Servo.h>

Servo joint1;
Servo joint2;
Servo joint3;
Servo joint4;
Servo gripper;

String receivedData = "";

int joint1Position = 0;
int joint2Position = 71;
int joint3Position = 36;
int joint4Position = 71;
int gripper_Position = 85;

void setup() {
  // Attach servos to pins
  joint1.attach(9);
  joint2.attach(10);
  joint3.attach(11);
  joint4.attach(6);
  gripper.attach(5);

  // Initialize serial communication
  Serial.begin(9600);

  // Set initial position for joint2
  gripper.write(gripper_Position);
}

void loop() {
  if (Serial.available()) {
    char inChar = Serial.read();
    if (inChar != '\n') {
      receivedData += inChar;
    } else {
      int lastCommaIndex = receivedData.lastIndexOf(',');
      String gripperPosStr = receivedData.substring(lastCommaIndex + 1);
      int gripperPos = gripperPosStr.toInt();

      gripper.write(gripperPos);
      receivedData = "";  // Clear the stored data
    }
  }
}
