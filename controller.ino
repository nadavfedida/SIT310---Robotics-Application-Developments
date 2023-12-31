//   SIT 310
// Robotics Application Development
//  Task 2.4HD
// Nadav Fedida Student ID: 220548228 Date: April 9, 2023

// In writing, explain the kinematic equations you have used, document which parts of the code you changed. If you have tackled the second part of the task (correct velocity units), list some reasons as to why the robot may not reach exactly 1 meter.
// As you can see in the supplied code below, I used the following kinematic equation :
//  LEFT Wheel Speed =
// RIGHT Wheel Speed =
//  Angular Velocity×Rover Width Linear Velocity − 2
// !
// !
//  Wheel Circumference × Conversion Factor  Angular Velocity×Rover Width
//  Linear Velocity + 2
// Wheel Circumference × Conversion Factor
//  The above formulas separate the linear and angular velocities to the two wheels respectively, allowing the robot to turn smoothly.
// The original formula did not account for the conversion factor that I introduced. This is to achieve the 1m distance travelled in 4 seconds when given 0.25m/s2.
// In order to calibrate the linear velocity correctly, I did some trial and error to get as close as i can to the 1m mark.
// I have used this formula as it takes into account the robot width, and wheel circumference which helped it get a more accurate conversion to the small scale rover that we are using.
// A reason why it might not reach exactly 1m is because it is running on a timer rather than revolutions taken. If this was a servo motor, we could command it to spin exactly X amount of times in order to have exact repeatability. However, since we are using a time factor, it is far less accurate and less repeatable.
// video
// This is my submission for 2.4HD Enjoy the clip.
// https://youtu.be/kqN-9CTEBew
// controller
//  1 2 3 4 5 6 7 8 9
// 10
// 11
// 12
// 13
// 14
// 15
// 16
// 17
// 18
// 19
// 20
// 21
// 22
// 23
// 24
// 25
// import rclpy
// from rclpy.node import Node
// from std_msgs.msg import String from geometry_msgs.msg import Twist import serial
// from pynput import keyboard import getpass
// USB_PORT = ’/dev/ttyACM6’ FACTOR = 200
// class KeyboardController(Node):
// def __init__(self):
// super().__init__(’controller’)
// self.subscription = self.create_subscription(String, ’/remote/rover_cmd’,
// self.listener_callback , 10) self.subscription
// self.ser = serial.Serial(USB_PORT)
// def __del__(self): self.ser.close()
// def listener_callback(self, msg):
// 1

// 26 cmd = msg.data.split(’,’)
// 27 linear_vel = float(cmd[0])
// 28 angular_vel = float(cmd[1])
// 29
// 30 self.inverse_kinematics(linear_vel ,
// 31
// 32
// 33 def inverse_kinematics(self, linear_vel, angular_vel): 34
// 35 rover_width = 0.4
// 36 wheel_radius = 0.07
// 37 wheel_circumference = 2 * 3.14159265359 * wheel_radius
// 38
// 39 # Calculate wheel speeds in RPM
// 40 linear_vel_left = linear_vel - (angular_vel * rover_width / 2)
// 41 linear_vel_right = linear_vel + (angular_vel * rover_width / 2)
// 42 wheel_speed_left = round(linear_vel_left / wheel_circumference*FACTOR)
// 43 wheel_speed_right = round(linear_vel_right / wheel_circumference*FACTOR)
// 44
// 45 command = f"_ANGL:{wheel_speed_left},{wheel_speed_right}\n"
// 46 self.ser.write(command.encode())
// 47
// 48 #for printing purposes only
// 49 command = f"_ANGL:{wheel_speed_left},{wheel_speed_right}"
// 50 print(command)
// 51 52
// 53 def main(args=None):
// 54 rclpy.init(args=args)
// 55 rover_controller = KeyboardController()
// 56 rclpy.spin(rover_controller)
// 57
// 58 rclpy.shutdown()
// angular_vel)
//   2

// robot bridge
//  1 import rclpy
// 2 from rclpy.node import Node
// 3 from std_msgs.msg import String
// 4 import sys
// 5 import time
// 6
// 7
// 8 USB_PORT = ’/dev/ttyACM0’ 9
// 10 class Robot(Node): 11
// 12 def __init__(self):
// 13 super().__init__(’bridge’)
// 14 self.publisher = self.create_publisher(String, ’/remote/rover_cmd’, 10)
// 15 # if len(sys.argv) > 1:
// 16 17
// 18 def send(self, val):
// 19 command = String()
// 20 command.data = val #str(sys.argv[1])
// 21 time.sleep(1)
// 22 resutl = self.publisher.publish(command)
// 23 # resutl = self.publisher.publish(command)
// 24 # resutl = self.publisher.publish(command)
// 25
// 26 print(command.data) 27
// 28
// 29 def main(args=None):
// 30
// 31
// 32
// 33
// 34
// 35
// 36
// 37
// 38
// 39
// 40 if 41
// rclpy.init(args=args)
// robot = Robot () time.sleep(0.5)
// if (len(sys.argv[1]) > 0):
// robot.send(sys.argv[1])
// # rclpy.spin_once(robot)
// robot.destroy_node() rclpy.shutdown()
// __name__ == ’__main__’: main()
//   3

// Arduino program
//  1
// 2 #include <stdlib.h>
// 3
// 4 const int TIME = 1000; 5
// 6 int motorright = 9;
// 7 int motorrightdir = 7;
// 8 int motorleft = 10;
// 9 int motorleftdir = 8;
// 10
// 11 const int echoPinLeft =
// 12 const int trigPinLeft =
// 13
// 14 const int echoPinFront
// 15 const int trigPinFront
// 16
// 17 const int echoPinRight
// 18 const int trigPinRight
// 19
// 20 long duration;
// 21 int distance;
// 22
// 23 //For serial receive.
// 24 const byte numChars = 16;
// 25 char receivedChars[numChars]; // an array to store the received data
// 26 String received; //The data as a string
// 27 boolean newData = false;
// 28
// 29 void setup() {
// 30 pinMode(trigPinLeft, OUTPUT); // Sets the trigPin as an Output
// 31 pinMode(echoPinLeft, INPUT); // Sets the echoPin as an Input
// 32 pinMode(trigPinFront ,
// 33 pinMode(echoPinFront ,
// 34 pinMode(trigPinRight ,
// 35 pinMode(echoPinRight ,
// 36 pinMode(motorright , OUTPUT);
// 37 pinMode(motorleft , OUTPUT);
// 38 pinMode(motorrightdir ,
// 39 pinMode(motorleftdir ,
// 40
// 41 Serial.begin(9600);
// OUTPUT); OUTPUT);
// move forawrds
// 42
// 43 // Start the motors to
// 44 String start = "30";
// 45 ForwardAlways(start);
// 46 }
// 47
// 48 void loop()
// 49 {
// 50
// 51
// 52
// 53
// 54
// 55
// 56
// 57
// 58
// 59
// 60
// 61
// 62
// 63 } 64
// 65 int
// 66 {
// 67
// 68
// 69
// 70
// 71
// 72
// 73
// 74
// 75
// 5; 6;
// = 11; = 12;
// = 2; = 4;
// OUTPUT); // Sets the trigPin as an Output INPUT); // Sets the echoPin as an Input OUTPUT); // Sets the trigPin as an Output INPUT); // Sets the echoPin as an Input
// //Serial.print("[");
// // Serial.print(ultrasonic(echoPinLeft ,trigPinLeft));
// // Serial.print(",");
// // Serial.print(ultrasonic(echoPinFront ,trigPinFront));
// // Serial.print(",");
// // Serial.print(ultrasonic(echoPinRight ,trigPinRight));
// // Serial.println("");
// delay(100); //wait for 1/10th of a second.
// ultrasonic(int echoPin , int trigPin)
// recvWithEndMarker(); processCommand();
// // Clears the trigPin digitalWrite(trigPin , LOW); delayMicroseconds(2);
// // Sets the trigPin on HIGH state for 10 micro seconds digitalWrite(trigPin , HIGH);
// delayMicroseconds(10);
// digitalWrite(trigPin , LOW);
// 4

// 76 // Reads the echoPin, returns the sound wave travel time in microseconds
// 77 duration = pulseIn(echoPin ,
// 78 // Calculating the distance
// 79 distance= duration*0.034/2;
// 80 return distance;
// 81 }
// 82
// 83
// 84 void processCommand() {
// HIGH);
// 85 if 86 { 87
// 88
//  89
//  90
//  91
//  92
//  93
//  94
//  95
//  96
//  97
//  98
//  99
// 100
// 101
// 102
// 103
// 104
// 105
// 106
// 107
// 108
// 109
// 110
// 111
// 112
// 113
// 114 }
// 115 }
// 116
// (newData == true)
// // Serial.print("Data Received"); Serial.println(receivedChars);
// String type, _speed;
// int pos = received.indexOf(’:’); if(pos!=-1){
// type = received.substring(0, pos);
// _speed = received.substring(pos + 1); }
// String A = "60,80";
// String instruction = received.substring(0,5);
// String data = received.substring(6,11);
// String spd = received.substring(7,10); //Serial.print(instruction);
// //Serial.print(data);
// if(instruction == "MOVEF") forward(data.toInt()); if(instruction == "MOVEB") backward(data.toInt()); if(instruction == "_TRNL") spinL(data.toInt()); if(instruction == "_TRNR") spinR(data.toInt()); if(instruction == "_STOP") stop();
// if(instruction == "_FFFF") ForwardAlways(spd);//spd.toInt() if(type == "_ANGL") AngularVel(_speed); //speed
// newData = false;
// 117 void recvWithEndMarker()
// 118 {
// 119 static byte ndx = 0;
// 120 char endMarker = ’\n’;
// 121 char rc;
// 122
// 123 while
// 124 {
// 125 rc
// 126
// 127 if
// 128 {
// 129
// 130
// 131
// 132
// 133
// 134
// 135 }
// 136 else
// 137 {
// 138 receivedChars[ndx] = ’\0’; // terminate the string
// 139 received = String(receivedChars);
// 140 ndx = 0;
// 141 newData = true;
// 142 }
// 143 }
// 144 }
// 145
// 146 //direction is controlled by the digital pin 7 and 8.
// 147 // HIGH is backward, LOW is forward
// 148 // Pins 9 and 10 control speed.
// 149 // Length of time controls the distance
// 150
// 151 void forward(int time)
// 152 {
// 153
// (Serial.available() > 0 && newData == false) = Serial.read();
// (rc != endMarker)
// receivedChars[ndx] = rc; ndx++;
// if (ndx >= numChars)
// {
// ndx = numChars - 1; }
// digitalWrite(motorrightdir , LOW);
// 5

// 154 analogWrite(motorright ,180);
// 155 digitalWrite(motorleftdir , LOW);
// 156 analogWrite(motorleft ,
// 157 delay(time);
// 158 stop();
// 159 }
// 180);
// 160
// 161 void backward(int time)
// 162 {
// 163 digitalWrite(motorrightdir ,
// 164 analogWrite(motorright ,180);
// 165 digitalWrite(motorleftdir , HIGH);
// 166 analogWrite(motorleft ,
// 167 delay(time);
// 168 stop();
// 169 }
// 180);
// 170
// 171 void spinL(int time)
// 172 {
// 173 digitalWrite(motorrightdir ,
// 174 analogWrite(motorright ,180);
// 175 digitalWrite(motorleftdir , HIGH);
// 176 analogWrite(motorleft ,
// 177 delay(time);
// 178 stop();
// 179 }
// 180);
// 180
// 181 void spinR(int time)
// 182 {
// 183 digitalWrite(motorrightdir ,
// 184 analogWrite(motorright ,180);
// 185 digitalWrite(motorleftdir , LOW);
// 186 analogWrite(motorleft , 180);
// 187 delay(time);
// 188 stop();
// 189 }
// 190 191
// 192 void stop()
// 193 {
// 194 analogWrite(motorright , 0);
// 195 analogWrite(motorleft , 0);
// 196 }
// 197 198
// 199 void ForwardAlways(String spd)
// 200 {
// 201 int _speed = spd.toInt();
// 202 digitalWrite(motorrightdir ,
// 203 analogWrite(motorright ,_speed);
// 204 digitalWrite(motorleftdir , LOW);
// 205 analogWrite(motorleft , _speed);
// 206 }
// 207
// 208 void AngularVel(String DATA)
// 209 {
// 210
// 211
// 212
// 213
// 214
// 215
// 216
// 217
// 218
// 219
// 220
// 221
// 222
// 223
// 224
// 225
// 226
// 227
// 228
// 229
// 230
// 231
// LOW);
// left = DATA.substring(0, pos);
// LOW); HIGH);
// String left, right;
// int pos = DATA.indexOf(’,’); if(pos!=-1){
// right = DATA.substring(pos + 1); }
// int L = atoi(left.c_str()); int R = atoi(right.c_str());
// Serial.println(L); Serial.println(R);
// if
// {
// (L < 0 & R > 0)
// stop();
// digitalWrite(motorrightdir , analogWrite(motorright , R); digitalWrite(motorleftdir , analogWrite(motorleft , L); delay(TIME);
// HIGH);
// LOW);
// HIGH);
// 6

// 232 }
// 233 elseif(L>0&R<0)
// 234 {
// 235 digitalWrite(motorrightdir ,
// 236 analogWrite(motorright , R);
// 237 digitalWrite(motorleftdir ,
// 238 analogWrite(motorleft , L);
// 239 delay(TIME);
// 240 stop();
// 241 }
// 242 elseif(L>0&R>0)
// 243 {
// 244 digitalWrite(motorrightdir ,
// 245 analogWrite(motorright , R);
// 246 digitalWrite(motorleftdir ,
// 247 analogWrite(motorleft , L);
// 248 delay(TIME);
// 249 stop();
// 250 }
// 251 else
// 252 {
// 253 digitalWrite(motorrightdir ,
// 254 analogWrite(motorright , R);
// 255 digitalWrite(motorleftdir ,HIGH);
// 256 analogWrite(motorleft , L);
// 257 delay(TIME);
// 258 stop();
// 259 }
// 260 }
// HIGH); LOW);
// LOW); LOW);
// HIGH);
//   7
