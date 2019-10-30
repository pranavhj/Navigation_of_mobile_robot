# Navigation_of_mobile_robot
Make the connections as shown in the initializations of the code
Using this code one can navigate a 2 wheeled diffrential robot(Dc motors used) in a known enviroment using analog rotary encoders and a IMU 
The MPU 6050 is used to detect the yaw angle and navigate through a known enviroment using this sensor.
The encoders tell the robot the distance it has moved and when it has to stop
We can use the target_yaw(yaw,dist) function to make the robot move at a particular angle read by the IMU for distance d.

