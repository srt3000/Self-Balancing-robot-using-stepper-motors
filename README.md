# Self-Balancing-robot-using-stepper-motors


Self balancing robot using Arduino uno, Nema 17 Stepper motors, MPU6050, CNC shield and DRV8255 stepper motor drivers


## Working
•	The MPU6050 has a 3-axis accelerometer and a 3-axis gyroscope. 

•	The accelerometer measures acceleration along the three axes and the gyroscope measures angular rate about the three axes.

•	 To measure the angle of inclination of the robot we need acceleration values along y and z-axes. 

•	The atan2(y,z) function gives the angle in radians between the positive z-axis of a plane and the point given by the coordinates (z,y) on that plane, with positive sign for counter-clockwise angles (right half-plane, y > 0), and negative sign for clockwise angles (left half-plane, y < 0).

•	The 3-axis gyroscope of MPU6050 measures angular rate (rotational velocity) along the three axes. For our self-balancing robot, the angular velocity along the x-axis alone is sufficient to measure the rate of fall of the robot. 
