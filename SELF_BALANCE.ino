
/*
HOW TO USE ROBOT:
1.turn on robot
2.pin 13 led will start blinking(indicates calibration)
3.wait until led stops blinking(indicates end of calibration)
4.gently lift the robot to standing position & leave it
*/

/*hardware connections:
 * A Arduino CNC shield is plugged into Arduino UNO
 * 2 stepper drivers DRV8825 are plugged into X & Y ports in the shield
 * both of them are set to 1/8th microstepping mode which gives accuracy of 1.8(stepping angle)/8 = 0.225 degrees
 * the board is supplied 12v from lipo battery
 * Gyro's SCL >> CNC board's SCL
 * Gyro's SDA >> CNC board's SDA
 */
#include <Wire.h>

//Change only these:
//how to tune PID - https://www.youtube.com/watch?v=uyHdyF0_BFo
#define PID_P_gain 11.9  //11.9
#define PID_I_gain 1.16//1.16
#define PID_D_gain 1.9  //1.9
#define PID_tolerance 0.5
#define PID_tolerance_min -0.5
float PID_set_point = 7.0;//the balance point of the robot

#define motor_delay_max 400
#define rev_dir true// if motor is spinning in opp direction set this to true
//dont change these:
#define acc_calibration_value 0 // calibration value for raw accelerometer data,
                                //for example,like u keep the robot in perpendicular position, note the accelerometer value and then store it as calibration value,
                                //usually it is somewhere near 0 but If we want the robot to balance properly from starting then it should be finely calibrated
#define gyro_address 0x68       //gyro's default address
#define motor_delay_min (motor_delay_max * -1)   //Motor delay specifies the max pid limit, if it is negative motor runs in reverse


//for MPU 6050
int gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature; //variables for raw mpu data - in degrees per second
long gyro_x_cal, gyro_y_cal, gyro_z_cal; // calibration values
float x_gyro_val, y_gyro_val, z_gyro_val; //values for computation - in angles
float angle_acc;
int angle_pitch_buffer, angle_roll_buffer, accelerometer_data_raw;
long loop_timer; 
//For PID controller:
float PID_output, PID_error, PID_prev_D_error, PID_I_mem; //PID_I_mem= pid integral memory,PID_prev_D_error= previous derivative error
//For Motor
int motor_delay = 0;
int motor_curr_count, motor_mem_count; // Motor curr count is like a counter to count till it reaches mem count value and then trigger the change,  
                                       //Motor mem count is the delay time between steps, more it's value- the slower the stepper motor will rotate
// for starting/stopping the robot
bool state = false;

int zero_pos = 0;             
int zero_pos_dir_mem = 0;


void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(5, OUTPUT);//dir of motor X
  pinMode(2, OUTPUT);//step of motor X
  pinMode(6, OUTPUT);//dir of motor Y
  pinMode(3, OUTPUT);//step of motor Y

  pinMode(13, OUTPUT); // for indication

  
  TWBR = 12;                  //Set the I2C clock speed to 400kHz, TWBR is Two Wire Baud Rate register of the MCU to set the speed of I2C communication
  //for enabling 20us interrupt to run motor
  TCCR2A = 0;               //Make sure that the TCCR2A register is set to zero,it is to reset the register, 
                            //TCCR2A and TCCR2B are timer counter control registers A and B
  TCCR2B = 0;               //Make sure that the TCCR2A register is set to zero, it is to reset the register
  TIMSK2 |= (1 << OCIE2A);  //Set the interupt enable bit OCIE2A in the TIMSK2 register or Enable interrupt when Timer reaches OCRA (output compare register A)
  TCCR2B |= (1 << CS21);    //Set the CS21 bit in the TCCRB register to set the prescaler to 8, timer speed (Hz) = ((Arduino clock speed (16MHz)) / prescaler).So a 1 prescaler will increment the counter at 16MHz, an 8 prescaler will increment it at 2MHz
  OCR2A = 39;               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);   //Set counter 2 to CTC (clear timer on compare) mode
  
  setup_mpu_6050();//starts communication with MPU_6050
  read_mpu_6050_data();//reads and updates data of MPU 6050
  y_gyro_val = angle_acc;
  
  loop_timer = micros();//Returns the number of microseconds in loop_timer
}

void loop()
{

  read_mpu_6050_data();  //starts the robot when it is almost in upright position
  if(state == false && angle_acc > -0.5 && angle_acc < 0.5) //if state is false and angle_accis between -0.5 and 0.5
  {                     
    y_gyro_val = angle_acc;                                                 
    state = true;                                                              
    zero_pos = 0;
    digitalWrite(13, HIGH);
  }

  //converting angular velocity to angle
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
//x_gyro_val += gyro_x * 0.000031;// we don't use x values since we need only y axis values
  //x_gyro_val = x_gyro_val * 0.9996 + angle_acc * 0.0004; //Correct the drift of the gyro angle with the accelerometer angle
  y_gyro_val += gyro_y * 0.000031;
  y_gyro_val = y_gyro_val * 0.9996 + angle_acc * 0.0004;  //Correct the drift of the gyro angle with the accelerometer angle

  //PID calculations
  if(PID_output > 10 || PID_output < -10)PID_error += PID_output * 0.015 ; //if pid error is between -10&10, pid error = pid error+output*0.015
                                                                           //0.015 is a predicted value, intially it was 0 but it was found that if PID output is slightly increased it acts like a braking feature and helps it recover fatser
  PID_error = PID_set_point - y_gyro_val;
  PID_I_mem += PID_error * (PID_I_gain);
  PID_output = (PID_error * PID_P_gain) + PID_I_mem + (PID_D_gain* (PID_error - PID_prev_D_error)) ;
  PID_prev_D_error = PID_error;

  if(PID_output > motor_delay_max)PID_output = motor_delay_max;          //Limit the PI-controller to the maximum controller output
  else if(PID_output < motor_delay_min)PID_output = motor_delay_min;     // motor delay max is defined above as 200, so if pid>200, motor delay max=200, Motor delay min is -(motor delay max)or -200

  if((PID_output < PID_tolerance) && (PID_output > PID_tolerance_min))PID_output = 0;//if PID output is within tolerance dont move the motor
                                                                                     //tolerance and its min values are defined above as -0.5 and 0.5, so if it is a value between them, make pid value 0
  //for stopping the robot automatically if it goes beyond a unrecoverable state
  if(y_gyro_val > 30 || y_gyro_val < -30 || state == false){    // if gyro y values are between -30&30 or if state is false
    PID_output = 0;                                             // make pid output, pid i memory as 0          
    PID_I_mem = 0;                                                          
    state = false;                                              // stops movement                
    zero_pos = 0;
    digitalWrite(13, LOW);
  }


  //motor calculations
  if(PID_output > 0)PID_output = 405 - (1/(PID_output + 9)) * 5500;       //These eqns relates duration between steps since pid output corresponds to final speed of the robot    and final speed of robot
  else if(PID_output < 0)PID_output = -405 - (1/(PID_output - 9)) * 5500;

  if(PID_output > 0)motor_delay = motor_delay_max - PID_output;  //to adjust speed of motor by changing delay
  else if(PID_output < 0)motor_delay = motor_delay_min - PID_output;  //to adjust speed of motor by changing delay
  else motor_delay = 0;     //only when pid output =0

  if(rev_dir)motor_delay *= -1;     //it is defined at the start of program
  //maintains loops at 250 Hz
  while(micros() - loop_timer < 4000);  ////Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop                               
  loop_timer = micros();                                              
}

//occurs every 20us
ISR(TIMER2_COMPA_vect){         //In every 20us this gets executed and has high priority, even if the program is in Main loop, it will pause and execute command inside this function
  motor_curr_count ++;          //increments by 1 at each 20us
  if(motor_curr_count > motor_mem_count)
  {
    motor_curr_count = 0;
    motor_mem_count = motor_delay;
    zero_pos_dir_mem = motor_delay;
//for changing direction in motor delay is negative
    if(motor_mem_count < 0){
      motor_mem_count *= -1;
      PORTD |= 0b00100000; // set pin 5 to HIGH
      PORTD |= 0b01000000; // set pin 6 to HIGH
    }
    else {
      PORTD &= 0b11011111; // set pin 5 to low
      PORTD &= 0b10111111; // set pin 6 to low
    }
  }
  //for stepping the motor
  else if(motor_curr_count == 1){
    PORTD |= 0b00000100; // set pin 2 to HIGH
    PORTD |= 0b00001000; //set pin 3 to HIGH
    if(zero_pos_dir_mem < 0)zero_pos -= 1;
    else zero_pos += 1;
  }
  else if(motor_curr_count == 2){
    PORTD &= 0b11111011; // set pin 2 to low
    PORTD &= 0b11110111;// set pin 3 to low
  }
}

//subroutine that reads gyro data
void read_mpu_6050_data(){                                        
  //accelerometer angle calculations
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;          
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         

  angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer
  
  // for gyro
  Wire.beginTransmission(0x68);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(0x68, 4);                                        //Request 4 bytes from the gyro
  gyro_x = Wire.read()<<8|Wire.read();                           //Combine the two bytes
  gyro_y = Wire.read()<<8|Wire.read();                         //Combine the two bytes
}

//sets up communication with MPU-6050 and calibrates its jitter
void setup_mpu_6050(){
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 
  
  //callibration
  Serial.println("Calibrating Gyro");
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 100 == 0){
      Serial.print(". ");                                              // prints . 20 times
      digitalWrite(13, !digitalRead(13));                              //to toggle the value
    }
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  
  Serial.println();
  digitalWrite(13, HIGH);
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the average offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the average offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the average offset
  delay(1500);
  Serial.println("Calibration Done");
  digitalWrite(13, LOW);
  Serial.println(gyro_x);
  Serial.println(gyro_y);
  Serial.println(gyro_z);
 
  Serial.println(acc_x);
  Serial.println(acc_y );
  Serial.println(acc_z);
 
}
