#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>
#include "TWI_290.c"

//TOP USS P13 ON BOARD
#define TOP_TRIG_PIN PB5  // Trigger pin on PORTB; OC2A
#define TOP_ECHO_PIN PD3  // Echo pin on PORTB; INT0

//FRONT USS P6 ON BOARD
#define FRONT_TRIG_PIN PB3
#define FRONT_ECHO_PIN PD2

#define propulsion_fan_direction OCR1A //easier to read in code
#define levitation_fan_pwm OCR0A //easier to read in code
#define propulsion_fan_pwm OCR0B //easier to read in code

//Direction MACRO
#define LEFT 1
#define RIGHT 254
#define CENTER 180

//SERVO IS P9 ON BOARD (PWM0)

//LEVITATION FAN IS P4 ON BOARD (PWM1)

//PROPULSION FAN IS P3 ON BOARD (PWM2)

//IMU IS P7 ON BOARD - PC5 -> SCL, PC4 -> SDA
//defining registers address
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define I2C_ADDRESS 0x68

//defininf gyro output registers for read
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

// defining accel output registers for read
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

//VARIABLES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
volatile uint32_t front_pulse, top_pulse;
float z_gyro_offset = 0;

//UART~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//UART start transmission function
void send_reading(char data) {
  while (!(UCSR0A & (1 << UDRE0))); // Wait until the buffer is empty
  UDR0 = data; // Send received char to UDR0 (Data register)
}

// send string to transmission function
void send_string(const char* str) { // send string to send function
  while (*str) { // while string not empty
    send_reading(*str++); //send string to send_Readin
  }
}

//USS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void USS_INIT() {
  //front USS
  DDRB |= (1 << FRONT_TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << FRONT_ECHO_PIN); // Set ECHO_PIN as input

  //top USS
  DDRB |= (1 << TOP_TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << TOP_ECHO_PIN); // Set ECHO_PIN as input
}


void trigger_sensor(int TRIG_PIN) { //TRIG_PIN is a variable in this case to use the same function twice
  PORTB &= ~(1 << TRIG_PIN);
  _delay_us(2);

  // Send a 10µs pulse to the trigger pin
  PORTB |= (1 << TRIG_PIN);
  _delay_us(10);
  PORTB &= ~(1 << TRIG_PIN);
}



long pulse_length(int ECHO_PIN) //ECHO_PIN is a variable in this case to use the same function twice
{
  long duration = 0;

  while (!(PIND & (1 << ECHO_PIN)));

  while (PIND & (1 << ECHO_PIN)) {
    duration++;
    _delay_us(0.05);

    if (duration == 40000) { //overflow at  686 cm idk you can change this, stops unnecessary waiting
      return duration;
    }
  }
  return duration;
}

uint32_t getDistance(long pulse) {
  return ((pulse * 0.0343) / 2);
}

int scan_front_uss(void) {
  _delay_ms(5);
  //front ultrasonic sensor
  trigger_sensor(FRONT_TRIG_PIN); //sends pulse to the trigger
  front_pulse = pulse_length(FRONT_ECHO_PIN); //see how long the pulse takes to get to the echo
  int distance = getDistance(front_pulse); //calculate the distance

  return distance;
}

int scan_top_uss(void) {
  _delay_ms(5);
  //top ultrasonic sensor
  trigger_sensor(TOP_TRIG_PIN); //sends pulse to the trigger
  top_pulse = pulse_length(TOP_ECHO_PIN); //see how long the pulse takes to get to the echo
  int distance = getDistance(top_pulse); //calculate the distance

  return distance;
}

//IMU~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MPU6050_init(void) {
  TWI_status = TWI_start(I2C_ADDRESS, TW_WRITE);
  if (TWI_status) return;

  TWI_status = TWI_write(0x6B);
  if (TWI_status) return;

  TWI_status = TWI_write(0x00);
  if (TWI_status) return;

  TWI_stop();
}

//vars to hold values on each axis
int16_t x_gyro, y_gyro, z_gyro;
int16_t x_accel, y_accel, z_accel;

void Read_Gyro_Data() {
  uint16_t x_gyro_data[2], y_gyro_data[2], z_gyro_data[2];

  // Read X-axis

  Read_Reg_N(I2C_ADDRESS, GYRO_XOUT_H, 2, x_gyro_data);
  x_gyro = (x_gyro_data[0] << 8) | x_gyro_data[1];

  // Read Y-axis

  Read_Reg_N(I2C_ADDRESS, GYRO_YOUT_H, 2, y_gyro_data);
  y_gyro = (y_gyro_data[0] << 8) | y_gyro_data[1];

  // Read Z-axis
  Read_Reg_N(I2C_ADDRESS, GYRO_ZOUT_H, 2, z_gyro_data);
  z_gyro = (z_gyro_data[0] << 8) | z_gyro_data[1];
}

//Reading Accelerometer Data held in the accel config register
void Read_Accel_Data() {
  //read x-axis
  int16_t x_accel_data[2]; //array holding low and high bytes of gyro register
  Read_Reg_N(ACCEL_CONFIG, ACCEL_XOUT_H, 2, x_accel_data); //read starting from H and store to array
  x_accel = (x_accel_data[0] << 8) | (x_accel_data[1] & 0xFF); // shift by 8 bits x_gyro_data (1byte) -> MSB and combine with the other byte

  //y
  int16_t y_accel_data[2];
  Read_Reg_N(ACCEL_CONFIG, ACCEL_YOUT_H, 2, y_accel_data);
  y_accel = (y_accel_data[0] << 8) | (y_accel_data[1] & 0xFF);


  //z
  int16_t z_accel_data[2];
  Read_Reg_N(ACCEL_CONFIG, ACCEL_ZOUT_H, 2, z_accel_data);
  z_accel = (z_accel_data[0] << 8) | (z_accel_data[1] & 0xFF);

  //Note: &FF to ensure it's reading 8 bits, avoid sign extensions
}

float yaw = 0;
float prev_yaw = 0;

float Calculate_Yaw() {
    float z_gyro_rate = ((z_gyro - z_gyro_offset) / 77.2); // Apply calibrated offset
    
    // Improved noise filtering
    if (fabs(z_gyro_rate) < 0.65) {
        z_gyro_rate = 0;
    }
    
    // Use complementary filter to reduce drift
    float dt = 0.1; // 100ms sample time
    yaw += z_gyro_rate * dt;
    
    // Keep yaw between -180 and 180 degrees
    if (yaw > 90) yaw -= 360;
    if (yaw < -90) yaw += 360;
    
    return yaw;
}

//set servo position
void set_servo_position(float yaws) {

  //map yaw to values in range[0,255]
  yaws *= 1.1;
  if(yaws < -90) yaws = -90;
  if(yaws > 90) yaws = 90;
  
  uint16_t position = ((yaws+90) * 255 / 180);

  //set PWM to value
  OCR1A = Servo_angle[position];
}

// Add these new variables at the top with other variables
#define TURN_THRESHOLD 2.0    // Degrees of acceptable error in turning
#define DRIFT_THRESHOLD 0.5   // Degrees of acceptable drift
#define PID_KP 2.0           // Proportional gain
#define PID_KI 0.1           // Integral gain
#define PID_KD 0.1           // Derivative gain

// Add these PID variables
float integral = 0;
float last_error = 0;

// Add this new PID control function
float calculate_pid_output(float target, float current) {
    float error = target - current;
    
    // Keep error between -180 and 180 degrees
    if (error > 90) error -= 360;
    if (error < -90) error += 360;
    
    integral += error * 0.1;  // dt = 0.1
    
    // Anti-windup
    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;
    
    float derivative = (error - last_error) / 0.1;
    last_error = error;
    
    float output = (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivative);
    
    // Limit output
    if (output > 90) output = 90;
    if (output < -90) output = -90;
    
    return output;
}

//MAIN CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(void)
{
  //initializing
  gpio_init();
  USS_INIT(); //Initialize US sensor
  uart_init(9600); //Initialize Uart
  TWI_init(); //Init TWI
  MPU6050_init(); //Initialize MPU6050
  timer1_50Hz_init(0); //in init_290.c, initialize PWM0 for servo and PWM1 for leviation fan
  timer0_init(); //PWM2 for propulsion fan

  // Initialize IMU
  Write_Reg(I2C_ADDRESS, 0x6B, 0x00);
  Write_Reg(I2C_ADDRESS, ACCEL_CONFIG, 0x00); // ±2g range
  Write_Reg(I2C_ADDRESS, GYRO_CONFIG, 0x00);  // ±500 deg/s range
//Main variables
  int front_distance, top_distance, left_distance, right_distance, center_distance;
  char buffer[50];

  //Servo direcrion init

  //calculate initial value for z axis
  int32_t z_gyro_offset = 0;

  // Serial.println("Start calibration");
  for (int m = 0; m < 500; m++) {
    Read_Gyro_Data();
    z_gyro_offset += (z_gyro);
    _delay_ms(1);
  }
  z_gyro_offset /= 500;

  //turn on both fans
  levitation_fan_pwm = 255; //start fan at max rpm
  propulsion_fan_pwm = 255; //start fan at max rpm

  float target_yaw = 0;
  float diff = 0;

  while (1) //constantly loops while the power is on
  {
    // read IMU
    _delay_ms(20);
    Read_Gyro_Data();
    // Read_Accel_Data();

    yaw = Calculate_Yaw();

    // Calculate PID output for steering
    float steering_correction = calculate_pid_output(target_yaw, yaw);
    set_servo_position(steering_correction);
    
    // read USS value
    front_distance = scan_front_uss(); //calculate the distance
    top_distance = scan_top_uss();

    if (top_distance <= 40) { //first check if the top is detected since most important
      levitation_fan_pwm = 0; //stop fan
      propulsion_fan_pwm = 0; //stop fan
      break; //end
    }

    else if (front_distance <= 10) { //second check the front if its in front of a wall
      propulsion_fan_pwm = 0; //stop fan
      levitation_fan_pwm = 0; //stop both fan
      _delay_ms(1000);
      
      //scan left side
      set_servo_position(-90); //turn servo all the way left
      _delay_ms(750);
      left_distance = scan_front_uss();

      //Scan center
      set_servo_position(0); //turn servo to center
      _delay_ms(750);
      center_distance = scan_front_uss();

      //scan right side
      set_servo_position(90); //turn servo all the way right
      _delay_ms(750);
      right_distance = scan_front_uss();

      //comparing left and right wall distances
      if ((left_distance < right_distance) && (front_distance < right_distance)) { //wall on left -> turn right
        target_yaw += 90;  // Increment target by 90 degrees
        if (target_yaw > 180) target_yaw -= 360;  // Keep within ±180 range
      }
      else if ((right_distance < left_distance) && (front_distance < left_distance)) { //wall on right -> turn left
        target_yaw -= 90;  // Decrement target by 90 degrees
        if (target_yaw < -180) target_yaw += 360;  // Keep within ±180 range
      }
      else{
        yaw = 0;
        set_servo_position(yaw);
      }

      //if they are equal it will just repeat the loop, detect the wall in front again, then check again
      propulsion_fan_pwm = 255; //restart fan
      levitation_fan_pwm = 255; //restart levi fan at max rpm
      _delay_ms(200);
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }
  return 0; //end
}