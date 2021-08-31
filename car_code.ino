#include <ECE3.h>

// FUNCTION DECLARATIONS
double get_max_correction();
double map_correction(double correction);
int get_error();
bool all_dark();
void do_180();
void stop_moving();
int get_max_error();



// SET ENVIRONMENT CONSTANTS
int base_speed_max = 95;
int base_speed = 40;
const int turn_speed = 100;
const double k_p = 0.4;
const double k_d = 0.6;
const int max_speed_change_max = 200;
const int min_speed_change_min = -max_speed_change_max;

// DEFINE PINS
const int LED_FR = 41;
const int LED_BR = 58;
const int bumper0 = 24;

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

// SET CALIBRATED SENSOR VALUES
uint16_t sensorValues[8];
int minSensorValues[8] = {733, 616, 593, 570, 524, 639, 556, 709};
int maxSensorValues[8] = {1767, 1884, 1907, 1882, 1871, 1861, 1943, 1791};

// SET SENSOR WEIGHT ARRAYS
const int sensorWeights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
const double max_correction = get_max_correction();
const double min_correction = -max_correction;
const int max_error = get_max_error();
int prev_error = 0;
int prev_error2 = 0;
const uint16_t dark_threshhold = 1500;

bool saw_end = false;
int iter_count = 0;
double k_d_value = 0;
int max_speed_change = 100;
int min_speed_change = -max_speed_change;



void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  // initialize pins
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  // set both motors to be moving forward
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
    
  delay(2000);
}


void loop()
{
  iter_count++;
  
  // read raw sensor values. 0 means max reflectance and 2500 means min reflectance
  ECE3_read_IR(sensorValues);

  int error = get_error();

  double correction = k_p * error + k_d_value * (error - prev_error);
  //  double correction = k_p * error + k_d * (abs(error) - abs(last_error));
  
  correction = map(correction, min_correction, max_correction, min_speed_change, max_speed_change);

  set_motor_speeds(correction);
  
  // check for T at end of track
  if (all_dark()) {
    if (saw_end) {
      stop_moving();
    }
    else {
      do_180();
      saw_end = true;
    }
  }

  prev_error2 = prev_error;
  prev_error = error;

  if (iter_count > 30) {
    base_speed = base_speed_max;
    k_d_value = k_d;
    max_speed_change = max_speed_change_max;
    min_speed_change = min_speed_change_min;
  }
}




// ***************************** HELPER FUNCTIONS **********************************

const void set_motor_speeds(int correction) {
  // calculate motor speeds
  int left_speed = base_speed - correction;
  int right_speed = base_speed + correction;

  // check if we need to reverse a motor direction
  if (left_speed < 0) {
    digitalWrite(left_dir_pin, HIGH);
  }
  else {
    digitalWrite(left_dir_pin, LOW);
  }

  if (right_speed < 0) {
    digitalWrite(right_dir_pin, HIGH);
  }
  else {
    digitalWrite(right_dir_pin, LOW);
  }
  
  // write left and right motor speeds
  analogWrite(left_pwm_pin, abs(left_speed));
  analogWrite(right_pwm_pin, abs(right_speed));
}

// uses sensor weights to compute max error
int get_max_error() {
  int error = 0;
  for (int i=4; i<8; i++) {
    error += sensorWeights[i];
  }

  return error;
}

void off_path_procedure(int error) {
  int correction = 0;
  correction = error < 0 ? -max_speed_change : max_speed_change;

  analogWrite(left_pwm_pin, base_speed - correction);
  analogWrite(right_pwm_pin, base_speed + correction);

  delay(200);
}

// uses the sensor weights, Kd, and Kp to compute the max possible correction value
double get_max_correction() {
  double max_correction = 0;
  for (int i=4; i<8; i++) {
    max_correction += 1000 * sensorWeights[i];
  }
  return max_correction * k_p + k_d * (1600);;
}


// take the correction (i.e. error modified by Kp and Kd) and map it to 
//  the correct correction bounds
double map_correction(double correction) {
  // get the maximum possible correction value, based on current weighting
  int max_bound = 0;
  for (int i=0; i<8; i++) {
    max_bound += 1000 * sensorWeights[i];
  }
}


// gets the weighted sensor sum from the array of sensor values and normalizes it.
// must read sensor values first before calling this function!
int get_error() {
  int sum = 0;
  for (unsigned char i = 0; i<8; i++) {
    int normalized_val = ( (sensorValues[i] - minSensorValues[i]) * 1000 ) / maxSensorValues[i];
    sum += normalized_val * sensorWeights[i];
  }
 
  return sum;
}


// returns true only if 18 sensors read black over 3 separate readings
bool all_dark() {
  int sum = 0;
  for (int x=0; x<3; x++) {
    ECE3_read_IR(sensorValues);
    for (int i=0; i<8; i++) {
      if (sensorValues[i] > dark_threshhold) {
        sum += 1;
      }
    }
  }

  return (sum >= 18);
}


void do_180() {
  resetEncoderCount_left();
  digitalWrite(right_dir_pin, HIGH);
  analogWrite(left_pwm_pin,turn_speed);
  analogWrite(right_pwm_pin,turn_speed);

  while(getEncoderCount_left() < 380) {};

  digitalWrite(right_dir_pin, LOW);
}

void do_180_2() {
  resetEncoderCount_left();
  digitalWrite(right_dir_pin, HIGH);
  analogWrite(left_pwm_pin,40);
  analogWrite(right_pwm_pin,40);

  delay(2700);

  Serial.println(getEncoderCount_left());

  digitalWrite(right_dir_pin, LOW);
}

void stop_moving() {
  analogWrite(left_pwm_pin,0);
  analogWrite(right_pwm_pin,0);
  
  digitalWrite(left_nslp_pin,LOW);
  digitalWrite(right_nslp_pin,LOW);
}
