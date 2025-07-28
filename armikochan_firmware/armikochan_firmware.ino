#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>

// ESP CONFIG
#define BAUD 115200
#define I2C_SDA 15
#define I2C_SCL 16

// TIME CONST
#define DEBOUNCE_DELAY_HOME 100 // In Microseconds
#define REVERSE_INTERVAL 200 // In Miliseconds

// PHYSICAL CONST
#define STEPPER_NUM 7

// STEPPER PARAM
#define STEPPER_MAX_SPEED 2000
#define STEPPER_RUN_SPEED 1000
#define STEPPER_ACEL 5000

const int dir_pins[STEPPER_NUM] = { 4,5,6,7,17,18,8 };
const int step_pins[STEPPER_NUM] = { 3,9,10,11,12,13,14 };
const int hall_pins[STEPPER_NUM] = { 21,47,48,45,35,39,40 };

bool stepper_homed[STEPPER_NUM] = {false};
bool all_homed = false;

AccelStepper stepper[STEPPER_NUM] = {
  AccelStepper(AccelStepper::DRIVER, step_pins[0], dir_pins[0]),
  AccelStepper(AccelStepper::DRIVER, step_pins[1], dir_pins[1]),
  AccelStepper(AccelStepper::DRIVER, step_pins[2], dir_pins[2]),
  AccelStepper(AccelStepper::DRIVER, step_pins[3], dir_pins[3]),
  AccelStepper(AccelStepper::DRIVER, step_pins[4], dir_pins[4]),
  AccelStepper(AccelStepper::DRIVER, step_pins[5], dir_pins[5]),
  AccelStepper(AccelStepper::DRIVER, step_pins[6], dir_pins[6]),
};

MultiStepper multi;

void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}

void steppers_move_n_check() {
  while (!all_homed) {
    for (int i=0; i<STEPPER_NUM; i++){
      if (!stepper_homed[i]){
        stepper[i].runSpeed();
      }
      if(digitalRead(hall_pins[i]) == HIGH) {
        delayMicroseconds(DEBOUNCE_DELAY_HOME);
        Serial.println("DEBUG: Switch Triggered Frist Time");
        if(digitalRead(hall_pins[i]) == HIGH) {
          Serial.println("DEBUG: Switch Triggered Second Time");
          stepper_homed[i] = true;
        }
      }
    }
    all_homed = true;
    for (int i=0; i<STEPPER_NUM; i++) {
      if (!stepper_homed[i]) {
        all_homed = false;
      }
    }
  }
}

void home_steppers(){
  Serial.println("DEBUG: Starting Homing");

  /*
    
  */

  steppers_move_n_check();

  // Reset stepper_homed variable
  for (int i=0; i<STEPPER_NUM; i++) {
    stepper_homed[i] = false;
    stepper[i].setSpeed(-STEPPER_RUN_SPEED); // to make the stepper rotate in opposite direction
  }

  // Move in opposite direction for 'reverse_interval' time period
  unsigned long reverse_start_time = millis();
  while(millis() - reverse_start_time < REVERSE_INTERVAL){
    for(int i=0; i<STEPPER_NUM; i++){
      stepper[i].runSpeed();
    }
  }

  steppers_move_n_check();

  for(int i=0; i<STEPPER_NUM; i++){
    stepper[i].setCurrentPosition(0);
    stepper[i].setAcceleration(STEPPER_ACEL);
  }

  Serial.println("DEBUG: Homing Done");
}

void update_positions(){};
void update_angles(){};
void set_gripper(){};

void setup(){
  Serial.begin(115200);

  for(int i=0; i<STEPPER_NUM; i++){
    pinMode(dir_pins[i], OUTPUT);
    pinMode(step_pins[i], OUTPUT);
    pinMode(hall_pins[i], INPUT);
    stepper[i].setMaxSpeed(STEPPER_MAX_SPEED);
    stepper[i].setSpeed(STEPPER_RUN_SPEED);
    multi.addStepper(stepper[i]);
  }

  Serial.println("DEBUG: Setup Complete");
}

void loop(){
  Serial.println("Program Sucessfully entered loop");
  delay(1000);
}
