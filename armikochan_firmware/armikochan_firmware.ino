#include <AccelStepper.h>
#include <MultiStepper.h>

#define STEPPER_NUM 7
#define DEBOUNCE_DELAY 100 // In Microseconds

const int dir_pins[STEPPER_NUM] = { 4,5,6,7,17,18,8 };
const int step_pins[STEPPER_NUM] = { 3,9,10,11,12,13,14 };
const int hall_pins[STEPPER_NUM] = { 35,45,48,47,21, };

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

void home_steppers(){
  void home_steppers() {
  Serial.println("DEBUG: Starting Homing...");

  for(int i=0; i<STEPPER_NUM; i++) {
    
    // First Homing Cycle
    while (true) {
      stepper[i].runSpeed();

      if(digitalRead(hall_pins[i]) == HIGH) {
        delayMicroseconds(DEBOUNCE_DELAY);
        if(digitalRead(hall_pins[i]) == HIGH) {
          Serial.println("DEBUG: Switch Triggered");
          break;
        }
      }
    }

    // Move in opposite direction for second homing cycle
    stepper[i].setSpeed(-1000); //Reverse Direction
    unsigned long start_time = millis();
    unsigned long interval = 200;
    while(millis() - start_time < interval) {
      stepper[i].runSpeed();
    }

    // Second Homing Cycle
    while (true) {
      stepper[i].runSpeed();

      if(digitalRead(hall_pins[i]) == HIGH) {
        delayMicroseconds(DEBOUNCE_DELAY);
        if(digitalRead(hall_pins[i]) == HIGH) {
          Serial.println("DEBUG: Switch Triggered");
          break;
        }
      }
    }

    stepper[i].setCurrentPosition(0);
    stepper[i].setAcceleration(5000);
  }
}
};
void update_positions();
void update_angles();
void set_gripper();

void setup(){
  Serial.begin(115200);

  for(int i=0; i<STEPPER_NUM; i++){
    pinMode(dir_pins[i], OUTPUT);
    pinMode(step_pins[i], OUTPUT);
    pinMode(hall_pins[i], INPUT);
    stepper[i].setMaxSpeed(2000);
    stepper[i].setSpeed(1500);
    multi.addStepper(stepper[i]);
  }

  Serial.println("INITIALIZED");
}

void loop(){
  Serial.println("Program Sucessfully entered loop");
  delay(1000);
}
