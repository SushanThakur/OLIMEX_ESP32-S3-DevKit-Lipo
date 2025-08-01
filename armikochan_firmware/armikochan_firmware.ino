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

TaskHandle_t serial_h;
TaskHandle_t home_steppers_h;
TaskHandle_t move_steppers_h;
TaskHandle_t check_angles_h;
TaskHandle_t grip_control_h;

bool all_homed = false;
bool stepper_homed[STEPPER_NUM] = {false};
bool new_pos = false;
bool grip_update = false;

const int dir_pins[STEPPER_NUM] = { 4,5,6,7,17,18,8 };
const int step_pins[STEPPER_NUM] = { 3,9,10,11,12,13,14 };
const int hall_pins[STEPPER_NUM] = { 21,47,48,45,35,39,40 };

const float max_shaft_angles[STEPPER_NUM] = {0.0};
const float min_shaft_angles[STEPPER_NUM] = {0.0};


float target_shaft_angles[STEPPER_NUM] = {0.0};
float target_stepper_steps[STEPPER_NUM] = {0.0};

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

// Fast string-to-integer conversion
inline int32_t fastAtoi(const char *str) {
    int32_t val = 0;
    while (*str >= '0' && *str <= '9') {
        val = val * 10 + (*str++ - '0');
    }
    return val;
}

void shaft_angles_to_steps() {
  // convert shaft angles to respective stepper steps 
}

void steppers_move_n_check() {
  // Reset stepper_homed array
  for (int i=0; i<STEPPER_NUM; i++) {
    stepper_homed[i] = false;
  }

  while (!all_homed) {
    for (int i=0; i<STEPPER_NUM; i++){
      if (!stepper_homed[i]){
        stepper[i].runSpeed();
      }
      if(digitalRead(hall_pins[i]) == HIGH) {
        delayMicroseconds(DEBOUNCE_DELAY_HOME);
        if(digitalRead(hall_pins[i]) == HIGH) {
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

/* 
  =========================
    TASKS
  =========================
*/

void home_steppers_t( void *pvParameters ) {
  if( !all_homed ){

    steppers_move_n_check();

    // Make the stepper rotate in opposite direction
    for (int i=0; i<STEPPER_NUM; i++) {
      stepper[i].setSpeed(-STEPPER_RUN_SPEED); 
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
  }
}

void serial_t( void *pvParameters ) {
  while(Serial.available() > 0) {
    char buffer[128];
    size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';
    String read_string = String(buffer);
    read_string.trim();

    if (read_string == "MOVE") {
      // READ ANGLES FROM SERIAL
      char buffer[128];
      size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
      buffer[len] = '\0';
      int index = 0;
      char *token = strtok(buffer, ",");
      while (token != NULL && index < STEPPER_NUM ) {
        float angle = fastAtoi(token);
        target_shaft_angles[index] = constrain(angle, min_shaft_angles[index], max_shaft_angles[index]);
        index++;
        token = strtok(NULL, ",");
      }
      if (index != STEPPER_NUM) {
        Serial.println("DEBUG: ERROR: Incomplete arm position data");
      }
      shaft_angles_to_steps();
      new_pos = true;
    } 
    else if (read_string == "HOME") {
      all_homed = false;
    }
  }
}

void move_steppers_t( void *pvParameters ) {
  if ( new_pos ){
    for( int i=0; i<STEPPER_NUM; i++ ){
      stepper[i].moveTo(target_stepper_steps[i]);
    }
    // multi.run();
    if ( !multi.run() ) {
      new_pos = false;
    }
  }
}

void check_angles_t( void *pvParameters ) {
  
}

void grip_control_t( void *pvParameters ) {
  
}

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

  // Task Setup
  xTaskCreatePinnedToCore (
    home_steppers_t,
    "home_steppers_t",
    10000, // Stack Size
    NULL,
    0,
    &home_steppers_h,
    0
  );

  xTaskCreatePinnedToCore (
    serial_t,
    "serial_t",
    10000, // Stack Size
    NULL,
    0,
    &serial_h,
    0
  );

  xTaskCreatePinnedToCore (
    move_steppers_t,
    "move_steppers_t",
    10000, // Stack Size
    NULL,
    0,
    &move_steppers_h,
    0
  );

  xTaskCreatePinnedToCore (
    check_angles_t,
    "check_angles_t",
    10000, // Stack Size
    NULL,
    0,
    &check_angles_h,
    1
  );

  xTaskCreatePinnedToCore (
    grip_control_t,
    "grip_control_t",
    10000, // Stack Size
    NULL,
    0,
    &grip_control_h,
    1
  );

}

void loop(){

}
