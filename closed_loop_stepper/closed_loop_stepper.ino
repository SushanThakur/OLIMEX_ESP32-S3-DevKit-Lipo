#include <AS5600.h>
#include <AccelStepper.h>
#include "freertos/semphr.h"

#define I2C_SDA 15
#define I2C_SCL 16

#define DIR_PIN 4
#define STEP_PIN 5

#define STEP_SPEED 2000

const int steps_per_rev = 800;

int target_pos = 0;
int current_pos = 0;

TaskHandle_t angle_h;
TaskHandle_t step_h;

AS5600 as5600;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void step_t (void *pvParameters) 
{
  vTaskDelay(1000/portTICK_PERIOD_MS);
  while(true) {
    stepper.run();
//    current_pos = stepper.currentPosition();/
  }
}

void angle_t (void *pvParameters)
{
  while(true) {
    static uint32_t last_read = 0;
    
    if(millis() - last_read >= 2) {
      last_read = millis();
      
      static int last_angle = 0;
      int angle = 0;
      
      if(as5600.isConnected()) {
        int cumulative = as5600.getCumulativePosition();
        int rev = as5600.getRevolutions();
        angle = map(cumulative - 4096*rev, 0, 4096, 0, 360) + 360*rev;
        
        if(abs(angle - last_angle) < 90) {
          last_angle = angle;
        }
      }
      current_pos = last_angle;
      int del = target_pos - current_pos;
      if (del >= 2 || del <= -2){
//        target_pos = (step_position - la/st_angle) * steps_per_rev;
        stepper.moveTo(stepper.currentPosition() + del*steps_per_rev/360);
      } 

      Serial.printf("DEBUG: del = %d || angle = %d || current pos = %d || distanceToGo = %d \n",del,last_angle, stepper.currentPosition() ,stepper.distanceToGo());
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(50000);  // Reduced to 50kHz
  Wire.setTimeout(5);    // 5ms timeout 

  as5600.begin(8);
  as5600.setDirection(AS5600_CLOCK_WISE);

  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  as5600.setDirection(AS5600_CLOCK_WISE);
  as5600.resetPosition();

  xTaskCreatePinnedToCore 
  (
    angle_t,
    "angle_t",
    10000,
    NULL,
    0,
    &angle_h,
    0
  );

  xTaskCreatePinnedToCore 
  (
    step_t,
    "step_t",
    10000,
    NULL,
    1,
    &step_h,
    1
  );

  // Stepper setup
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(2000);
  stepper.setSpeed(2000);
  stepper.setAcceleration(500);

  delay(1000);
}

void loop() {
  vTaskDelete(NULL);
}
