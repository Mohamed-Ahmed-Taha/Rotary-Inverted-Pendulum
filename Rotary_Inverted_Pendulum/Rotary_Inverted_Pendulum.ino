#include <Wire.h>
#include <esp32-hal-ledc.h>
#include <FreeRTOSConfig.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>

#define ENCODER_TASK_PRIORITY 1
#define SERVO_TASK_PRIORITY 1

void xEncoderTask(void *pvParameters);
void xServoTask(void *pvParameters);

#define DIR_PIN 23
#define AS5600_ADDR (uint8_t)0x36
#define RAWANGLE_ADDRH (uint8_t)0x0C
#define STATUS_ADDR (uint8_t)0x0B
#define MD_BIT 5

#define PWM_PIN 4
#define PWM_CHANNEL 0
#define PWM_FREQ 50
#define PWM_RESOLUTION 16

float sensorAngle = 0.0, newAngle = 0.0;
float servoAngle = 0.0;


void setup() {

  initAS5600();
  initPWM();

  xTaskCreate(xEncoderTask, "Encoder Task", 2048, NULL, ENCODER_TASK_PRIORITY, NULL);
  xTaskCreate(xServoTask, "Servo Task", 1024, NULL, SERVO_TASK_PRIORITY, NULL);

  // vTaskStartScheduler(); // Arduino IDE automatically starts Scheduler

}

void loop() {
  // Empty. Things are done in Tasks.
  // Serial.println(configMINIMAL_STACK_SIZE);
}

void xEncoderTask(void *pvParameters)
{
  while(1){
    if (!detectMagnet())
      continue;
    newAngle = getAngle();
    
    if(newAngle != sensorAngle) {
      sensorAngle = newAngle;
      Serial.println(sensorAngle);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void xServoTask(void *pvParameters)
{
  while(1){
    if(sensorAngle >= 90.0 && sensorAngle <= 270.0){
      servoAngle = sensorAngle - 90.0;
      updateAngle(servoAngle);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}



float getAngle() {
  int regValue = 0;
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAWANGLE_ADDRH);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (size_t) 2, true);
  if (Wire.available() >= 2) {
    char c = Wire.read();
    regValue |= c;
    regValue <<= 8;
    c = Wire.read();
    regValue |= c;
  }
  float res = regValue * (360.0 / 4095.0);

  return res;
}

bool detectMagnet() {
  char c;
  Wire.beginTransmission(AS5600_ADDR); // Begins transmision and starts to queue data given by Wire.write()
  Wire.write(STATUS_ADDR); // Adds address to queue to be sent by I2C
  char error = Wire.endTransmission(false); // Transmits data queued, but sends a repeated start to start to accept data
  if (error != 0) { // A return of 0 means that transmission is succsessful, other wise it indicates an error. Return to Wire documentation.
    Serial.println("Error in I2C end Transmission no. %i");
    Serial.println(error);
    while(1);
  }

  Wire.requestFrom(AS5600_ADDR, (size_t) 1, true); // request one byte of data, True means send stop message after requesting data
  if(Wire.available())
    c = Wire.read();  

  return (c & (1 << MD_BIT)) != 0;  
}

void updateAngle(float angle)
{
	// Zero angle -> CCR = 0.65 / 20 = 0.0325 * 65535 = 2,129.8875
	// 180 angle -> CCR = 2.35 / 20 = 0.1175 * 65535 = 7,700.3625
	// From angle to CCR: angle * (7,700.3625 - 2,129.8875)/180 + 2,129.8875
	int duty = (uint16_t) ((angle * ((7700.3625 - 2129.8875)/180.0)) + 2129.8875);
  ledcWrite(PWM_CHANNEL, duty);
}

void initAS5600()
{
  pinMode(DIR_PIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  digitalWrite(DIR_PIN, LOW);
}

void initPWM()
{
  pinMode(PWM_PIN, OUTPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
}
