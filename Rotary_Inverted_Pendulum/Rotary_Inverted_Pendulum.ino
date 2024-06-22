#include <Wire.h>
#include <BluetoothSerial.h>
// #include <WiFi.h>
// #include <WiFiClient.h>
// #include <WiFiAP.h>

#define SERVO_TASK_PRIORITY 5
#define ENCODER_TASK_PRIORITY 4
#define BUTTON_TASK_PRIORITY 3
#define APP_COMM_PRIORITY 2
#define PRINT_DATA_PRIORITY 1

void xEncoderTask(void *pvParameters);
void xServoTask(void *pvParameters);
void xPrintData(void *pvParameters);
void xAppCommunication(void *pvParameters);
void xButtonTask(void *pvParameters);

#define DIR_PIN 23
#define AS5600_ADDR (uint8_t)0x36
#define RAWANGLE_ADDRH (uint8_t)0x0C
#define STATUS_ADDR (uint8_t)0x0B
#define MD_BIT 5

#define PWM_PIN 4
#define PWM_CHANNEL 0
#define PWM_FREQ 50
#define PWM_RESOLUTION 20
#define BUTTON_PIN 25

#define PERIOD pdMS_TO_TICKS(50)

void calculatePID();
double getAngle();
bool detectMagnet();
void updateAngle(double angle);
void resetServo();
void initAS5600();
void initPWM();

// const char *ssid = "4a2leb el donya";
// const char *password = "w23na_el_donya";

// WiFiServer server(80);

double sensorAngle = 0.0;
double servoAngle = 0.0;
int duty = 0;

BluetoothSerial serialBT;

int button = 0;
int buttonPrev = 0;

double currentTime = 0.0, deltaTime = 0.0, previousTime = 0.0;
double errorValue = 0.0, edot = 0.0, errorIntegral = 0.0, previousError = 0.0;
double controlSignal = 0.0;

const double proportional = 0.24; //0.23;
const double integral = 0.000001; //0.0000001;
const double derivative = 0.09; //0.09;

const double magnetPlaceError = 20.87 + 7.61 - 5.0 - 3.39;
;
const double targetPosition = 180.0;
const double servoResetAngle = 90.0;

int tangle = 0;
int mangle = 0;
int sangle = 0;
int pangle = 0;
int iangle = 0;
int dangle = 0;
int cangle = 0;

int semaphore = 0;

void setup() {

  // WiFi_init();
  Serial.begin(115200);
  serialBT.begin("42leb el donya");
  initAS5600();
  initPWM();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  resetServo();

  xTaskCreate(xAppCommunication, "App Communication", 2048, NULL, APP_COMM_PRIORITY, NULL);
  xTaskCreate(xEncoderTask, "Encoder Task", 2048, NULL, ENCODER_TASK_PRIORITY, NULL);
  xTaskCreate(xServoTask, "Servo Task", 2048, NULL, SERVO_TASK_PRIORITY, NULL);
  xTaskCreate(xPrintData, "Print Task", 2048, NULL, PRINT_DATA_PRIORITY, NULL);
  xTaskCreate(xButtonTask, "Button Task", 2048, NULL, BUTTON_TASK_PRIORITY, NULL);

  // vTaskStartScheduler(); // Arduino IDE automatically starts Scheduler
}

void loop() {
  // Empty. Things are done in Tasks.
  // Serial.println(configMINIMAL_STACK_SIZE);
}

void xAppCommunication(void *pvParameters)
{
  while(1){
    if(1){
      serialBT.print("Desired Angle: ");
      serialBT.println(targetPosition);
      vTaskDelay(pdMS_TO_TICKS(250));
      serialBT.print("Sensor Angle: ");
      serialBT.println(sensorAngle);
      vTaskDelay(pdMS_TO_TICKS(250));
      serialBT.print("Servo Angle: ");
      serialBT.println(servoAngle);
      vTaskDelay(pdMS_TO_TICKS(250));
      serialBT.print("P Scale: ");
      serialBT.println(proportional);
      vTaskDelay(pdMS_TO_TICKS(250));
      serialBT.print("I Scale: ");
      serialBT.println(integral);
      vTaskDelay(pdMS_TO_TICKS(250));
      serialBT.print("D Scale: ");
      serialBT.println(derivative);
      vTaskDelay(pdMS_TO_TICKS(250));
      serialBT.print("Control Signal: ");
      serialBT.println(controlSignal);
      vTaskDelay(pdMS_TO_TICKS(250));
      serialBT.println("|");
      vTaskDelay(pdMS_TO_TICKS(100));
    }

  //    WiFiClient client = server.available();   // listen for incoming clients

  //   if (client) {                             // if you get a client,
  //     Serial.println("New Client.");           // print a message out the serial port
  //     String currentLine = "";                // make a String to hold incoming data from the client
  //     while (client.connected()) {            // loop while the client's connected
  //       if (client.available()) {             // if there's bytes to read from the client,
  //         char c = client.read();             // read a byte, then
  //         Serial.write(c);                    // print it out the serial monitor
  //         if (c == '\n') {                    // if the byte is a newline character

  //           // if the current line is blank, you got two newline characters in a row.
  //           // that's the end of the client HTTP request, so send a response:
  //           if (currentLine.length() == 0) {
  //             // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  //             // and a content-type so the client knows what's coming, then a blank line:
  //             client.println("HTTP/1.1 200 OK");
  //             client.println("Content-type:text/html");
  //             client.println();
            
  //             // the content of the HTTP response follows the header:
  //             client.print("Desired Angle: ");
  //             client.print(targetPosition);
  //             client.print(" Degrees.<br>");
  //             client.print("Sensor Angle: ");
  //             client.print(sensorAngle);
  //             client.print(" Degrees.<br>");
  //             client.print("Servo Angle: ");
  //             client.print(servoAngle);
  //             client.print(" Degrees.<br>");
  //             client.print("P Scale: ");
  //             client.print(proportional);
  //             client.print(".<br>");
  //             client.print("I Scale: ");
  //             client.print(integral);
  //             client.print(".<br>");
  //             client.print("D Scale: ");
  //             client.print(derivative);
  //             client.print(".<br>");
  //             client.print("Control Signal: ");
  //             client.print(controlSignal);
  //             client.print(" Degrees.<br>");

  //             // The HTTP response ends with another blank line:
  //             client.println();
  //             // break out of the while loop:
  //         //   } else {    // if you got a newline, then clear currentLine:
  //         //     currentLine = "";
  //         //   }
  //         // } else if (c != '\r') {  // if you got anything else but a carriage return character,
  //         //   currentLine += c;      // add it to the end of the currentLine
  //         // }
  //       }
  //     }
  //     // close the connection:
  //   }
  }
}

void xEncoderTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    if(!detectMagnet())
      continue;
    
    sensorAngle = getAngle() - magnetPlaceError; // Remove difference produced by misplacing magnet opposite to sensor

    if(semaphore){
    

      if(sensorAngle < 100.0 || sensorAngle > 260.0){
        resetServo();
        continue;
      }

      calculatePID();
    }
    vTaskDelayUntil(&xLastWakeTime, PERIOD);
  }
}

void xServoTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    if(semaphore){
      servoAngle += controlSignal;
      if(servoAngle < 0.0 || servoAngle > 180.0){
        resetServo();
        continue;
      }
      
      updateAngle(servoAngle);
    }

    vTaskDelayUntil(&xLastWakeTime, PERIOD);
  }
}

void xButtonTask(void *pvParameters)
{
  while(1){
    button = digitalRead(BUTTON_PIN);
    if(!button && buttonPrev) {
      semaphore = 1;
      buttonPrev = 0;
    }

    if(button && !buttonPrev)
      buttonPrev = 1;
    
    vTaskDelay(PERIOD);
  }
}

void xPrintData(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    Serial.print("Sensor Angle: ");
    Serial.print(sensorAngle);
    Serial.print("                    Servo Angle: ");
    Serial.print(servoAngle);
    Serial.print("                    Conrol Signal: ");
    Serial.print(controlSignal);
    Serial.print("                    Button Signal: ");
    Serial.print(button);
    Serial.print("                    Semaphore: ");
    Serial.println(semaphore);
    vTaskDelayUntil(&xLastWakeTime, PERIOD);
  }
}

// void WiFi_init(){
//     Serial.begin(115200);
//   Serial.println();
//   Serial.println("Configuring access point...");

//   // You can remove the password parameter if you want the AP to be open.
//   // a valid password must have more than 7 characters
//   if (!WiFi.softAP(ssid, password)) {
//     log_e("Soft AP creation failed.");
//     while(1);
//   }
//   IPAddress myIP = WiFi.softAPIP();
//   Serial.print("AP IP address: ");
//   Serial.println(myIP);
//   server.begin();

//   Serial.println("Server started");
// }

void calculatePID()
{
  //Determining the elapsed time
  currentTime = xTaskGetTickCount(); //current time
  deltaTime = currentTime - previousTime; //time difference in seconds
  previousTime = currentTime; //save the current time for the next iteration to get the time difference
  //---
  errorValue = sensorAngle - targetPosition; //Current position - target position (or setpoint)

  edot = (errorValue - previousError) / deltaTime; //edot = de/dt - derivative term

  errorIntegral = errorIntegral + (errorValue * deltaTime); //integral term - Newton-Leibniz, notice, this is a running sum!

  controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral); //final sum, proportional term also calculated here

  previousError = errorValue; //save the error for the next iteration to get the difference (for edot)
}

double getAngle() 
{
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
  double res = regValue * (360.0 / 4095.0);

  return res;
}

bool detectMagnet()
{
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

void updateAngle(double angle)
{
	// Zero angle -> CCR = 0.55 / 20 = 0.025 * 1,048,575 = 26,214.375
	// 180 angle -> CCR = 2.45 / 20 = 0.125 * 1,048,575 = 131,071.875
	// From angle to CCR: angle * (131,071.875 - 26,214.375)/180 + 26,214.375
	int duty = (uint32_t) ((angle * ((131071.875 - 26214.375)/180.0)) + 26214.375);
  ledcWrite(PWM_CHANNEL, duty);
}

void resetServo()
{
  servoAngle = servoResetAngle;
  updateAngle(servoAngle);

  currentTime = 0.0; deltaTime = 0.0; previousTime = 0.0;
  errorValue = 0.0; edot = 0.0; errorIntegral = 0.0; previousError = 0.0;
  controlSignal = 0.0;

  semaphore = 0;

  delay(1000);
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