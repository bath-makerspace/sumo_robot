

/*
**  Author: Ian-Lim-Collab
**  Date : 30th Jan 2025
**  Version : Super Loop Architecture
**  Simple Control Logic for Dual Wheel Robot Using Controller with Bluepad Library
**  
*/

#include <Bluepad32.h>

// #define MOTOR_1_PWM           14
#define MOTOR_1_A             26
#define MOTOR_1_B             27

// #define MOTOR_2_PWM           32
#define MOTOR_2_A             33
#define MOTOR_2_B             25


#define DEBUG_MODE_ENABLED
// #define WAND_ENABLED

#define UPDATE_RATE           150

#define BUTTON_Y              8
#define BUTTON_X              4
#define BUTTON_B              2
#define BUTTON_A              1

#define MAX_PWM_VAL           255 // setup as 8bits pwm

#ifdef WAND_ENABLED
  #include <ESP32Servo.h>
  #define WAND_PIN              16
  #define MAX_SERVO_ANGLE       90
  #define MIN_SERVO_ANGLE       90
#endif

ControllerPtr myControllers;

#ifdef WAND_ENABLED
  Servo wand;
#endif 

const uint8_t CONTROLLER_ADDR[6] = {0xAC,0x8E,0xBD,0x05,0xF8,0xD1};

volatile int Motor1PwmVal = 0;
volatile int Motor2PwmVal = 0;

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
#ifdef DEBUG_MODE_ENABLED
  void dumpGamepad(ControllerPtr ctl);
#endif

void processGamepad(ControllerPtr ctl) {
  #ifdef WAND_ENABLED
    if(ctl->buttons() & BUTTON_A){
      wand.write(MAX_SERVO_ANGLE);
    } else{
      wand.write(MIN_SERVO_ANGLE);
    }
  #endif
  #ifdef DEBUG_MODE_ENABLED
  if(ctl->buttons() & BUTTON_A){
    Serial.println("Smack");
  }
  #endif

    if(ctl-> brake() > 0){
      #ifdef DEBUG_MODE_ENABLED
        Serial.println("BRAKE");
      #endif
      // PWM output range 0 to 255
      // Brake() 0 to 1023
      // axisX() -512 to 512 
      // Divide the Brake input(10bits, 1023) by 8 to bring it down to 
      // 7 Bits, 128
      // Divide the axisX(1 sign bits + 9 bits,) by 4 to bring it down to 
      // 7 bits, 128
      // Sum the values for one wheel and subtract for the other for differential drive
      
      Motor1PwmVal = (ctl->buttons() & BUTTON_Y) ? (ctl->brake() >> 2) - (ctl-> axisX() >> 2)  : (ctl->brake() >> 3) - (ctl-> axisX() >> 2);
      Motor2PwmVal = (ctl->buttons() & BUTTON_Y) ? (ctl->brake() >> 2) + (ctl-> axisX() >> 2)  : (ctl->brake() >> 3) + (ctl-> axisX() >> 2);
      Motor1PwmVal = Motor1PwmVal > MAX_PWM_VAL ? MAX_PWM_VAL : Motor1PwmVal<0 ? 0 : Motor1PwmVal;
      Motor2PwmVal = Motor2PwmVal > MAX_PWM_VAL ? MAX_PWM_VAL : Motor2PwmVal<0 ? 0 : Motor2PwmVal;
      #ifdef DEBUG_MODE_ENABLED
        Serial.printf("\n%d,%d,%d\n",(ctl->brake() >> 2),Motor1PwmVal,MAX_PWM_VAL);
      #endif
      
      analogWrite(MOTOR_1_A,0);
      analogWrite(MOTOR_1_B,Motor1PwmVal);
      analogWrite(MOTOR_2_A,Motor2PwmVal);
      analogWrite(MOTOR_2_B,0);
    }else if(ctl-> throttle() > 0){
      #ifdef DEBUG_MODE_ENABLED
        Serial.println("POWER");
      #endif
      Motor1PwmVal = (ctl->buttons() & BUTTON_Y) ? (ctl->throttle() >> 2) - (ctl-> axisX() >> 2)  : (ctl->throttle() >> 3) - (ctl-> axisX() >> 2);
      Motor2PwmVal = (ctl->buttons() & BUTTON_Y) ? (ctl->throttle() >> 2) + (ctl-> axisX() >> 2)  : (ctl->throttle() >> 3) + (ctl-> axisX() >> 2);
      Motor1PwmVal = Motor1PwmVal > MAX_PWM_VAL ? MAX_PWM_VAL : Motor1PwmVal<0 ? 0 : Motor1PwmVal;
      Motor2PwmVal = Motor2PwmVal > MAX_PWM_VAL ? MAX_PWM_VAL : Motor2PwmVal<0 ? 0 : Motor2PwmVal;

      analogWrite(MOTOR_1_A,Motor1PwmVal);
      analogWrite(MOTOR_1_B,0);
      analogWrite(MOTOR_2_A,0);
      analogWrite(MOTOR_2_B,Motor2PwmVal);
    } else {
      #ifdef DEBUG_MODE_ENABLED
        Serial.println("STOP");
      #endif
      Motor1PwmVal = 0;
      Motor2PwmVal = 0;
      analogWrite(MOTOR_1_A,0);
      analogWrite(MOTOR_1_B,0);
      analogWrite(MOTOR_2_A,0);
      analogWrite(MOTOR_2_B,0);
    }
    #ifdef DEBUG_MODE_ENABLED
      dumpGamepad(ctl);
    #endif
}

void setup() {
  #ifdef DEBUG_MODE_ENABLED
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  #endif
  const uint8_t* addr = BP32.localBdAddress();
  #ifdef DEBUG_MODE_ENABLED
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  #endif

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  // pinMode(MOTOR_1_PWM,OUTPUT);
  pinMode(MOTOR_1_A,OUTPUT);
  pinMode(MOTOR_1_B,OUTPUT);
  // pinMode(MOTOR_2_PWM,OUTPUT);
  pinMode(MOTOR_2_A,OUTPUT);
  pinMode(MOTOR_2_B,OUTPUT);

  #ifdef WAND_ENABLED
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    wand.setPeriodHertz(50);
    wand.attach(WAND_PIN,1000,2000);
  #endif 
}

void loop() {
  // put your main code here, to run repeatedly:
  bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
  delay(UPDATE_RATE);
}

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    if (myControllers == nullptr) {
        #ifdef DEBUG_MODE_ENABLED
          Serial.printf("CALLBACK: Controller is connected,");
        #endif
        // Additionally, you can get certain gamepad properties like:
        // Model, VID, PID, BTAddr, flags, etc.
        ControllerProperties properties = ctl->getProperties();
        #ifdef DEBUG_MODE_ENABLED
          Serial.printf("model: %s, VID=0x%04x, PID=0x%04x, ADDR[%02X:%02X:%02X:%02X:%02X:%02X]\n", 
              ctl->getModelName().c_str(), properties.vendor_id, properties.product_id, 
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5]);
          // Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
          //                 properties.product_id);
        #endif
        for(int i =0 ; i < 6; i++){
          if(properties.btaddr[i] != CONTROLLER_ADDR[i]){
            #ifdef DEBUG_MODE_ENABLED
              Serial.printf("Controller Address[%02X:%02X:%02X:%02X:%02X:%02X] does not match with predefined [%02X:%02X:%02X:%02X:%02X:%02X]\n",
                properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
                properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
                CONTROLLER_ADDR[0], CONTROLLER_ADDR[1], CONTROLLER_ADDR[2],
                CONTROLLER_ADDR[3], CONTROLLER_ADDR[4], CONTROLLER_ADDR[5]);
            #endif
            ctl->disconnect(); 
            return;
          }
        }
        myControllers = ctl;
        foundEmptySlot = true;
    }
    if (!foundEmptySlot) {
        #ifdef DEBUG_MODE_ENABLED
          Serial.println("CALLBACK: Controller connected, but could not found empty slot");
        #endif
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    if (myControllers == ctl) {
        #ifdef DEBUG_MODE_ENABLED
          Serial.printf("CALLBACK: Controller disconnected");
        #endif
        myControllers = nullptr;
        foundController = true;
    }

    if (!foundController) {
      #ifdef DEBUG_MODE_ENABLED
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
      #endif
    }
}

void processControllers() {
    if (myControllers && myControllers->isConnected() && myControllers->hasData()) {
        if (myControllers->isGamepad()) {
            processGamepad(myControllers);
        } else {
          #ifdef DEBUG_MODE_ENABLED
            Serial.println("Unsupported controller");
          #endif
        }
    }
}

#ifdef DEBUG_MODE_ENABLED
  void dumpGamepad(ControllerPtr ctl) {
      Serial.printf(
          "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
          "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
          ctl->index(),        // Controller Index
          ctl->dpad(),         // D-pad
          ctl->buttons(),      // bitmask of pressed buttons
          ctl->axisX(),        // (-511 - 512) left X Axis
          ctl->axisY(),        // (-511 - 512) left Y axis
          ctl->axisRX(),       // (-511 - 512) right X axis
          ctl->axisRY(),       // (-511 - 512) right Y axis
          ctl->brake(),        // (0 - 1023): brake button
          ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
          ctl->miscButtons(),  // bitmask of pressed "misc" buttons
          ctl->gyroX(),        // Gyro X
          ctl->gyroY(),        // Gyro Y
          ctl->gyroZ(),        // Gyro Z
          ctl->accelX(),       // Accelerometer X
          ctl->accelY(),       // Accelerometer Y
          ctl->accelZ()        // Accelerometer Z
      );
  }
#endif
