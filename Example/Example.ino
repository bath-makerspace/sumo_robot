/*
**  Author: Ian-Lim-Collab
**  Date : 30th Jan 2025
**  Version : Super Loop Architecture
**  Simple Control Logic for Dual Wheel Robot Using Controller with Bluepad Library
**  
*/

#include <Bluepad32.h>

#define MOTOR_1_PWM           14
#define MOTOR_1_A             26
#define MOTOR_1_B             27

#define MOTOR_2_PWM           32
#define MOTOR_2_A             33
#define MOTOR_2_B             25

#define DEBUG_MODE_ENABLED

#define UPDATE_RATE           150

ControllerPtr myControllers;

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
#ifdef DEBUG_MODE_ENABLED
  void dumpGamepad(ControllerPtr ctl);
#endif

void processGamepad(ControllerPtr ctl) {
    if(ctl-> brake() > 0){
      #ifdef DEBUG_MODE_ENABLED
        Serial.println("BRAKE");
      #endif
      digitalWrite(MOTOR_1_A,LOW);
      digitalWrite(MOTOR_1_B,HIGH);
      digitalWrite(MOTOR_2_A,HIGH);
      digitalWrite(MOTOR_2_B,LOW);
      // PWM output range 0 to 255
      // Brake() 0 to 1023
      // axisX() -512 to 512 
      // Divide the Brake input(10bits, 1023) by 8 to bring it down to 
      // 7 Bits, 128
      // Divide the axisX(1 sign bits + 9 bits,) by 4 to bring it down to 
      // 7 bits, 128
      // Sum the values for one wheel and subtract for the other for differential drive
      analogWrite(MOTOR_1_PWM,(ctl->brake() >> 3) - (ctl-> axisX() >> 2));
      analogWrite(MOTOR_2_PWM,(ctl->brake() >> 3) + (ctl-> axisX() >> 2));
    }else if(ctl-> throttle() > 0){
      #ifdef DEBUG_MODE_ENABLED
        Serial.println("POWER");
      #endif
      digitalWrite(MOTOR_1_A,HIGH);
      digitalWrite(MOTOR_1_B, LOW);
      digitalWrite(MOTOR_2_A, LOW);
      digitalWrite(MOTOR_2_B,HIGH);
      analogWrite(MOTOR_1_PWM,(ctl->throttle() >> 3) - (ctl-> axisX() >> 2));
      analogWrite(MOTOR_2_PWM,(ctl->throttle() >> 3) + (ctl-> axisX() >> 2));
    } else {
      #ifdef DEBUG_MODE_ENABLED
        Serial.println("STOP");
      #endif
      digitalWrite(MOTOR_1_A, LOW);
      digitalWrite(MOTOR_1_B, LOW);
      digitalWrite(MOTOR_2_A, LOW);
      digitalWrite(MOTOR_2_B, LOW);
      analogWrite(MOTOR_1_PWM,0);
      analogWrite(MOTOR_2_PWM,0);
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

  pinMode(MOTOR_1_PWM,OUTPUT);
  pinMode(MOTOR_1_A,OUTPUT);
  pinMode(MOTOR_1_B,OUTPUT);
  pinMode(MOTOR_2_PWM,OUTPUT);
  pinMode(MOTOR_2_A,OUTPUT);
  pinMode(MOTOR_2_B,OUTPUT);
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
          Serial.printf("CALLBACK: Controller is connected");
        #endif
        // Additionally, you can get certain gamepad properties like:
        // Model, VID, PID, BTAddr, flags, etc.
        ControllerProperties properties = ctl->getProperties();
        #ifdef DEBUG_MODE_ENABLED
          Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                          properties.product_id);
        #endif
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
