#include <ArduinoBLE.h>
#include <Servo.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service
Servo rollServo;
Servo pitchServo; 
Servo rollServo2;
Servo pitchServo2; 
int ser1, ser2, ser3, ser4;
int pos1 = 90, pos2 = 90, pos3 = 90, pos4 = 90;

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic switchCharacteristic2("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic switchCharacteristic3("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic switchCharacteristic4("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = LED_BUILTIN; // pin to use for the LED

void setup() {
  Serial.begin(9600);
  rollServo.attach(9); 
  pitchServo.attach(10); 
  rollServo2.attach(11); 
  pitchServo2.attach(12); 

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Robot");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);
  ledService.addCharacteristic(switchCharacteristic2);
  ledService.addCharacteristic(switchCharacteristic3);
  ledService.addCharacteristic(switchCharacteristic4);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(60);
  switchCharacteristic2.writeValue(60);
  switchCharacteristic3.writeValue(60);
  switchCharacteristic4.writeValue(90);

  // start advertising
  BLE.advertise();
  Serial.println("BLE LED Peripheral");
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
          if (switchCharacteristic.written())
          {
            ser1 = switchCharacteristic.value();
          }  
          else if (switchCharacteristic2.written())
          {
            ser2 = switchCharacteristic2.value();
          }  
          else if (switchCharacteristic3.written())
          {
            ser3 = switchCharacteristic3.value();
          }  
          else if (switchCharacteristic4.written())
          {
            ser4 = switchCharacteristic4.value();
          }

          if (ser1 >= 120 && ser1<=180 && pos1 <= 180) 
          {
            Serial.println("Right");
            rollServo.write(pos1+1);
            pos1 = pos1 + 1;
            delay(30);
          }
          else if (ser1 <= 60 && ser1 >= 0 && pos1 >= 0) 
          {
            Serial.println("Left");
             rollServo.write(pos1-1);
            pos1 = pos1 - 1;
            delay(30);
          }

          if (ser2 >= 120 && ser2<=180 && pos2 >= 60 ) 
          {
            Serial.println("Left");
            pitchServo.write(pos2-1);
            pos2 = pos2 - 1;
            delay(30);
          }

          else if (ser2 <= 60 && ser2 >= 0 && pos2 <= 130) 
          {
          
            Serial.println("Right");
            pitchServo.write(pos2+1);
            pos2 = pos2 + 1;
            delay(30);
          }

          if (ser3 <= 60 && ser3 >= 0 && pos3 >= 80) 
          {
            Serial.println("Left");
            rollServo2.write(pos3-1);
            pos3 = pos3 - 1;
            delay(30);
          }

          else if (ser3 >= 120 && ser3<=180 && pos3 <= 150) 
          {
           
            Serial.println("Right");
            rollServo2.write(pos3+1);
            pos3 = pos3 + 1;
            delay(30);
          }         
         pitchServo2.write(switchCharacteristic4.value());
         Serial.println(switchCharacteristic4.value());     
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

