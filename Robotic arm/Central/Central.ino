#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_APDS9960.h>
#include <Servo.h>

// Sampling rate (in Hz) and period (in seconds)
const float samplingRate = 50.0;  // adjust as needed

int potpin = A0;  // analog pin used to connect the potentiometer
int val;   
// Complementary filter alpha value (adjust as needed)
const float alpha = 0.98;

// Variables to store orientation angles
float roll = 0.0;
float pitch = 0.0;

Servo rollServo; // Servo object for roll control
Servo pitchServo; // Servo object for pitch control
int rollPos = 0;
int pitchPos = 0;
float xAccel, yAccel, zAccel;
float xGyro, yGyro, zGyro;
// variables for button
const int buttonPin = 5;  // the number of the pushbutton pin
int buttonState = 0;  // variable for reading the pushbutton status
int buttonState2 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor!");
  }
  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central - LED control");

  // start scanning for peripherals
  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "Robot") {
      return;
    }

    // stop scanning
    BLE.stopScan();

    controlLed(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
  }
}

void controlLed(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");
  BLECharacteristic ledCharacteristic2 = peripheral.characteristic("19b10002-e8f2-537e-4f6c-d104768a1214");
  BLECharacteristic ledCharacteristic3 = peripheral.characteristic("19b10003-e8f2-537e-4f6c-d104768a1214");
  BLECharacteristic ledCharacteristic4 = peripheral.characteristic("19b10004-e8f2-537e-4f6c-d104768a1214");

  if (!ledCharacteristic) {
    Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  } else if (!ledCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    // while the peripheral is connected
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(xAccel, yAccel, zAccel);
    IMU.readGyroscope(xGyro, yGyro, zGyro);

    // Calculate roll and pitch angles using accelerometer
    float accelRoll = atan2(yAccel, zAccel) * 180.0 / M_PI;
    float accelPitch = atan2(-xAccel, sqrt(yAccel * yAccel + zAccel * zAccel)) * 180.0 / M_PI;

    // // Integrate gyroscope data to update roll and pitch angles
    // roll += xGyro * dt;
    // pitch += yGyro * dt;

    // // Apply complementary filter to combine accelerometer and gyroscope data
    // roll = alpha * roll + (1 - alpha) * accelRoll;
    // pitch = alpha * pitch + (1 - alpha) * accelPitch;

    // Map roll and pitch angles to servo positions
    rollPos = map(accelRoll, -90, 90, 0, 180); // Map roll angle to servo position (0-180)
    pitchPos = map(accelPitch, -90, 90, 0, 180); // Map pitch angle to servo position (0-180)
    
  } 
    
    //  int gesture = APDS.readGesture();
    //   switch (gesture) {
    //     case GESTURE_UP:
    //       buttonState = HIGH;
    //       Serial.println("Detected UP gesture");
    //       break;
    //     case GESTURE_DOWN:
    //       buttonState = LOW;
    //       Serial.println("Detected Down gesture");
    //       break;
    //     case GESTURE_LEFT:
    //       buttonState2 = HIGH;
    //       Serial.println("Detected Left gesture");
    //       break;
    //     case GESTURE_RIGHT:
    //       buttonState2 = LOW;
    //       Serial.println("Detected Right gesture");
    //       break;
    //     default:
    //       break;
    // }
    if (APDS.proximityAvailable()) {
    int proximity = APDS.readProximity();
    Serial.println(proximity);                   // sets the servo position according to the scaled value
    if (proximity == 0) {
      buttonState = LOW;
      Serial.println(buttonState);                  // sets the servo position according to the scaled value

    }
    else 
    {
      buttonState = 1;
      Serial.println(buttonState);                   // sets the servo position according to the scaled value

    }
    }
    
    if ( buttonState == HIGH ) 
    {
      ledCharacteristic.writeValue((byte)rollPos);
      ledCharacteristic2.writeValue((byte)pitchPos);
      val = analogRead(potpin);
      Serial.println(val);
            // reads the value of the potentiometer (value between 0 and 1023)
      val = map(val, 0, 1023, 0, 180);     // scale it for use with the servo (value between 0 and 180)
      ledCharacteristic4.writeValue((byte)val);
    }
    else 
    {
      ledCharacteristic.writeValue((byte)rollPos);
      ledCharacteristic3.writeValue((byte)pitchPos);
      val = analogRead(potpin);
      Serial.println(val);
            // reads the value of the potentiometer (value between 0 and 1023)
      val = map(val, 0, 1023, 0, 180);     // scale it for use with the servo (value between 0 and 180)
      ledCharacteristic4.writeValue((byte)val);
    }   
  }
  Serial.println("Peripheral disconnected");
}
