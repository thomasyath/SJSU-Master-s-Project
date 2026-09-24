#include "BluetoothSerial.h"
#include <Adafruit_ISM330DHCX.h>

String device_name = "Vest_accel";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
Adafruit_ISM330DHCX ism330dhcx;

void setup() {
  Serial.begin(115200);
  SerialBT.begin(device_name);  //Bluetooth device name
  //SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  if (!ism330dhcx.begin_I2C()) {
    Serial.println("Failed to find ISM330DHCX chip");
    while (1) {
      delay(10);
    }
  }
  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
  ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
}
void loop() {
  sensors_event_t accel, gyro, temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);
  String dataToSend = String(millis()) + ","
    + String(accel.acceleration.x) + "," 
    + String(accel.acceleration.y) + "," 
    + String(accel.acceleration.z) + "," 
    + String(gyro.gyro.x) + "," 
    + String(gyro.gyro.y) + ","
    + String(gyro.gyro.z);
  SerialBT.println(dataToSend);
  delay(10);
}
