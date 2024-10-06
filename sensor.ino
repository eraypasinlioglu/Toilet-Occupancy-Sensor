//SLAVE
//uart:9600,0,0
//role:0
//addr:0023:00:010A44



/////////////////////////////////////////////////////////////////////////////////////////////
//                                   HARDWARE CONNECTIONS                                  //
/////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                         //
//  HC-05 BT MODULE        ARDUINO NANO PINS       VL53L0X SENSOR      ARDUINO NANO PINS   //
//             PINS: VCC -> 5V                             PINS:  VIN -> 3V3               //
//                   GND -> GND                                   GND -> GND               //
//                   TXD -> D2       D3                           SCL -> A5                //
//                                   |(1K OHM)                    SDA -> A4                //
//                   RXD ->   -------|                                                     //
//                                   |(2K OHM)                                             //
//                                   |                                                     //
//                                  GND                                                    //
//                                                                                         //
//      Arduino Nano is programmed via USB-TTL UART module                                 //
//              TXD pin <- RX                                                              //
//              RXD pin <- TX                                                              //
//              GND pin <- GND                                                             //
//              5V pin  <- VCC                                                             //
//                                                                                         //
/////////////////////////////////////////////////////////////////////////////////////////////


#include <avr/sleep.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>

//#define LONG_RANGE    //uncomment to use long range mode of sensor
//#define HIGH_SPEED    //uncomment to get faster results
#define HIGH_ACCURACY   //uncomment to get more accurate results
#define interruptPin 2

VL53L0X sensor;
SoftwareSerial Bluetooth(2, 3);
int dist=0;

void setup()
{
  Wire.begin();
  pinMode(interruptPin, INPUT);
  Serial.begin(9600);
  Bluetooth.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
}

void sleep() //sleep mode function
{
  sleep_enable();
  attachInterrupt(0, wakeup, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  digitalWrite(LED_BUILTIN, LOW);
  sleep_cpu();
}
void wakeup() //function to wake Arduino up
{
  sleep_disable();
  detachInterrupt(0);
}

void config() //configuration mode
{
  //This function aims to configure the sensor to the distance of lock
  //To get most accurate results, configure while the lock is half locked
  digitalWrite(LED_BUILTIN, HIGH); //indicator
  while(dist==0)
  {
    if(Bluetooth.read()==33) dist=sensor.readRangeSingleMillimeters();
  }
  //Serial.print("dist: ");Serial.println(dist); //check if sensor gets a distance value
}

void loop() //main
{
  if(dist==0) config(); //configure mode when the module is powered first time
  delay(500);
  
  if(digitalRead(2)!=LOW) sleep(); //sleep when module is not used
  //WARNING: bluetooth module still works when arduino is sleeping
  digitalWrite(LED_BUILTIN, HIGH); //indicator    
  //Serial.println(sensor.readRangeSingleMillimeters()); //check if sensor gets a distance value

  if (sensor.readRangeSingleMillimeters() > dist) Bluetooth.write(34);
  else Bluetooth.write(35);
  delay(500);
}
