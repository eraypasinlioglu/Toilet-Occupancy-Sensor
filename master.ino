//MASTER
//uart:9600,0,0
//role:1
//addr:0023:00:010754
//name
//cmode=0
//bnd=0023:00:010A44 (slave)
//button pin 12

////////////////////////////////////////////////////////
//              HARDWARE CONNECTIONS                  //
////////////////////////////////////////////////////////
//  HC-05 BT MODULE        ARDUINO UNO PINS           //
//             PINS: VCC -> 5V                        //
//                   GND -> GND                       //
//                   TXD -> D2       D3               //
//                                   |(1K OHM)        //
//                   RXD ->   -------|                //
//                                   |(2K OHM)        //
//                                   |                //
//                                  GND               //
//                                                    //
//               D12---BUTTON---GND                   //
//                                                    //
//                                                    //
////////////////////////////////////////////////////////




#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(2,3);

char c;
//int cnt=0;  //to keep track how many times the button is pressed //uncomment to test the system

void setup()
{
  Serial.begin(9600);
  Bluetooth.begin(9600);

  pinMode(12,INPUT_PULLUP); //button pin
}

void loop() //main
{
  if(digitalRead(12)==LOW) //when button is pressed
  {
    //cnt++;  //count how many times the button is pressed //uncomment to test the system
    //Serial.println(cnt); //to keep track how many times the button is pressed //uncomment to test the system
    
    Bluetooth.write(33);
    delay(1200);

    c=Bluetooth.read();

    if(c==34) Serial.println("Full"); //full
    else if(c==35) Serial.println("Empty"); //empty
    else Serial.println("ERROR"); //error
  }
}


