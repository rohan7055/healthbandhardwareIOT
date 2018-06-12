
/*  Pulse Sensor Amped 1.5    by Joel Murphy and Yury Gitman   http://www.pulsesensor.com

----------------------  Notes ----------------------  ----------------------
This code:
1) Blinks an LED to User's Live Heartbeat   PIN 13
2) Fades an LED to User's Live HeartBeat    PIN 5
3) Determines BPM
4) Prints All of the Above to Serial

Read Me:
https://github.com/WorldFamousElectronics/PulseSensor_Amped_Arduino/blob/master/README.md
 ----------------------       ----------------------  ----------------------
*/

#include<Wire.h>
#define PACKET_SIZE 8



// convert float to byte array
typedef union float2bytes_t   // union consists of one variable represented in a number of different ways 
{ 
  float f; 
  byte b[sizeof(float)]; 

}; 


const byte addrSlaveI2C =  21;  // I2C Slave address of this device
byte I2C_Packet[PACKET_SIZE];     // Array to hold data sent over I2C to main Arduino
bool printDataflag = false;

float2bytes_t f2b;

#define PROCESSING_VISUALIZER 1
#define SERIAL_PLOTTER  2

//  Variables
int pulsePin = 0;                // Pulse Sensor purple wire connected to analog pin 0
int tempPin=1;
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.
volatile int alert=0;
int d=2;   // to store on or off value

// SET THE SERIAL OUTPUT TYPE TO YOUR NEEDS
// PROCESSING_VISUALIZER works with Pulse Sensor Processing Visualizer
//      https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer
// SERIAL_PLOTTER outputs sensor data for viewing with the Arduino Serial Plotter
//      run the Serial Plotter at 115200 baud: Tools/Serial Plotter or Command+L
static int outputType = SERIAL_PLOTTER;


void setup(){
  pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
  pinMode(fadePin,OUTPUT);          // pin that will fade to your heartbeat!
  pinMode(tempPin, INPUT);
  pinMode(pulsePin, INPUT);
  //push button
  pinMode(2,INPUT);
//pinMode(13,OUTPUT);
  Serial.begin(115200);             // we agree to talk fast!
  Serial.println(F("Initialize wire library for slave I2C"));
  Wire.begin(addrSlaveI2C);    // Initiate the Wire library and join the I2C bus 
  Wire.onRequest(sendData); // Register a function to be called when a master requests data from this slave device. 
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS
   // IF YOU ARE POWERING The Pulse Sensor AT VOLTAGE LESS THAN THE BOARD VOLTAGE,
   // UN-COMMENT THE NEXT LINE AND APPLY THAT VOLTAGE TO THE A-REF PIN
//   analogReference(EXTERNAL);
}


//  Where the Magic Happens
void loop(){

//for reading digital output
  d=digitalRead(2);
  delay(100);
  if(d==0)
  { 
    
    alert=1;
  }else{
    alert=0;
  }

    serialOutput() ;
    if (printDataflag)
  {
    PrintData(f2b.f);
    printDataflag = false;
  }


  if (QS == true){     // A Heartbeat Was Found
                       // BPM and IBI have been Determined
                       // Quantified Self "QS" true when arduino finds a heartbeat
        fadeRate = 255;         // Makes the LED Fade Effect Happen
                                // Set 'fadeRate' Variable to 255 to fade LED with pulse
        serialOutputWhenBeatHappens();   // A Beat Happened, Output that to serial.
        QS = false;                      // reset the Quantified Self flag for next time
  }

  ledFadeToBeat();                      // Makes the LED Fade Effect Happen
  delay(20);                             //  take a break
}





void ledFadeToBeat(){
    fadeRate -= 15;                         //  set LED fade value
    fadeRate = constrain(fadeRate,0,255);   //  keep LED fade value from going into negative numbers!
    analogWrite(fadePin,fadeRate);          //  fade LED
  }

// callback for sending data
void sendData() {
   // Convert float to 4-byte array
  f2b.f = getTemperature(tempPin);
  I2C_Packet[0] = f2b.b[0];
  I2C_Packet[1] = f2b.b[1];
  I2C_Packet[2] = f2b.b[2];
  I2C_Packet[3] = f2b.b[3];
  I2C_Packet[4]=(byte)(BPM>>8);
  I2C_Packet[5]=(byte)(BPM);
  I2C_Packet[6]=(byte)(alert>>8);
  I2C_Packet[7]=(byte)(alert);

  

  Wire.write(I2C_Packet, PACKET_SIZE); 
  printDataflag = true;

}

float getTemperature(byte pin) {
  int sensorValue = analogRead(pin);
  float mv = (sensorValue/1024.0)*5000;
  return mv/10;
}




// Print out data that will be sent to the master
void PrintData(float somefloat)
{
  Serial.print("Temperature :");
  Serial.println(somefloat);
  
}

