
/********************************************************
**  Downloaded from:                                   **
**  http://www.arduino-projekte.de                     **
********************************************************/

#include <AH_AD9850.h>

//CLK - D6, FQUP - D7,  BitData - D8, RESET - D9
//AH_AD9850(int CLK, int FQUP, int BitData, int RESET);
AH_AD9850 AD9850(6, 7, 8, 9);

void setup()
{
  //reset device
  AD9850.reset();                   //reset module
  delay(1000);
  AD9850.powerDown();               //set signal output to LOW
  
  // initialize serial communication
  Serial.begin(9600);
}

void loop(){

 //set_frequency(boolean PowerDown, byte Phase, double Freq); 
 AD9850.set_frequency(0,0,1000);    //set power=UP, phase=0, 1kHz frequency
 delay(1000); 

 AD9850.set_frequency(2500);        //set 2.5kHz frequency
 delay(1000); 
 
 AD9850 << 5000;                    //set 5kHz frequency in C++ style
 delay(1000);
 
 
 //phase test
 for (int phase=0;phase<32;phase++)   
 {
   AD9850.set_frequency(0, phase, 1000);   //change phase delay in 11.25°(2PI/32) steps
   delay(1000);
 }
 
}
