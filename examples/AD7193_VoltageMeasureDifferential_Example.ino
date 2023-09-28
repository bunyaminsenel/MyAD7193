#include <SPI.h>
#include <MyAD7193.hpp>

AD7193 AD7193;

void setup() {
  Serial.begin(115200);

  AD7193.begin();
/*
  --------------------------------------------------------------
  |   G2   |   G1   |   G0   |   Gain   |   ADC Input Range   |
  --------------------------------------------------------------
  |   0    |   0    |   0    |    1    |       ±2.5 V        |
  |   0    |   0    |   1    | Reserved|         -           |
  |   0    |   1    |   0    | Reserved|         -           |
  |   0    |   1    |   1    |    8    |      ±312.5 mV      |
  |   1    |   0    |   0    |   16    |      ±156.2 mV      |
  |   1    |   0    |   1    |   32    |      ±78.125 mV     |
  |   1    |   1    |   0    |   64    |      ±39.06 mV      |
  |   1    |   1    |   1    |  128    |      ±19.53 mV      |
  --------------------------------------------------------------
*/
  AD7193.SetPGAGain(64);

/* Reset all registers to power-on values. 
   After reset, wait for 500 µs before using the serial interface. */
  AD7193.reset();

  delayMicroseconds(500);

/*
--------------------------------------------------------------
|  CH3   |   CH2  |   CH1  |   CH0   |   Function Input    |
--------------------------------------------------------------
|   0    |   0    |   0    |    0    |       0X0           |
|   0    |   0    |   0    |    1    |       0X1           |
|   0    |   0    |   1    |    0    |       0X2           |
|   0    |   1    |   0    |    0    |       0X4           |
|   1    |   0    |   0    |    0    |       0X8           |
|   0    |   0    |   1    |    1    |       0X3           |
|   0    |   1    |   0    |    1    |       0X5           |
|   0    |   1    |   1    |    0    |       0X6           |
|   0    |   1    |   1    |    1    |       0X7           |
|   1    |   0    |   0    |    1    |       0X9           |
|   1    |   0    |   1    |    0    |       0XA           |
|   1    |   1    |   0    |    0    |       0XC           |
|   1    |   1    |   1    |    1    |       0XF           |
--------------------------------------------------------------
*/
  AD7193.SetChannel(0xC);

/* DAT_STA bit: Transmit status with data for channel identification. */
  AD7193.AppendStatusValuetoData();

  AD7193.CalibrateFullScale();

/* Continuous conversion mode: ADC continuously converts and outputs data when enabled. */
  AD7193.IntitiateContinuousConversion();

/*
  ---------------------------
  |  sinc4    |  Sinc3      
  --------------------------
    0         |  1        
  ---------------------------
*/
  AD7193.setSinc(0); 

/*
  -----------------------------------------------
  |   AVG1   |   AVG0   |   Average              |
  -----------------------------------------------|
  |    0     |    0     |  No averaging 0        |
  |    0     |    1     |  Average by 2          |
  |    1     |    0     |  Average by 8          |
  |    1     |    1     |  Average by 16         |
  ------------------------------------------------
*/
  AD7193.SetAverage(0);

/*
  -------------------------------
  |   Chop Enable   |   Status   |
  -------------------------------
  |       0         |  Disabled  |
  |       1         |  Enabled   |
  -------------------------------
*/
  AD7193.SetChop(0);

 // Debug - Check MODE REGISTER 
  AD7193.GetRegisterValue(1,3,1);

  // Debug - Check CONFIGURATION REGISTER
  AD7193.GetRegisterValue(2,3,1);

  Serial.println("\nBegin AD7193 conversion");
}


void loop() {
  AD7193.ReadADCChannel();
  delay(10);
}
