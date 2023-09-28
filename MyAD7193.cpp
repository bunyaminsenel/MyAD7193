#include <MyAD7193.hpp>
#include <string.h>


uint32_t registerMap[4] = {
    0x00,     // COMMUNICATIONS REGISTER
    0x080060, // MODE REGISTER
    0x000117, // CONFIGURATION REGISTER
    0x000000  // DATA REGISTER
};

// Define register sizes
constexpr int NUM_REGISTERS = 8;
uint8_t registerSize[NUM_REGISTERS] = {1, 3, 3, 3, 1, 1, 3, 3};

AD7193::AD7193()
{
  // Constructor
}

bool AD7193::begin(void)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE3));
  pinMode(SLAVE_SELECT_PIN, OUTPUT);
  digitalWrite(SLAVE_SELECT_PIN, HIGH);

  return true;
}

void AD7193::reset(void)
{
  digitalWrite(SLAVE_SELECT_PIN, LOW);

  for (int i = 0; i < 6; i++)
  {
    SPI.transfer(0xFF);
  }

  digitalWrite(SLAVE_SELECT_PIN, HIGH);
}

void AD7193::WaitForADC(void)
{
  // Beklenen durumu açıklamak için bir yorum ekleyin.
  // AD7193_RDY_STATE pininin "LOW" durumunu bekliyoruz.
  while (digitalRead(AD7193_RDY_STATE) == HIGH)
  {
    // Bekleme süresi boyunca hiçbir şey yapmayın.
    // Bu, mikrodenetleyiciyi meşgul etmemizi önler.
  }
  // ADC hazır olduğunda bu noktaya ulaşılır.

}

void AD7193::Calibrate(uint8_t mode)
{

  uint32_t oldRegValue = 0x0;

  uint32_t newRegValue = 0x0;

  oldRegValue = GetRegisterValue(AD7193_REG_MODE, 3, 1);

  oldRegValue &= ~AD7193_MODE_SEL(0x7);

  newRegValue = oldRegValue | AD7193_MODE_SEL(mode);

  digitalWrite(SLAVE_SELECT_PIN, LOW);

  SetRegisterValue(AD7193_REG_MODE, newRegValue, 3, 0); /*!< CS is not modified. */

  WaitForADC();

  digitalWrite(SLAVE_SELECT_PIN, HIGH);
}

void AD7193::CalibrateFullScale(void)
{
  Serial.print("\nInitiate Internal Calibration, starting with Zero-scale calibration...");

  digitalWrite(SLAVE_SELECT_PIN, LOW);
  delay(100);

  Serial.print("\n\nNow zero scale calibration...");

  registerMap[1] &= 0x1FFFFF; //keep all bit values except Channel bits
  registerMap[1] |= 0x800000; // internal zero scale calibration

  SetRegisterValue(1, registerMap[1], 3, 0);  // overwriting previous MODE reg setting 

  WaitForADC();

  Serial.print("\n\nNow full-scale calibration...");

  registerMap[1] &= 0x1FFFFF; //keep all bit values except Channel bits
  registerMap[1] |= 0xA00000; // internal full scale calibration

  SetRegisterValue(1, registerMap[1], 3, 0);  // overwriting previous MODE reg setting 

  WaitForADC();

  digitalWrite(SLAVE_SELECT_PIN, HIGH);
  delay(100); 
}

void AD7193::SetPGAGain(int gain)
{
  uint32_t gainSetting;
  int regAddress = AD7193_REG_CONF;

  if(gain == 1)         {gainSetting = 0x0;}
  else if (gain == 8)   {gainSetting = 0x3;}
  else if (gain == 16)  {gainSetting = 0x4;}
  else if (gain == 32)  {gainSetting = 0x5;}
  else if (gain == 64)  {gainSetting = 0x6;}
  else if (gain == 128) {gainSetting = 0x7;}
  else {
    assert(gain <= 128);
    Serial.println("\tERROR - Invalid Gain Setting - no changes made.  Valid Gain settings are 1, 8, 16, 32, 64, 128");
    return;
  }

  

  registerMap[regAddress] &= 0xFFFFF8; // keep all bit values except gain bits
  registerMap[regAddress] |= gainSetting;

  SetRegisterValue(regAddress, registerMap[regAddress], registerSize[regAddress], 1);
}

void AD7193::SetFilter(int filterRate)
{

  Serial.print("\nSetting Filter Rate Select Bits to ");
  Serial.println(filterRate);

  if (filterRate > 0x3ff)
  {
    Serial.println("\tERROR - Invalid Filter Rate Setting - no changes made.  Filter Rate is a 10-bit value");
    return;
  }

  registerMap[1] &= 0xFFFC00; // keep all bit values except filter setting bits
  registerMap[1] |= filterRate;

  SetRegisterValue(1, registerMap[1], registerSize[1], 1);
}

void AD7193::setSinc(uint8_t mode)
{
  uint32_t modeSetting;

  Serial.print("\nSetting Sinc3 Filter Select Bits to ");
  Serial.println(mode);

  if(mode == 0)         {modeSetting = 0x0;}
  else if (mode == 1)   {modeSetting = 0x8000;}
  else{
  Serial.println("\tERROR - Invalid Mode Setting - no changes made.  Valid Mode settings are 0, 1");
  return;
  }

  registerMap[1] &= 0xFF7FFF; // keep all bit values except filter setting bits
  registerMap[1] |= modeSetting;

  SetRegisterValue(1, registerMap[1], registerSize[1], 1);
}

void AD7193::SetChop(uint8_t chop)
{
  uint32_t chopSetting;
  int regAddress = AD7193_REG_CONF;

  Serial.print("\nSetting Sinc3 Filter Select Bits to ");
  Serial.println(chop);

  if(chop == 0)         {chopSetting = 0x0;}
  else if (chop == 1)   {chopSetting = 0x800000;}
  else{
  Serial.println("\tERROR - Invalid Mode Setting - no changes made.  Valid Mode settings are 0, 1");
  return;
}

  registerMap[regAddress] &= 0xEFFFFF; // keep all bit values except filter setting bits
  registerMap[regAddress] |= chopSetting;

  SetRegisterValue(regAddress, registerMap[regAddress], registerSize[regAddress], 1);
}

void AD7193::SetAverage(uint8_t average)
{
  uint32_t averageSetting;

  Serial.print("\nSetting Average Select Bits to ");
  Serial.println(average);

  if(average == 0)         {averageSetting = 0x0;}
  else if (average == 2)   {averageSetting = 0x010000;}
  else if (average == 8)  {averageSetting = 0x020000;}
  else if (average == 16)  {averageSetting = 0x030000;}
  else {
    Serial.println("\tERROR - Invalid average Setting - no changes made.  Valid Gain settings are 0, 2, 8, 16 ");
    return;
  }

  registerMap[1] &= 0xFCFFFF; // keep all bit values except filter setting bits
  registerMap[1] |= averageSetting;

  SetRegisterValue(1, registerMap[1], registerSize[1], 1);
}

void AD7193::SetChannel(uint16_t channel)
{
  // Serial.print("\nSetting Channel Select Bits to ");
  // Serial.println(channel);

  uint32_t channelBits = channel << 8;

  int regAddress = AD7193_REG_CONF;

  assert(channel <= 0x0F);

  if (channel > 0x0F)
  {
    Serial.println("\tERROR - Invalid Channel Setting - no changes made.  Valid Channel settings are 0, 1, 2, 3, 4, 5, 6, 7");
    assert(channel <= 0x7);
    return;
  }

  registerMap[2] &= 0xFF00FF; // keep all bit values except Channel bits
  registerMap[2] |= channelBits;

  SetRegisterValue(regAddress, registerMap[regAddress], registerSize[regAddress], 1);
}

void AD7193::IntitiateContinuousConversion(void)
{
  digitalWrite(SLAVE_SELECT_PIN, LOW);

  registerMap[1] &= 0x1FFFFF; //keep all bit values except Channel bits
  registerMap[1] |= 0x000000; // single conversion mode bits

  SetRegisterValue(1, registerMap[1], 3, 1); // overwriting previous MODE reg setting
}

void AD7193::AppendStatusValuetoData(void)
{
  Serial.println("\nEnabling DAT_STA Bit (appends status register to data register when reading)");

  registerMap[1] &= 0xEFFFFF; // keep all bit values except DAT_STA bit
  registerMap[1] |= 0x100000; // set DAT_STA to 1

  SetRegisterValue(1, registerMap[1], registerSize[1], 1);

  // Serial.print(" - New Mode Reg Value: ");
  // Serial.println(registerMap[1], HEX);

  registerSize[3] = 4; // change register size to 4, b/c status register is now appended
}

AD7193::AdcPack AD7193::ReadADCData(void)
{
  uint32_t read_buffer {0};
  auto buf_ptr = reinterpret_cast<uint8_t*>(&read_buffer);

  digitalWrite(SLAVE_SELECT_PIN, LOW);
  SPI.transfer(0x58);

  for (uint8_t i = 0; i < 4; ++i)
  {
    buf_ptr[i] = SPI.transfer(0xFF);
  }

  digitalWrite(SLAVE_SELECT_PIN, HIGH);

  read_buffer = __builtin_bswap32(read_buffer);
  AdcPack result {0};
  memcpy(&result, &read_buffer, sizeof(result));

  return result;
}

void AD7193::ReadADCChannel(void)
{
  int channel = 0;
  float voltage = 0;
  float mVref = 2.5;
  uint8_t mPolarity = 0;

  auto result = ReadADCData();
  uint8_t status = result.status;
  uint32_t data = result.data;

  int PGASetting = registerMap[2] & 0x000007; // keep only the PGA setting bits
  int PGAGain;

   if (PGASetting == 0) {
    PGAGain = 1;
  } else if (PGASetting == 3) {
    PGAGain = 8;
  } else if (PGASetting == 4) {
    PGAGain = 16;
  } else if (PGASetting == 5) {
    PGAGain = 32;
  } else if (PGASetting == 6) {
    PGAGain = 64;
  } else if (PGASetting == 7) {
    PGAGain = 128;
  } else {
    PGAGain = 1;
  }

  // Serial.print("PGA Gain = ");
  // Serial.println(PGAGain);

  if (mPolarity == 1)
  {
    voltage = ((double)data / 0xFFFFFF / (1)) * mVref;
  }
  if (mPolarity == 0)
  {
		voltage = (((float)data / 0x7FFFFF) - 1) * (mVref / PGAGain);
  }

 switch (status) {
  case 0:
    channel = 0; // Eğer son byte 0 ise 0. kanalı seç
    Serial.print("Channel 0:\t");
    Serial.println(voltage*1000, 5);
    break;

  case 1:
    channel = 1; // Eğer son byte 1 ise 1. kanalı seç
    Serial.print("Channel 1:\t");
    Serial.println(voltage*1000, 5);
    break;

  case 2:
    channel = 2; // Eğer son byte 2 ise 2. kanalı seç
    Serial.print("Channel 2:\t");
    Serial.println(voltage*1000, 5);
    break;

  case 3:
    channel = 3; // Eğer son byte 3 ise 3. kanalı seç
    Serial.print("Channel 3:\t");
    Serial.println(voltage*1000, 5);
    break;

  // default:
    // Eğer lastByte 0, 1 veya 2 değilse buraya düşer
    // Serial.println("Invalid lastByte value");
}
}

//==================================================================================================
// Set Register Value Function
//==================================================================================================

void AD7193::SetRegisterValue(uint8_t registerAddress, uint32_t registerValue, uint8_t bytesNumber, uint8_t modifyCS)
{

  uint8_t commandByte = 0;
  uint8_t txBuffer[4] = {0, 0, 0, 0};

  commandByte = AD7193_COMM_WRITE | AD7193_COMM_ADDR(registerAddress);

  txBuffer[0] = (registerValue >> 0) & 0x000000FF;
  txBuffer[1] = (registerValue >> 8) & 0x000000FF;
  txBuffer[2] = (registerValue >> 16) & 0x000000FF;
  txBuffer[3] = (registerValue >> 24) & 0x000000FF;
  if (modifyCS == 1)
  {
    digitalWrite(SLAVE_SELECT_PIN, LOW);
  }

  SPI.transfer(commandByte);
  while (bytesNumber > 0)
  {
    SPI.transfer(txBuffer[bytesNumber - 1]);
    bytesNumber--;
  }
  if (modifyCS == 1)
  {
    digitalWrite(SLAVE_SELECT_PIN, HIGH);
  }
}

//==================================================================================================
// Get Register Value Function
//==================================================================================================

uint32_t AD7193::GetRegisterValue(uint8_t registerAddress, uint8_t bytesNumber, uint8_t modifyCS)
{ // getregistervalue

  uint8_t receiveBuffer = 0;
  uint8_t writeByte = 0;
  uint8_t byteIndex = 0;
  uint32_t buffer = 0;

  writeByte = AD7193_COMM_READ | AD7193_COMM_ADDR(registerAddress);
  if (modifyCS == 1)
  {
    digitalWrite(SLAVE_SELECT_PIN, LOW);
  }

  SPI.transfer(writeByte);
  while (byteIndex < bytesNumber)
  {
    receiveBuffer = SPI.transfer(0);
    buffer = (buffer << 8) + receiveBuffer;
    byteIndex++;
  }

  if (modifyCS == 1)
  {
    digitalWrite(SLAVE_SELECT_PIN, HIGH);
  }

  uint8_t str[32];
  sprintf(str, "%06x", buffer);

  Serial.print("    Read Register Address: ");
  Serial.print(registerAddress, HEX);
  Serial.print(", command: ");
  Serial.print(writeByte, HEX);
  Serial.print(", recieved:0x");
  Serial.println(buffer, HEX);
  // Serial.print(" - ");
  // Serial.println(str);

  return (buffer);
}
