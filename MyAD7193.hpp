#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <assert.h>

// SPI settings
#define SLAVE_SELECT_PIN   10
#define AD7193_RDY_STATE  MISO   // pin to watch for data ready state
#define SPI_CLOCK_SPEED   SPI_CLOCK_DIV16

// Macros for setting mode bits
#define AD7193_MODE_SEL(x)      (((x) & 0x7) << 21) /*!< Operation Mode Select. */

/* AD7193 Register Map */
#define AD7193_REG_COMM         0 // Communications Register (WO, 8-bit) 
#define AD7193_REG_STAT         0 // Status Register         (RO, 8-bit) 
#define AD7193_REG_MODE         1 // Mode Register           (RW, 24-bit 
#define AD7193_REG_CONF         2 // Configuration Register  (RW, 24-bit)
#define AD7193_REG_DATA         3 // Data Register           (RO, 24/32-bit) 
#define AD7193_REG_ID           4 // ID Register             (RO, 8-bit) 
#define AD7193_REG_GPOCON       5 // GPOCON Register         (RW, 8-bit) 
#define AD7193_REG_OFFSET       6 // Offset Register         (RW, 24-bit 
#define AD7193_REG_FULLSCALE    7 // Full-Scale Register     (RW, 24-bit)

/* Communications Register Bit Designations (AD7193_REG_COMM) */
#define AD7193_COMM_WEN         (1 << 7)           // Write Enable. 
#define AD7193_COMM_WRITE       (0 << 6)           // Write Operation.
#define AD7193_COMM_READ        (1 << 6)           // Read Operation. 
#define AD7193_COMM_ADDR(x)     (((x) & 0x7) << 3) // Register Address. 
#define AD7193_COMM_CREAD       (1 << 2)           // Continuous Read of Data Register.

/**
 * @brief This class manages the AD7193 Analog-to-Digital Converter.
 *
 * @details It provides functions for configuration, calibration, and data retrieval.
 */
class AD7193
{
    public:

    struct AdcPack {
      uint32_t status : 8;
      uint32_t data : 24;
    };

    AD7193();

    // Initialize the AD7193
    bool begin(void);

    // Reset the AD7193
    void reset(void);

    // Calibrate the AD7193
    void Calibrate(uint8_t mode);

    // Calibrate the AD7193 Full scale
    void CalibrateFullScale(void);

    // Wait for the ADC to become ready
    void WaitForADC(void);

    // Set PGA gain
    void SetPGAGain(int gain);

    // Set the selected channel
    void SetChannel(uint16_t channel);

    // Set the Mode
    void setSinc(uint8_t mode);

    // Set Avariage
    void SetAverage(uint8_t average);

    void SetChop(uint8_t chop);

    // Append status value to data
    void AppendStatusValuetoData(void);

    // Read ADC data
    AdcPack ReadADCData(void);

    // Read ADC channel
    void ReadADCChannel(void);

    // Set the averaging filter rate
    void SetFilter(int filterRate);

    // Initiate continuous conversion
    void IntitiateContinuousConversion(void);

    // Set register value
    void SetRegisterValue(uint8_t registerAddress, uint32_t registerValue, uint8_t bytesNumber, uint8_t modifyCS);

    // Get register value
    uint32_t GetRegisterValue(uint8_t registerAddress, uint8_t bytesNumber, uint8_t modifyCS);

};