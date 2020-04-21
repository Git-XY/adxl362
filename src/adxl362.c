/***************************************************************************//**
 *   @file   ADXL362.c
 *   @brief  Implementation of ADXL362 Driver.
 *   @author DNechita(Dan.Nechita@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 797
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "adxl362.h"
#include <rtdevice.h>
#include <board.h>
/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
#define SPI_BUS_NAME				"spi2"
#define SPI_ADXL362_DEVICE_NAME 	"spi21"

#define CS_PIN       GET_PIN(A, 10)
//#define INT1_PIN   GET_PIN(C, 6)
//#define INT2_PIN   GET_PIN(C, 8)

//#ifdef  ADXL362_DEBUG
//#define ADXL362_TRACE         rt_kprintf
//#else
//#define ADXL362_TRACE(...)
//#endif

static struct rt_spi_device spi_dev_adxl362;

/***************************************************************************//**
 * @brief Initializes communication with the device and checks if the part is
 *        present by reading the device id.
 *
 * @return 1 if the initialization was successful and the device is present,
 *         0 if an error occurred.
*******************************************************************************/
rt_err_t ADXL362_Init(void)
{	
	rt_err_t res;
	rt_base_t pin = CS_PIN;
    rt_pin_mode(CS_PIN, PIN_MODE_OUTPUT);

    res = rt_spi_bus_attach_device(&spi_dev_adxl362, SPI_ADXL362_DEVICE_NAME, SPI_BUS_NAME, (void *)&pin);
    if (res != RT_EOK)
        return res;
	
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz     = 2.5* 1000 *1000; /* 5M,SPI max 5MHz,ADXL362 4-wire spi */
        rt_spi_configure(&spi_dev_adxl362, &cfg);
    }
	
	rt_uint8_t regValue;
	ADXL362_GetRegisterValue(&regValue, ADXL362_REG_PARTID, 1);
	rt_kprintf("regValue:%x\r\n",regValue);
    if(regValue != ADXL362_PART_ID)
		return -RT_ERROR;
    return RT_EOK;
}

INIT_DEVICE_EXPORT(ADXL362_Init);
/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerValue - Data value to write.
 * @param registerAddress - Address of the register.
 * @param bytesNumber - Number of bytes. Accepted values: 0 - 1.
 *
 * @return None.
*******************************************************************************/
rt_err_t ADXL362_SetRegisterValue(unsigned short registerValue,
                              unsigned char  registerAddress,
                              unsigned char  bytesNumber)
{
    unsigned char buffer[4] = {0, 0, 0, 0};

    buffer[0] = ADXL362_WRITE_REG;
    buffer[1] = registerAddress;
    buffer[2] = (registerValue & 0x00FF);
    buffer[3] = (registerValue >> 8);
	
	if (rt_spi_send(&spi_dev_adxl362, buffer, bytesNumber + 2) != bytesNumber + 2)
        return -RT_ERROR;
    else
        return RT_EOK;
}

/***************************************************************************//**
 * @brief Performs a burst read of a specified number of registers.
 *
 * @param pReadData - The read values are stored in this buffer.
 * @param registerAddress - The start address of the burst read.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return None.
*******************************************************************************/
rt_err_t ADXL362_GetRegisterValue(unsigned char* pReadData,
                              unsigned char  registerAddress,
                              unsigned char  bytesNumber)
{
    unsigned char buffer[2];
    
    buffer[0] = ADXL362_READ_REG;
    buffer[1] = registerAddress;

	if(rt_spi_send(&spi_dev_adxl362,buffer,2) != 2)
        return -RT_ERROR;
	
	if(rt_spi_recv(&spi_dev_adxl362,pReadData,bytesNumber) != bytesNumber)
	    return -RT_ERROR;

   return RT_EOK;
}

/***************************************************************************//**
 * @brief Reads multiple bytes from the device's FIFO buffer.
 *
 * @param pBuffer - Stores the read bytes.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return None.
*******************************************************************************/
rt_err_t ADXL362_GetFifoValue(unsigned char* pBuffer, unsigned int bytesNumber)
{
    unsigned char buf = ADXL362_WRITE_FIFO;

	if(rt_spi_send(&spi_dev_adxl362,&buf,1) != 1)
        return -RT_ERROR;
	
	if(rt_spi_recv(&spi_dev_adxl362,pBuffer,bytesNumber) != bytesNumber )
	    return -RT_ERROR;

    return RT_EOK;
}

/***************************************************************************//**
 * @brief Resets the device via SPI communication bus.
 *
 * @return None.
*******************************************************************************/
void ADXL362_SoftwareReset(void)
{
    ADXL362_SetRegisterValue(ADXL362_RESET_KEY, ADXL362_REG_SOFT_RESET, 1);
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *                  Example: 0 - standby mode.
 *		                     1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetPowerMode(unsigned char pwrMode)
{
    unsigned char oldPowerCtl = 0;
    unsigned char newPowerCtl = 0;

    ADXL362_GetRegisterValue(&oldPowerCtl, ADXL362_REG_POWER_CTL, 1);
    newPowerCtl = oldPowerCtl & ~ADXL362_POWER_CTL_MEASURE(0x3);
    newPowerCtl = newPowerCtl |
                  (pwrMode * ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON));
    ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
}

/***************************************************************************//**
 * @brief Selects the measurement range.
 *
 * @param gRange - Range option.
 *                  Example: ADXL362_RANGE_2G  -  +-2 g
 *                           ADXL362_RANGE_4G  -  +-4 g
 *                           ADXL362_RANGE_8G  -  +-8 g
 *                           
 * @return None.
*******************************************************************************/
void ADXL362_SetRange(unsigned char gRange)
{
    unsigned char oldFilterCtl = 0;
    unsigned char newFilterCtl = 0;

    ADXL362_GetRegisterValue(&oldFilterCtl, ADXL362_REG_FILTER_CTL, 1);
    newFilterCtl = oldFilterCtl & ~ADXL362_FILTER_CTL_RANGE(0x3);
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_RANGE(gRange);
    ADXL362_SetRegisterValue(newFilterCtl, ADXL362_REG_FILTER_CTL, 1);
}

/***************************************************************************//**
 * @brief Selects the Output Data Rate of the device.
 *
 * @param outRate - Output Data Rate option.
 *                  Example: ADXL362_ODR_12_5_HZ  -  12.5Hz
 *                           ADXL362_ODR_25_HZ    -  25Hz
 *                           ADXL362_ODR_50_HZ    -  50Hz
 *                           ADXL362_ODR_100_HZ   -  100Hz
 *                           ADXL362_ODR_200_HZ   -  200Hz
 *                           ADXL362_ODR_400_HZ   -  400Hz
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetOutputRate(unsigned char outRate)
{
    unsigned char oldFilterCtl = 0;
    unsigned char newFilterCtl = 0;

    ADXL362_GetRegisterValue(&oldFilterCtl, ADXL362_REG_FILTER_CTL, 1);
    newFilterCtl = oldFilterCtl & ~ADXL362_FILTER_CTL_ODR(0x7);
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_ODR(outRate);
    ADXL362_SetRegisterValue(newFilterCtl, ADXL362_REG_FILTER_CTL, 1);
}

/***************************************************************************//**
 * @brief Reads the 3-axis raw data from the accelerometer.
 *
 * @param x - Stores the X-axis data(as two's complement).
 * @param y - Stores the X-axis data(as two's complement).
 * @param z - Stores the X-axis data(as two's complement).
 *
 * @return None.
*******************************************************************************/
void ADXL362_GetXyz(short* x, short* y, short* z)
{
    unsigned char xyzValues[6] = {0, 0, 0, 0, 0, 0};

    ADXL362_GetRegisterValue(xyzValues, ADXL362_REG_XDATA_L, 6);
    *x = ((short)xyzValues[1] << 8) + xyzValues[0];
    *y = ((short)xyzValues[3] << 8) + xyzValues[2];
    *z = ((short)xyzValues[5] << 8) + xyzValues[4];
}

/***************************************************************************//**
 * @brief Reads the temperature of the device.
 *
 * @return tempCelsius - The value of the temperature(degrees Celsius).
*******************************************************************************/
float ADXL362_ReadTemperature(void)
{
    unsigned char rawTempData[2] = {0, 0};
    short         signedTemp     = 0;
    float         tempCelsius    = 0;

    ADXL362_GetRegisterValue(rawTempData, ADXL362_REG_TEMP_L, 2);
    signedTemp = (short)(rawTempData[1] << 8) + rawTempData[0];
    tempCelsius = (float)signedTemp * 0.065f;
    
    return tempCelsius;
}

/***************************************************************************//**
 * @brief Configures the FIFO feature.
 *
 * @param mode - Mode selection.
 *               Example: ADXL362_FIFO_DISABLE        -  FIFO is disabled.
 *                        ADXL362_FIFO_OLDEST_SAVED   -  Oldest saved mode.
 *                        ADXL362_FIFO_STREAM         -  Stream mode.
 *                        ADXL362_FIFO_TRIGGERED      -  Triggered mode.
 * @param waterMarkLvl - Specifies the number of samples to store in the FIFO.
 * @param enTempRead - Store Temperature Data to FIFO.
 *                     Example: 1 - temperature data is stored in the FIFO
 *                                  together with x-, y- and x-axis data.
 *                              0 - temperature data is skipped.
 *
 * @return None.
*******************************************************************************/
void ADXL362_FifoSetup(unsigned char  mode,
                       unsigned short waterMarkLvl,
                       unsigned char  enTempRead)
{
    unsigned char writeVal = 0;

    writeVal = ADXL362_FIFO_CTL_FIFO_MODE(mode) |
               (enTempRead * ADXL362_FIFO_CTL_FIFO_TEMP) |
               ADXL362_FIFO_CTL_AH;
    ADXL362_SetRegisterValue(writeVal, ADXL362_REG_FIFO_CTL, 1);
    ADXL362_SetRegisterValue(waterMarkLvl, ADXL362_REG_FIFO_SAMPLES, 2);
}

/***************************************************************************//**
 * @brief Configures activity detection.
 *
 * @param refOrAbs - Referenced/Absolute Activity Select.
 *                   Example: 0 - absolute mode.
 *                            1 - referenced mode.
 * @param threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * @param time - 8-bit value written to the activity timer register. The amount
 *               of time (in seconds) is: time / ODR, where ODR - is the output 
 *               data rate.
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetupActivityDetection(unsigned char  refOrAbs,
                                    unsigned short threshold,
                                    unsigned char  time)
{
    unsigned char oldActInactReg = 0;
    unsigned char newActInactReg = 0;

    /* Configure motion threshold and activity timer. */
    ADXL362_SetRegisterValue((threshold & 0x7FF), ADXL362_REG_THRESH_ACT_L, 2);
    ADXL362_SetRegisterValue(time, ADXL362_REG_TIME_ACT, 1);
    /* Enable activity interrupt and select a referenced or absolute
       configuration. */
    ADXL362_GetRegisterValue(&oldActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
    newActInactReg = oldActInactReg & ~ADXL362_ACT_INACT_CTL_ACT_REF;
    newActInactReg |= ADXL362_ACT_INACT_CTL_ACT_EN |
                     (refOrAbs * ADXL362_ACT_INACT_CTL_ACT_REF);
    ADXL362_SetRegisterValue(newActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
}

/***************************************************************************//**
 * @brief Configures inactivity detection.
 *
 * @param refOrAbs - Referenced/Absolute Inactivity Select.
 *                   Example: 0 - absolute mode.
 *                            1 - referenced mode.
 * @param threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * @param time - 16-bit value written to the inactivity timer register. The 
 *               amount of time (in seconds) is: time / ODR, where ODR - is the 
 *               output  data rate.
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetupInactivityDetection(unsigned char  refOrAbs,
                                      unsigned short threshold,
                                      unsigned short time)
{
    unsigned char oldActInactReg = 0;
    unsigned char newActInactReg = 0;
    
    /* Configure motion threshold and inactivity timer. */
    ADXL362_SetRegisterValue((threshold & 0x7FF),
                              ADXL362_REG_THRESH_INACT_L,
                              2);
    ADXL362_SetRegisterValue(time, ADXL362_REG_TIME_INACT_L, 2);
    /* Enable inactivity interrupt and select a referenced or absolute
       configuration. */
    ADXL362_GetRegisterValue(&oldActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
    newActInactReg = oldActInactReg & ~ADXL362_ACT_INACT_CTL_INACT_REF;
    newActInactReg |= ADXL362_ACT_INACT_CTL_INACT_EN |
                     (refOrAbs * ADXL362_ACT_INACT_CTL_INACT_REF);
    ADXL362_SetRegisterValue(newActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
}
