#include "adxl362.h"


int adxl362_sample()
{
unsigned char status      = 0;
short         xAxis       = 0;
short         yAxis       = 0;
short         zAxis       = 0;
short         temperature = 0;
	
//  if(ADXL362_Init() != RT_EOK)
//  {
//	  rt_kprintf("ADXL362 Init Err!\r\n");
//	  return -RT_ERROR;
//  }
//  else
//	  rt_kprintf("ADXL362 Init Ok!\r\n");
//	return RT_EOK;
  
    /* Put the device in standby mode. */
     ADXL362_SetPowerMode(0);
    
    /* Set accelerometer's output data rate to: 12.5 Hz. */
    ADXL362_SetOutputRate(ADXL362_ODR_12_5_HZ);
    
    /* Setup the activity and inactivity detection. */
    ADXL362_SetRegisterValue(
                  ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LINK),
                  ADXL362_REG_ACT_INACT_CTL,
                  1);
    ADXL362_SetupActivityDetection(1, 30, 1);
    ADXL362_SetupInactivityDetection(1, 700, 25);
    
    /* Start the measurement process. */
    ADXL362_SetPowerMode(1);
    
    /* Clear ACT and INACT bits by reading the Status Register. */
    ADXL362_GetRegisterValue(&status,
                             ADXL362_REG_STATUS,
                             1);
    
    /* Select the 4g measurement range. */
    ADXL362_SetRange(ADXL362_RANGE_4G);  
  
  while(1)
  {
      /* Wait for the detection of an activity or inactivity or for available
    data. */
        do
        {
            ADXL362_GetRegisterValue(&status, ADXL362_REG_STATUS, 1);
            
        }while(
               ((status & ADXL362_STATUS_DATA_RDY) == 0) && 
               ((status & ADXL362_STATUS_INACT) == 0)    &&
               ((status & ADXL362_STATUS_ACT) == 0));
        
        /* Get the data from the device and display it. */    
        if(status & ADXL362_STATUS_DATA_RDY)
        {    
            ADXL362_GetXyz(&xAxis, &yAxis, &zAxis);
            temperature = ADXL362_ReadTemperature();
			rt_kprintf("x:%d y:%d z:%d temp:%d\r\n",xAxis,yAxis,zAxis,temperature);
        }
		rt_thread_mdelay(50);
	}
        
}

MSH_CMD_EXPORT(adxl362_sample,adxl362 sample);
