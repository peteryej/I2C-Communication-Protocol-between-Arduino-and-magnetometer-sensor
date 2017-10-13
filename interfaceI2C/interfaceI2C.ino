//--------------------------------------------------------------------------------------------------------------------------
// Interface a LSM303C IMU using I2C 
// Author: Peter Ye
// Date:  10-7-2017
// Note: Took out the SPI interface functions in the SparkFunLSM303C library because some macro functions doesn't work with 
//       Arduino Due board. 
//       Used Wire1 interface instead of the Wire interface.
//---------------------------------------------------------------------------------------------------------------------------
//
#include "Wire.h"
#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"
#include "LSM303CTypes.h"
#include <FreeRTOS_ARM.h>


//-----------------------------------------------------------------------------------------------------------------------------
//initiate a LSM303C variable
LSM303C myIMU;
QueueHandle_t Read_Queue_Handle;

#define WhoAmIvalue  0x3d //WhoAmIvalue for magnetometer is looked up from datasheet as 0x3d

//set the magnetometer data type
struct Magdata {
  float valueX;
  float valueY;
  float valueZ;
};

void setup() {
  // put your setup code here, to run once:
  portBASE_TYPE s1, s2;
  
  //initiate serial port
  SerialUSB.begin(115200);
  while(!SerialUSB) {}
  
  //initiate IMU, many parameter options are listed below for easier configuration  
  if (myIMU.begin(
                ///// Interface mode options
                  //MODE_SPI,
                  MODE_I2C,

                ///// Magnetometer output data rate options
                  //MAG_DO_0_625_Hz,
                  //MAG_DO_1_25_Hz,
                  //MAG_DO_2_5_Hz,
                  //MAG_DO_5_Hz,
                  //MAG_DO_10_Hz,
                  //MAG_DO_20_Hz,
                  MAG_DO_40_Hz,
                  //MAG_DO_80_Hz,

                ///// Magnetic field full scale options
                  //MAG_FS_4_Ga,
                  //MAG_FS_8_Ga,
                  //MAG_FS_12_Ga,
                  MAG_FS_16_Ga,
                  
                ///// Magnetometer block data updating options
                  //MAG_BDU_DISABLE,
                  MAG_BDU_ENABLE,

                ///// Magnetometer X/Y axes ouput data rate
                  //MAG_OMXY_LOW_POWER,
                  //MAG_OMXY_MEDIUM_PERFORMANCE,
                  MAG_OMXY_HIGH_PERFORMANCE,
                  //MAG_OMXY_ULTRA_HIGH_PERFORMANCE,

                ///// Magnetometer Z axis ouput data rate
                  //MAG_OMZ_LOW_PW,
                  //MAG_OMZ_MEDIUM_PERFORMANCE,
                  MAG_OMZ_HIGH_PERFORMANCE,
                  //MAG_OMZ_ULTRA_HIGH_PERFORMANCE,

                ///// Magnetometer run mode
                  MAG_MD_CONTINUOUS,
                  //MAG_MD_SINGLE,
                  //MAG_MD_POWER_DOWN_1,
                  //MAG_MD_POWER_DOWN_2,

                ///// Acceleration full scale
                  ACC_FS_2g,
                  //ACC_FS_4g,
                  //ACC_FS_8g,

                ///// Accelerometer block data updating
                  //ACC_BDU_DISABLE,
                  ACC_BDU_ENABLE,

                ///// Enable X, Y, and/or Z axis
                  //ACC_DISABLE_ALL,
                  //ACC_X_ENABLE,
                  //ACC_Y_ENABLE,
                  //ACC_Z_ENABLE,
                  //ACC_X_ENABLE|ACC_Y_ENABLE,
                  //ACC_X_ENABLE|ACC_Z_ENABLE,
                  //ACC_Y_ENABLE|ACC_Z_ENABLE,
                  ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE,

                ///// Accelerometer output data rate
                  //ACC_ODR_POWER_DOWN
                  //ACC_ODR_10_Hz
                  //ACC_ODR_50_Hz
                  ACC_ODR_100_Hz
                  //ACC_ODR_200_Hz
                  //ACC_ODR_400_Hz
                  //ACC_ODR_800_Hz
                ) != IMU_SUCCESS)
  {
    SerialUSB.println("Failed setup.");
    while(1);
  }
  
  if (myIMU.MAG_WhoAMI_Read() != WhoAmIvalue){
    SerialUSB.println("Wrong WhoAmI value");
  }
  
  //task creation
  s1 = xTaskCreate(Read_task, NULL, 512, NULL, 2, NULL);
  s2 = xTaskCreate(Print_task, NULL, 512, NULL, 2, NULL);

  //create a queue with 100 fields of Magdata
  Read_Queue_Handle = xQueueCreate(100,sizeof(Magdata));
  
  //start task scheduler
  vTaskStartScheduler();
}

//send the magnotometer values to the queue
void Read_task(void *p){
  Magdata Sentvalue;
  
  while (1){
     Sentvalue.valueX = myIMU.readMagX();
     Sentvalue.valueY = myIMU.readMagY();
     Sentvalue.valueZ = myIMU.readMagZ();
    
    if (!xQueueSend(Read_Queue_Handle, &Sentvalue, 500)){
    SerialUSB.println("Failed to send to queue. Queue is full.");
    }
    vTaskDelay(500);
  }
  
}

//print the received magnotometer values from the queue
void Print_task(void *p){
  Magdata Receivedvalue;
  while (1) {
    if (!xQueueReceive(Read_Queue_Handle, &Receivedvalue, 5000)){
      SerialUSB.println("Failed to recieve color from queue. Queue is empty.");
    }else{
      SerialUSB.print("Recieved Mag X is: ");
      SerialUSB.println(Receivedvalue.valueX);
      SerialUSB.print("Recieved Mag Y is: ");
      SerialUSB.println(Receivedvalue.valueY);
      SerialUSB.print("Recieved Mag Z is: ");
      SerialUSB.println(Receivedvalue.valueZ);
    }
    vTaskDelay(500);
  }  
}

void loop() {
  // put your main code here, to run repeatedly:
}
