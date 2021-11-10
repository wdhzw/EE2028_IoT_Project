  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "stdio.h"
#include "stm32l4xx_it.h"
#include "string.h"
#define TEMP_THRESHOLD 37.4
#define GYRO_THRESHOLD 100
#define HUMID_THRESHOLD 70
#define PRESSURE_THRESHOLD 1050
//extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
static void MX_GPIO_Init(void);
static void UART1_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
UART_HandleTypeDef huart1;
int buttonpress=0,fall_warning=0;
int main(void)
{
	int mode=4;//0 for healthy mode and 1 for intensive care mode
	int seconds_count = 0,enter1=0,enter2=0;
	int current_time=0,lying_time=0,sample_time=0,press_time,blink_time=0,press=0;//press: how many times you press the button
	int temp_warning=0,gyro_warning=0,magnet_warning=0,humid_warning=0,pressure_warning=0,firstdetect=0;

	char line_print[64];
	char buffer[64];

	HAL_Init();
	UART1_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	BSP_HSENSOR_Init();
	BSP_PSENSOR_Init();

	MX_GPIO_Init();
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    SENSOR_IO_Write(0xD4,0x58,0x80);//enable free fall interrupt
    SENSOR_IO_Write(0xD4,0x5E,0x90);//configure in
    int blink=0;
	char password[]="0000",wrong[]="Password incorrect.(Enter EXIT to quit)\n\r",exitmode1[]="EXIT",exitmode2[]="exit",exitmode3[]="Exit";
	char exitmode[]="Exit Successfully\n\r";
	while (1)
	{
		int len=0;
		char data;
		char fall[]="Falling is detected\n\r";
		char breath[]="Check patient's breath!\n\r";
		char healthymode[]="Entering Healthy Mode\r\n";
		char intensivemode[]="Entering Intensive Care Mode\r\n";
		float accel_data[3];
		float gyro_data[3];
		float maget_data[3];
		float gyro_magnitude;
		float oringinal_orientation[3];
		char fever[]="Fever is detected\n\r";
		char pain[]="Pain is detected\n\r";
		char orientation[]="Check patient's abnormal orientation!\r\n";
		float temp_data,humidity_data,pressure_data;
		int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
		int16_t maget_data_i16[3]={0};
		if(HAL_GetTick()-sample_time>1000)
		{
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
		BSP_GYRO_GetXYZ(gyro_data);
		BSP_MAGNETO_GetXYZ(maget_data_i16);

		//gyro data
		gyro_data[0]=(float)gyro_data[0]/1000.0f;
		gyro_data[1]=(float)gyro_data[1]/1000.0f;
		gyro_data[2]=(float)gyro_data[2]/1000.0f;
		gyro_magnitude=(float)sqrt(gyro_data[0]*gyro_data[0]+gyro_data[1]*gyro_data[1]+gyro_data[2]*gyro_data[2]);
		//accel data
		accel_data[0] = (float)accel_data_i16[0] / 100.0f;
		accel_data[1] = (float)accel_data_i16[1] / 100.0f;
		accel_data[2] = (float)accel_data_i16[2] / 100.0f;
        //maget data
	    maget_data[0] = (float)maget_data_i16[0]/1000.0f;
	    maget_data[1] = (float)maget_data_i16[1]/1000.0f;
	    maget_data[2] = (float)maget_data_i16[2]/1000.0f;
	    if(firstdetect==0)
	    {
	    	oringinal_orientation[0]=maget_data[0];
	    	oringinal_orientation[1]=maget_data[1];
	    	oringinal_orientation[2]=maget_data[2];
	    	firstdetect=1;
	    }
		temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor
		humidity_data=BSP_HSENSOR_ReadHumidity();   // humid
		pressure_data=BSP_PSENSOR_ReadPressure();   // pressure

		sample_time=HAL_GetTick();
		}
        if(mode==0)
            {
        	if(enter1==0)
        	{
        	HAL_UART_Transmit(&huart1, (uint8_t*)healthymode, strlen(healthymode),0xFFFF);
        	enter1=1;
        	}
		    //mode toggle
        	if(buttonpress==1&&press==0)
				{
					press_time=HAL_GetTick();
					press=1;
					buttonpress=0;
				}
				else if(buttonpress==1&&press==1)
				{
					if(HAL_GetTick()-press_time<1000)
					{
						mode=1;
						press=0;
						buttonpress=0;
						enter2=0;
    				    blink=0;
    				    temp_warning=0;
    				    fall_warning=0;
    				    gyro_warning=0;
    				    magnet_warning=0;
    				    humid_warning=0;
    				    pressure_warning=0;
					}
					else {
						press_time=HAL_GetTick();
					    buttonpress=0;}
				}
        	//temperature&&free_fall
        		if(HAL_GetTick()-current_time>=10000)
        		{
        			 if(temp_data>TEMP_THRESHOLD)
        			 {
        			    temp_warning=1;

        			 }
        			 if(temp_warning==1)
        			 {
        				 HAL_UART_Transmit(&huart1, (uint8_t*)fever, strlen(fever),0xFFFF);
        				 blink=1;
        			 }

        			 if(fall_warning==1)
        			 {
        				 HAL_UART_Transmit(&huart1, (uint8_t*)fall, strlen(fall),0xFFFF);
        				 blink=1;
        			 }
        			 current_time=HAL_GetTick();
        		}

            }

        	if(mode==1)
        	{
        		if(enter2==0)
        		{
        		HAL_UART_Transmit(&huart1, (uint8_t*)intensivemode, strlen(intensivemode),0xFFFF);

        		enter2=1;
        		}
        		//mode_toggle
    			if(buttonpress==1)
    			   {
    					mode=0;
    				    buttonpress=0;
    				    enter1=0;
    				    blink=0;
    				    temp_warning=0;
    				    fall_warning=0;
    				    gyro_warning=0;
    				    magnet_warning=0;
    				    humid_warning=0;
    				    pressure_warning=0;
    				    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
    			   }
    			//warning message
        		if(HAL_GetTick()-current_time>=10000)
        		{
        		  /*temp sensor*/
        		   if(temp_data>TEMP_THRESHOLD)
        			   temp_warning=1;
        		   if(temp_warning==1)
        		   {
        			   HAL_UART_Transmit(&huart1, (uint8_t*)fever, strlen(fever),0xFFFF);
        			   blink=1;
        		   }
        		   /*accel sensor*/

        		   if(fall_warning==1)
        		   {
        			   HAL_UART_Transmit(&huart1, (uint8_t*)fall, strlen(fall),0xFFFF);
        			   blink=1;
        		   }
        		   /*gyro sensor*/
      			  if(gyro_magnitude>GYRO_THRESHOLD)
      			   {
      				 gyro_warning=1;
      			   }
      		  	if(gyro_warning==1)
      			  {
      			   HAL_UART_Transmit(&huart1, (uint8_t*)pain, strlen(pain),0xFFFF);
      			   blink=1;
      			   }
       		   /*humidity and pressure sensor*/
       		   if(humidity_data<HUMID_THRESHOLD)
       		   {
       			   humid_warning=1;
       		   }
       		   if(pressure_data>PRESSURE_THRESHOLD)
       		   {
       			   pressure_warning=1;
       		   }
       		   if(humid_warning==1||pressure_warning==1)
       		   {
       			  HAL_UART_Transmit(&huart1,(uint8_t*)breath,strlen(breath),0xFFFF);
       			  blink=1;
       		   }

     		   sprintf(line_print,"%.3d_TEMP_%.2f_ACC_%.2f_%.2f_%.2f \r\n",seconds_count,temp_data,accel_data[0]/9.8, accel_data[1]/9.8, accel_data[2]/9.8);
     		   HAL_UART_Transmit(&huart1,(uint8_t*)line_print , strlen(line_print),0xFFFF);
     		   seconds_count++;
     		   memset(line_print,0,strlen(line_print));
       		   sprintf(line_print,"%.3d GYRO %.1f MAGNETO %.1f %.1f %.1f \r\n",seconds_count,gyro_magnitude,maget_data[0], maget_data[1], maget_data[2]);
       		   HAL_UART_Transmit(&huart1,(uint8_t*)line_print,strlen(line_print),0xFFFF);
       		   seconds_count++;
       		   memset(line_print,0,strlen(line_print));
        	   sprintf(line_print,"%.3d HUMIDITY %.1f and BARO %.2f \n\r",seconds_count,humidity_data,pressure_data);
        	   HAL_UART_Transmit(&huart1,(uint8_t*)line_print,strlen(line_print),0xFFFF);
        	   seconds_count++;
        	   memset(line_print,0,strlen(line_print));

       		     current_time=HAL_GetTick();
           }
        		if(HAL_GetTick()-lying_time>10000)
        		{
        			if((maget_data[0]-oringinal_orientation[0])>0.3||(oringinal_orientation[0]-maget_data[0])>0.3)
        			{
        				magnet_warning=1;
        			}
        			if((maget_data[1]-oringinal_orientation[1])>0.3||(oringinal_orientation[1]-maget_data[1])>0.3)
        			{
        				magnet_warning=1;
        			}
        			if((maget_data[2]-oringinal_orientation[2])>0.3||(oringinal_orientation[2]-maget_data[2])>0.3)
        			{
        				magnet_warning=1;
        			}
        			if(magnet_warning==1)
        			{
        				HAL_UART_Transmit(&huart1, (uint8_t*)orientation, strlen(orientation),0xFFFF);
        				blink=1;
        			}
        			lying_time=HAL_GetTick();
        		}
       }
       if(mode==3)
       {
    	     char enterpassword[]="Please enter 4-digit password\n\r";
    	     HAL_UART_Transmit(&huart1, (uint8_t*)enterpassword, strlen(enterpassword),0xFFFF);
    		 memset(buffer,0,strlen(buffer));
    		 len=0;
    	      do
    	      {
    	    	  HAL_UART_Receive(&huart1,(uint8_t*)&data,1,0xFFFF);
    	    	  if(data!='\r'&& data!='\n')
    	    	  {
    	    		  len++;
    	    		  buffer[len-1]=data;
    	    	  }
    	      }
    	       while((len<=64)&&(data!='\r'));
    	       buffer[len]='\0';

  	      if(strcmp(exitmode1,buffer)==0||strcmp(exitmode2,buffer)==0||strcmp(exitmode3,buffer)==0)
    	   {

    		   mode=4;
    		   HAL_UART_Transmit(&huart1, (uint8_t*)exitmode, strlen(exitmode),0xFFFF);
    	   }
    	       else
    	       { if(strcmp(password,buffer)==0)
    		    	   {mode=0;}
    		       else
    		    	   HAL_UART_Transmit(&huart1,(uint8_t*)wrong,strlen(wrong),0xFFFF);

    	       }
  	   }



       if(mode==4)
       {
    	 char mode_start[]="This is a system to enhance monitoring of COVID patients\n\rPlease enter START to continue\n\r";
    	 char start[]="System Starting....\n\r";
    	 HAL_UART_Transmit(&huart1,(uint8_t*)mode_start,strlen(mode_start),0xFFFF);
  		 memset(buffer,0,strlen(buffer));
  		 len=0;
  	     do
  	      {
  	    	  HAL_UART_Receive(&huart1,(uint8_t*)&data,1,0xFFFF);
  	    	  if(data!='\r'&& data!='\n')
  	    	  {
  	    		  len++;
  	    		  buffer[len-1]=data;
  	    	  }
  	      }
  	       while((len<=64)&&(data!='\r'));
  	       buffer[len]='\0';
  	     if(strcmp("START",buffer)==0)
  	     {
  	    	HAL_UART_Transmit(&huart1,(uint8_t*)start,strlen(start),0xFFFF);
  	    	mode=3;
  	     }
       }
    		if(blink==1)
    		{
    			if(HAL_GetTick()-blink_time>500)
    			{
    				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
    				blink_time=HAL_GetTick();
    			}
    		}

}

}

static void MX_GPIO_Init(void)
	{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  /*Configure GPIO pin Output Level */
	  //pushbutton
	  HAL_GPIO_WritePin(GPIOC,BUTTON_EXTI13_Pin, GPIO_PIN_RESET);
	  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	  //freefall
	  HAL_GPIO_WritePin(GPIOD,LSM6DSL_INT1_EXTI11_Pin, GPIO_PIN_RESET);
	  GPIO_InitStruct.Pin = LSM6DSL_INT1_EXTI11_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	  //led
	  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
	  GPIO_InitStruct.Pin = LED2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}

static void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{

	    	 if(GPIO_Pin==GPIO_PIN_13)//mode_toggle
	    		buttonpress=1;
			 if(GPIO_Pin==GPIO_PIN_11)//Free fall
				 fall_warning=1;
	}
