TempTest

A simple program to read the temperature data from the HTS221 sensor on the STM32L475VG dev board. Most of the calibration code came directly from HTS221.c inside the board support package. <p>
  
   I used CubeMX to generate an IAR/STM32IDE project and then replaced the generated main.c file with this one.<p>
     
Setup: I used STM32 CubeMX to generate the IAR project files.
Use these pins for USART1 and I2C2, the default USART pins CubeMX sets up did not work for me.
          
  * 	   PB6 - USART1_RX (moved from PA10)
  * 	   PB7 - USART1_TX (moved from PA9)
  
The default I2C2 pins work for I2C2
  * 	   PB10 - I2C2_SCL
  * 	   PB11 - I2C2_SDA
  

<p>
     
  I've included the BoardSupport.pdf. It details how to install the BSP into a project and where the files are located. You don't need the BSP to run this program but the code is nice to have as a reference when nothing seems to work. <p>
    
I'm having trouble using HAL_Delay() when debugging in IAR. It seems to hang at that point. This doesn't seem to happen in STM32IDE. Not sure what's going on. I added a do nothing loop to kill some time. You'll have to play with the counter value to get a decent interval. Or, just uncomment the HAL_Delay line and see if it will work for you. 
