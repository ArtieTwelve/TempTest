TempTest

A simple program to read the temperature data from the HTS221 sensor on the STM32L475VG dev board. Most of the calibration code came directly from HTS221.c inside the board support package. <p>
  
   I used CubeMX to generate an IAR/STM32IDE project and then replaced the generated main.c file with this one.
   Please see the comments in main.c for details on setting up he USART and I2C pins in CubeMX.<p>
     
  I've included the BoardSupport.pdf. It details how to install the BSP into a project and where the files are located. You don't need the BSP to run this program but the code is nice to have as a reference when nothing seems to work. <p>
    
    I'm having trouble using HAL_Delay() when debugging in IAR. It seems to hang at that point. This doesn't seem to happen in STM32IDE. Not sure what's going on. I added a do nothing loop to kill some time or just uncomment the HAL_Delay line and try it for yourself. 
