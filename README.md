# esp32-vario
A variometer using esp32 and GY-86 mpu6050 or CJMCU-117 9250 in DMP mode

IDE is Visual Studio using Vmicro add-in for Arduino development. This is my first use of Github so I may have got some things wrong using it if you use this project.

This is a port of Hari Nair's https://github.com/har-in-air/ESP8266_MPU9250_MS5611_VARIO to esp32 and using the DMP functionaility of the 9250 IMU on the GY-86 breakout board.

The majortity of this code was some great work by Jeff Rowberg for the mpu6050 library and the i2cdevlib interface, and Hari Nair for the Vario design. The calibration came from  Luis Ródenas.

I integrated Jeff's MPU6050 library to take advantage of the 6050 dmp processing which works fine on esp8266 but a bit tempremental on esp32 as there are some issues with timing using Jeff's library and the I2C bus. You need to be able to read in chunks using a buffer size of about 16. So for my chips and setup I played around with the buffer_length set in i2cdevlib.cpp and eventually chose 16. Anything larger tends to screw up the data being read. Also found that the readings are noisy when reading from the DMP at 200Hz as jeff found. So use an ODR of 100Hz and modified the code accordingly to take that into account.

I have a define to enable/disable DMP processing rather than calculate manually the rotations. Seems to be accurate and reduces the loop elapsed time by a factor of 3. 

Correct calibration I have found is key but on two gy-86, values of 1700 and 1200 for kfzVariance an kfazVariance seem to work best. Configuration in the config.h file of zero theshold 20 and climb threshold 20 also produce good results.

To build, you only need the esp32 core library installed as this sketch uses the preference, wire and wifi libraries


This code uses the gy-86 IMU and the reference system is different than the CJMCU-117. If you use the  cjmcu-117 module then just turn the board upside down and you get the same reults as the gy-86 which assumes the chips are up facing up on the pcb when used.

gy-86 interface
3.3V  3.3v
GND  GND
SCL  GPIO21
SDA  GPIO22
INT  GPIO12
audio GPIO13
calibration button GPIO15
