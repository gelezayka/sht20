## **sht20**

sht20 rs485 (XY-MD01) - replacement firmware

### **Compile**

Use Keil C51 for compile and Nu-Link for flash


### **Modbus Registers**

Input Register:
0x0001 - Temperature
0x0002 - Humidity

Keep Register:
0x0101 - Device Address
0x0102 - Baud Rate 9600, 14400, 19200 and etc
0x0103 - Temperature correction(/10) -10.0~10.0
0x0104 - Humidity correction(/10) -10.0~10.0

