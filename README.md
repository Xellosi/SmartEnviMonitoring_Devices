# SmartEnviMonitoring
A weather reporting system that periodically reads temperature and humidity data and sends the values to the web server [SmartEnviMonitoring_Web](https://github.com/Xellosi/SmartEnviMonitoring_Web).

![image](https://github.com/Xellosi/SmartEnviMonitoring_Devices/blob/main/Resources/device.jpg)

# Componments and Connections
- stm32F407 discovery board
- DHT11
  > Data line: GPIO PA1
- lcd1602 I2C
  > I2C1
- ESP32-WROOM-32D (ESP AT commands)
  > UART2
  
# Setup
Change the following variables in main.c to match the server you set up.
- server_ip
- mqtt_port

HTTP URLs are composed from `server_ip` using these macros in `Core/Src/main.c`:
- `HTTP_PORT_STR`
- `HTTP_DEVICE_PATH`
- `HTTP_WEATHER_PATH`