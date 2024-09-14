# SmartEnviMonitoring
A weather reporting system that periodly read temperature and humidity and send the values to web server [SmartEnviMonitoring_Web](https://github.com/Xellosi/SmartEnviMonitoring_Web).

# Componments and Connections
- stm32F407 discovery board
- DHT11
  > Data line: GPIO PA1
- lcd1602 I2C
  > I2C1
- ESP32-WROOM-32D
  > UART2
  >
# Setup
Change the following variables in main.c to the url and ip address of the server you set up.
- http_device_url
- http_weather_url
- server_ip
- mqtt_port
