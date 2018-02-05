Needs Src/shared/config.h made with this format:


# config.h
```
#ifndef _CONFIG_H_
#define _CONFIG_H_

#define SERVER_IP_ADDRESS "XXX.XXX.XXX.XXX"
#define SERVER_PORT 80
#define MY_IP XXX, XXX, XXX, XXX
#define GATEWAY_IP XXX, XXX, XXX, XXX
#define SUBNET_MASK 255, XXX, XXX, XXX, XXX
#define WIFI_SSID "XXXXXXXXXXX"
#define WIFI_PASS "*******************"
#define GMT_OFFSET XX (-X or just X in hours)

#endif

#define ESP_ATMEGA_BAUD_RATE 1200
```