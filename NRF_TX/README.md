# IRQ Detection Example   
Use IRQ to detect transmission completion and reception completion.   

# About the nrf24l01 assertion

nrf24l01 has three assertions.   
- MAX_RT   
- RX_DR   
- TX_DS   

![Enhanced-Shokburst-1](https://user-images.githubusercontent.com/6020549/216748353-9028e1c9-0ec3-45fa-a13e-bfeabb42db86.jpg)

After the packet is transmitted by the PTX and received by the PRX the ACK packet with payload is transmitted from the PRX to the PTX.   
The RX_DR IRQ is asserted after the packet is received by the PRX, whereas on the PTX side the TX_DS IRQ is asserted when the ACK packet is received by the PTX.   
On the PRX side, the TX_DS IRQ for the ACK packet payload is asserted after a new packet from PTX is received.   

![Enhanced-Shokburst-2](https://user-images.githubusercontent.com/6020549/216748358-5264affc-6f7c-4f44-b8b8-23e0e30ddef1.jpg)

the ACK packet is lost and a retransmission is needed before the TX_DS IRQ is asserted, but the RX_DR IRQ is asserted immediately.   
MAX_RT IRQ is asserted if the auto retransmit counter(ARC_CNT) exceeds the programmed maximum limit(ARC).   
See the nrf24l01 datasheet for more details.   

In this project, RX_DX and MAX_RT are enabled, and TX_DS is disabled.   
RX_DX is asserted when reception is complete.   
Nothing is asserted when the transmission is complete.   
MAX_RT is asserted when transmission fails.   
Therefore, if there is no assertion after the transmission, the transmission succeeds, and if there is an assertion, the transmission fails.   
If you enable TX_DS, an assertion will occur regardless of whether the transmission was successful or unsuccessful.   
You will need to check the register again to distinguish between successful and unsuccessful transmission.   

However, if you want to know when transmission is complete sooner, you can enable TX_DS and determine the type of assertion within your application.   
To enable TX_DS, change the following in components/mirf/mirf.h:   
```
//#define mirf_CONFIG ((1<<MASK_TX_DS) | (1<<EN_CRC) | (0<<CRCO) )
#define mirf_CONFIG ((0<<MASK_TX_DS) | (1<<EN_CRC) | (0<<CRCO) )
```

To determine the type of assertion in your application, use the following function:   
This function reads the STATUS register.   
```
status = Nrf24_getStatus(&dev);
```

# Configuration   

![config-top](https://github.com/nopnop2002/esp-idf-mirf/assets/6020549/3aabd6f8-7477-4b71-b6c4-950d18402a87)

![config-app](https://github.com/nopnop2002/esp-idf-mirf/assets/6020549/01c6e755-05b1-43e7-90d5-c0eb063f5b82)

# Wiring
__For this project we need one more connection for IRQ detection.__   
|nRF24L01||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6||
|:-:|:-:|:-:|:-:|:-:|:-:|
|MISO|--|GPIO19|GPIO37|GPIO4|(*1)|
|MOSI|--|GPIO23|GPIO35|GPIO3|(*1)|
|SCK|--|GPIO18|GPIO36|GPIO2|(*1)|
|CE|--|GPIO16|GPIO34|GPIO1|(*1)|
|CSN|--|GPIO17|GPIO33|GPIO0|(*1)|
|IRQ|--|GPIO15|GPIO38|GPIO5|(*1)|
|GND|--|GND|GND|GND||
|VCC|--|3.3V|3.3V|3.3V||

(*1)You can change it to any pin using menuconfig.   
__IRQ needs to be able to use interrupts.__   
__Some GPIOs cannot use interrupts.__   

# Communicat with Arduino Environment   
Run this sketch.   
ArduinoCode\Peer-to-peer\StringTest   
