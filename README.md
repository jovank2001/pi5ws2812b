## Control WS2812B LED Matrix using your Raspberry Pi 5
This Repository contains a Driver for the new Raspberry Pi 5 RP1 Peripheral chip that can be used to control your WS2812b LEDs.

Huge kudos to [Praktronics](https://github.com/praktronics) for figuring out the SPI RP1 register addresses


### To Run Example which turns on 1st LED Green:
    1. Ensure hardware connections are made as shown in Connections
    2. pi5ws2812b $ gcc rp1Spi.c rp1SpiUtil.c pi5ws2812b.c -o run
    3. pi5ws2812b $ sudo ./run

Result:

<img src="example.jpg" alt="drawing" width="500" height = "400"/>

### Connections
![Alt text](connections.jpg)

