/*
main.c
2024-08-09

Testing the lgpio library for commanding ws2812b LEDs
Sets LED1 to Green 
gcc -Wall -o main main.c -lrgpio

./main
*/

#include <stdio.h>
#include <stdlib.h>

#include <../include/lg/lgpio.h>
#include <../include/lg/rgpio.h>

#define LFLAGS 0
#define GPIOPIN 21
#define GPIOCHIP 2 // For dev/gpiochip2 on RPI5
/*
Timing in microseconds for ws2812b LED pulses
From WS2812B.pdf
*/
#define T0H .45 
#define T0L .85 
#define T1H .8 
#define T1L .4 
#define TRESET 50


int main(int argc, char *argv[])
{
   int h;
   int i;
   double start, end;

   sbc = rgpiod_start(NULL, NULL);

   if (sbc < 0)
   {
      printf("connection failed\n");
      exit(-1);
   }

   h = gpiochip_open(sbc, GPIOCHIP);

   if (h >= 0)
   {
      if (lgGpioClaimOutput(h, LFLAGS, GPIOPIN, 0) == LG_OKAY)
      {
        //Send a 0b10
        start = lguTime();
        lgTxPulse(h, GPIOPIN, T1H, T1L, 0, 1);
        lgTxPulse(h, GPIOPIN, T0H, T0L, 0, 1);
        end = lguTime();
        printf("Took %.1f seconds to send a 1 then 0\n", LOOPS, end-start);

        /*Send a green pulse
         LEDs are in GRB format 
         So: 0b111111110000000000000000
        */

       for (int i = 0; i < LOOPS; i++)
       {
        start = lguTime();
        lguSleep(TRESET * .0000001); //Reset Command
        lgTxPulse(h, GPIOPIN, T1H, T1L, 0, 8); //Send 8 ones
        lgTxPulse(h, GPIOPIN, T0H, T0L, 0, 16); //Send 16 zeros
        end = lguTime();
        printf("Took %.1f seconds to send a Reset cmd followed by a green pulse to LED1 then 0\n", LOOPS, end-start);
      }

      }
      gpiochip_close(sbc, h);
   }

   rgpiod_stop(sbc);
}
