/*
main.c
2024-08-09

Testing the lgpio library for commanding ws2812b LEDs
gcc -Wall -o main main.c -llgpio

./main
*/

#include <stdio.h>
#include <stdlib.h>

#include <../include/lg/lgpio.h>

#define GPIOPIN 21
#define LOOPS 120
#define LFLAGS 0
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


int main(int argc, char *argv[]);

