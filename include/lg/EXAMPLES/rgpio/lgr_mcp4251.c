/*
lgr_mcp4251.c
2021-01-20
Public Domain
*/

#include <stdlib.h>

#include <rgpio.h>

#include "lg_mcp4251.h"

typedef struct mcp4251_s
{
   int sbc;     // sbc connection
   int device;  // SPI device
   int channel; // SPI channel
   int speed;   // SPI bps
   int flags;   // SPI flags
   int spih;    // SPI handle
   int value;   // wiper value
   int MAX_WIPER_VALUE;
   int WIPERS;
   callbk_t enable;
} mcp4251_t, *mcp4251_p;


mcp4251_p MCP4251_open(int sbc, int device, int channel, int speed, int flags)
{
   mcp4251_p s;

   s = calloc(1, sizeof(mcp4251_t));

   if (s == NULL) return NULL;

   s->sbc = sbc;         // sbc connection
   s->device = device;   // SPI device
   s->channel = channel; // SPI channel
   s->speed = speed;     // SPI speed
   s->flags = flags;     // SPI flags

   s->spih = spi_open(sbc, device, channel, speed, flags);

   if (s->spih < 0)
   {
      free(s);
      s = NULL;
   }

   return s;
}

mcp4251_p MCP4251_close(mcp4251_p s)
{
   if (s != NULL)
   {
      spi_close(s->sbc, s->spih);
      free(s);
      s = NULL;
   }
   return s;
}

int MCP4251_set_wiper(mcp4251_p s, int value)
{
   char buf[16];

   if (s == NULL) return -1;

   if ((value < 0) || (value > 128)) return -2;

   s->value = value;

   if (s->enable != NULL) s->enable(1);

   buf[0] = 0;
   buf[1] = value;

   spi_write(s->sbc, s->spih, buf, 2);

   if (s->enable != NULL) s->enable(0);

   return 0;
}

int MCP4251_get_wiper(mcp4251_p s)
{
   if (s == NULL) return -1;

   return s->value;
}

int MCP4251_increment_wiper(mcp4251_p s)
{
   if (s == NULL) return -1;

   if (s->value < 128) MCP4251_set_wiper(s, s->value + 1);

   return 0;
}

int MCP4251_decrement_wiper(mcp4251_p s)
{
   if (s == NULL) return -1;

   if (s->value > 0) MCP4251_set_wiper(s, s->value - 1);

   return 0;
}

int MCP4251_set_enable(mcp4251_p s, callbk_t enable)
{
   s->enable = enable;

   return 0;
}

#ifdef EXAMPLE

/*
gcc -D EXAMPLE -o mcp4251 lgr_mcp4251.c -lrgpio
./mcp4251
*/

#include <stdio.h>

#include <lgpio.h>
#include <rgpio.h>

#include "lg_mcp4251.h"

int main(int argc, char *argv[])
{
   int sbc=-1;
   mcp4251_p dac=NULL;
   int potpos;

   sbc = rgpiod_start(NULL, NULL);

   if (sbc < 0) return -1;

   dac = MCP4251_open(sbc, 0, 0, 50000, 0);

   if (dac == NULL) return -2;

   potpos = 0;

   while (1)
   {
      MCP4251_set_wiper(dac, potpos);

      lgu_sleep(0.2);

      potpos++;

      if (potpos > 128) potpos = 0;
   }

   return 0;
}

#endif

