#include "headers.h"

/// LED
#define NR_OF_LEDS 1
#define NR_OF_ALL_BITS 24 * NR_OF_LEDS
#define DIN 23

rmt_data_t led_data[NR_OF_ALL_BITS];
int led_index = 0;

rmt_obj_t* rmt_send = NULL;

void ledInit()
{
    if ((rmt_send = rmtInit(DIN, true, RMT_MEM_64)) == NULL)
    {
        Serial.println("[ledInit] ERROR cannot init led");
    }
}

void ledDeInit()
{
    rmtDeinit(rmt_send);
}

void ledRed()
{
    int colorRed[] = {0x00, 0xff, 0x00}; // GRB

    float realTick = rmtSetTick(rmt_send, 100);

    // Init data with only one led ON
    int led, col, bit;
    int i = 0;
    for (col = 0; col < 3; col++)
    {
        for (bit = 0; bit < 8; bit++)
        {
            if ((colorRed[col] & (1 << (7 - bit))) && (led == led_index))
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 8;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 4;
            }
            else
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 4;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 8;
            }
            i++;
        }
    }
    // make the led travel in the pannel
    if ((++led_index) >= NR_OF_LEDS)
    {
        led_index = 0;
    }

    // Send the data
    rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
}

void ledGreen()
{
    int colorGreen[] = {0xff, 0x00, 0x00}; // GRB

    float realTick = rmtSetTick(rmt_send, 100);

    // Init data with only one led ON
    int led, col, bit;
    int i = 0;
    for (col = 0; col < 3; col++)
    {
        for (bit = 0; bit < 8; bit++)
        {
            if ((colorGreen[col] & (1 << (7 - bit))) && (led == led_index))
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 8;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 4;
            }
            else
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 4;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 8;
            }
            i++;
        }
    }
    // make the led travel in the pannel
    if ((++led_index) >= NR_OF_LEDS)
    {
        led_index = 0;
    }

    // Send the data
    rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
}

void ledBlue()
{
    int colorBlue[] = {0x00, 0x00, 0xff}; // GRB

    float realTick = rmtSetTick(rmt_send, 100);

    // Init data with only one led ON
    int led, col, bit;
    int i = 0;
    for (col = 0; col < 3; col++)
    {
        for (bit = 0; bit < 8; bit++)
        {
            if ((colorBlue[col] & (1 << (7 - bit))) && (led == led_index))
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 8;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 4;
            }
            else
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 4;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 8;
            }
            i++;
        }
    }
    // make the led travel in the pannel
    if ((++led_index) >= NR_OF_LEDS)
    {
        led_index = 0;
    }

    // Send the data
    rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
}

void ledMagenta()
{
    int colorMagenta[] = {0x00, 0xff, 0xff}; // GRB

    float realTick = rmtSetTick(rmt_send, 100);

    // Init data with only one led ON
    int led, col, bit;
    int i = 0;
    for (col = 0; col < 3; col++)
    {
        for (bit = 0; bit < 8; bit++)
        {
            if ((colorMagenta[col] & (1 << (7 - bit))) && (led == led_index))
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 8;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 4;
            }
            else
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 4;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 8;
            }
            i++;
        }
    }
    // make the led travel in the pannel
    if ((++led_index) >= NR_OF_LEDS)
    {
        led_index = 0;
    }

    // Send the data
    rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
}

void ledBlack()
{
    int colorGreen[] = {0x00, 0x00, 0x00}; // GRB

    float realTick = rmtSetTick(rmt_send, 100);

    // Init data with only one led ON
    int led, col, bit;
    int i = 0;
    for (col = 0; col < 3; col++)
    {
        for (bit = 0; bit < 8; bit++)
        {
            if ((colorGreen[col] & (1 << (7 - bit))) && (led == led_index))
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 8;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 4;
            }
            else
            {
                led_data[i].level0 = 1;
                led_data[i].duration0 = 4;
                led_data[i].level1 = 0;
                led_data[i].duration1 = 8;
            }
            i++;
        }
    }
    // make the led travel in the pannel
    if ((++led_index) >= NR_OF_LEDS)
    {
        led_index = 0;
    }

    // Send the data
    rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
}