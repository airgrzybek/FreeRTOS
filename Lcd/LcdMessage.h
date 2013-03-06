/*
 * LcdMessage.h
 *
 *  Created on: 29-01-2013
 *      Author: Grzybek
 */

#ifndef LCDMESSAGE_H_
#define LCDMESSAGE_H_

#include <stdio.h>
#include <stdlib.h>
#include "lcdcmd.h"

#define LCD_MAX_LINE

typedef struct
{
    LcdCmd CMD;
    void * params;
}LcdCmdMsg;

typedef struct
{
    u8 line;
    char * message;
}LcdDisplayLineCmd;

typedef struct
{
    u8 line;
    u8 column;
    u8 size;
    char * message;
}LcdDisplayLineColumnCmd;



typedef struct
{
    u8 x;
    u16 y;
    u16 rad;
}LcdDrawCircleCmd;

typedef struct
{
    u16 color;
}LcdSetColorCmd;


#endif /* LCDMESSAGE_H_ */
