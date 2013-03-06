/*
 * lcdcmd.h
 *
 *  Created on: 03-02-2013
 *      Author: Grzybek
 */

#ifndef LCDCMD_H_
#define LCDCMD_H_


typedef enum eLcdCmd
{
    DRAW_CIRCLE=0,
    DRAW_RECT,
    DISPLAY_LINE,
    SET_TEXT_COLOR,
    SET_BACK_COLOR,
    CLEAR_LINE,
    CLEAR,
    DRAW_LINE,
    DISPLAY_CHAR,
    DISPLAY_LINE_COLUMN
}LcdCmd;

//typedef eLcdCmd LcdCmd;

#endif /* LCDCMD_H_ */
