/*
 * console.c
 *
 *  Created on: 02-02-2013
 *      Author: Grzybek
 */


#include "console.h"
#include <stdio.h>

#define ESC 0x1b

static Peripheral_Descriptor_t xConsole = NULL;

static const char * welcomeMsg = "DragonBoard command interpreter\r\n";
static const char * termChar = ">";
static const char * newLine = "\r\n";
static const char * cursorRight = "\033[1C";
static const char * cursorLeft = "\033[1D";
static const char * eraseChar = "\033[1X";

Peripheral_Descriptor_t * GetConsoleDescriptor(void)
{
    return &xConsole;
}

void putChar(char c)
{
    if(xConsole != NULL)
    {
        FreeRTOS_write(xConsole,&c,1);
    }
}

portBASE_TYPE vStartConsoleTask(void)
{
    portBASE_TYPE result = pdTRUE;

    xTaskCreate( ConsoleTask, ( signed char * ) "Console", CONSOLE_STACK_SIZE, NULL, CONSOLE_TASK_PRIORITY, ( xTaskHandle * ) NULL );
    return result;
}

portTASK_FUNCTION(ConsoleTask,parameters)
{
    char cRxedChar;
    char * pcOutputString;
    uint8_t index = 0, cmdIndex = 0;
    portBASE_TYPE xReturned = 0;
    char inputString[CONSOLE_CMD_MAX_INPUT_SIZE], lastInputString[CONSOLE_CMD_MAX_INPUT_SIZE];
    char terminalCmd[20];
    portBASE_TYPE cmdInput = pdFALSE;

    /* Obtain the address of the output buffer.  Note there is no mutual
     exclusion on this buffer as it is assumed only one command console
     interface will be used at any one time. */
    pcOutputString = FreeRTOS_CLIGetOutputBuffer();

    memset(inputString,0x00,CONSOLE_CMD_MAX_INPUT_SIZE);
    memset(lastInputString,0x00,CONSOLE_CMD_MAX_INPUT_SIZE);
    memset(terminalCmd, 0x00, 20);

    xConsole = FreeRTOS_open((int8_t*)CONSOLE_USART,NULL);

    configASSERT(xConsole);

    FreeRTOS_ioctl( xConsole, ioctlUSE_CHARACTER_QUEUE_RX, ( void * ) 40 );
    //FreeRTOS_ioctl( xConsoleUART, ioctlUSE_CHARACTER_QUEUE_TX, ( void * ) 50 );
    //FreeRTOS_ioctl( xConsoleUART, ioctlSET_RX_TIMEOUT, ( void * ) 100 );
    //FreeRTOS_ioctl( xConsoleUART, ioctlSET_TX_TIMEOUT, ( void * ) 1000 );


    FreeRTOS_write(xConsole,welcomeMsg,strlen(welcomeMsg));
    FreeRTOS_write(xConsole,termChar,strlen(termChar));

    while(1)
    {
        /* Only interested in reading one character at a time. */
        FreeRTOS_read( xConsole, &cRxedChar, 1);

        if(cRxedChar == ESC) // test if input is terminal command
        {
            cmdInput = pdTRUE;
            cmdIndex = 0;
            memset(terminalCmd,0x00,20);
            continue;
        }
        else if(cmdInput)
        {
            terminalCmd[cmdIndex++] = cRxedChar;

            if (cmdIndex > 1)
            {
                if(strcmp(terminalCmd,"[A") == 0)
                {
                    /*
                     * Write last input to console
                     */
                    FreeRTOS_write(xConsole,lastInputString,strlen(lastInputString));
                    memset(inputString,0x00,CONSOLE_CMD_MAX_INPUT_SIZE);
                    memcpy(inputString,lastInputString,strlen(lastInputString));
                    index = strlen(lastInputString);
                }
                else if(strcmp(terminalCmd,"[B") == 0)
                {
                    /*
                     * Write last input to console
                     */
                    FreeRTOS_write(xConsole,lastInputString,strlen(lastInputString));
                    memset(inputString,0x00,CONSOLE_CMD_MAX_INPUT_SIZE);
                    memcpy(inputString,lastInputString,strlen(lastInputString));
                    index = strlen(lastInputString);
                }
                else if(strcmp(terminalCmd,"[C") == 0)
                {
                    if ('\0' != inputString[index])
                    {
                        FreeRTOS_write(xConsole, cursorRight,
                                strlen(cursorRight));
                        index++;
                    }
                }
                else if(strcmp(terminalCmd,"[D") == 0)
                {
                    if (0 < index)
                    {
                        FreeRTOS_write(xConsole, cursorLeft,
                                strlen(cursorLeft));
                        index--;
                    }

                }
                else
                {

                }
                cmdInput = pdFALSE;
                continue;
            }

            continue;
        }


        /* Echo the character back. */
        FreeRTOS_write( xConsole, &cRxedChar, 1);

        if(cRxedChar == '\r' || cRxedChar == '\n')
        {

            if(index == 0)
            {
                FreeRTOS_write(xConsole,newLine,strlen(newLine));
                FreeRTOS_write(xConsole,termChar,strlen(termChar));
                continue;
            }
            FreeRTOS_write(xConsole,newLine,strlen(newLine));

            FreeRTOS_write(xConsole,newLine,strlen(newLine));

            /* Pass the received command to the command interpreter.  The
            command interpreter is called repeatedly until it returns
            pdFALSE as it might generate more than one string. */
            do
            {
                /* Get the string to write to the UART from the command
                 interpreter. */
                xReturned = FreeRTOS_CLIProcessCommand(inputString,
                        pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE);

                /* Write the generated string to the UART. */
                FreeRTOS_write( xConsole, pcOutputString,
                        strlen(pcOutputString));

            } while( xReturned != pdFALSE );

            strcpy(lastInputString,inputString);
            memset(inputString,0x00,CONSOLE_CMD_MAX_INPUT_SIZE);
            index = 0;


            FreeRTOS_write(xConsole,newLine,strlen(newLine));
            FreeRTOS_write(xConsole,termChar,strlen(termChar));
        }
        else
        {
            // we receive new character
            if(cRxedChar == '\r')
            {
                // do nothing
            }
            else if(cRxedChar == '\b')
            {
                /* Backspace was pressed.  Erase the last character in the
                string - if any. */
                if( index > 0 )
                {
                    index--;
                    inputString[index] = '\0';

                    // erase character on console
                    FreeRTOS_write(xConsole,eraseChar,strlen(eraseChar));

                }
                else
                {
                    FreeRTOS_write(xConsole,termChar,strlen(termChar));
                }
            }
            else
            {
                /* A character was entered.  Add it to the string
                entered so far.  When a \n is entered the complete
                string will be passed to the command interpreter. */
                if( ( cRxedChar >= ' ' ) && ( cRxedChar <= '~' ) )
                {
                    if( index < CONSOLE_CMD_MAX_INPUT_SIZE )
                    {
                        inputString[index] = cRxedChar;
                        index++;
                    }
                }
            }
        }
    }
}




