/*
 * CLI-commands.c
 *
 *  Created on: 02-02-2013
 *      Author: Grzybek
 */

#ifndef CLI_COMMANDS_C_
#define CLI_COMMANDS_C_

#include <stdio.h>
#include "CLI-commands.h"
#include "ff.h"
#include "RTC_Task.h"

/*
 * Globals
 */
static char messageTab[20];
static LcdDisplayLineCmd lcdMsg;

/* Structure that defines the "task-stats" command line command.  This generates
 a table that gives information on each task in the system. */
static const CLI_Command_Definition_t prvTaskStatsCommandDefinition =
{ (const int8_t * const ) "task-stats", /* The command string to type. */
        (const int8_t * const ) "task-stats:\r\n Displays a table showing the state of each FreeRTOS task\r\n\r\n",
        prvTaskStatsCommand, /* The function to run. */
        0 /* No parameters are expected. */
};

/* Structure that defines the "run-time-stats" command line command.   This
 generates a table that shows how much run time each task has */
static const CLI_Command_Definition_t prvRunTimeStatsCommandDefinition =
{ (const int8_t * const ) "run-time-stats", /* The command string to type. */
        (const int8_t * const ) "run-time-stats:\r\n Displays a table showing how much processing time each FreeRTOS task has used\r\n\r\n",
        prvRunTimeStatsCommand, /* The function to run. */
        0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t prvDummyCommandDefinition =
{ (const int8_t * const ) "dummy", /* The command string to type. */
        (const int8_t * const ) "dummy command:\r\n Displays a dummy string to ensure CLI interpreter is running properly.\r\n\r\n",
        prvDummyCommand, /* The function to run. */
        0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t prvLcdCommandDefinition =
{ (const int8_t * const ) "lcd", /* The command string to type. */
        (const int8_t * const ) "lcd [options]:\r\n"
        "Displays entered 'text' on LCD.\r\n"
        "\tOptions:\r\n"
        "\t-text line 'text': Display 'text' at specified line\r\n"
        "\t-circle x y rad : Draw a circle\r\n"
        "\t-text_color color : Set a text color [0-0xFFFF]\r\n"
        "\t-back_color color : Set a back screen color [0-0xFFFF]\r\n"
        "\t-clear color: Clears lcd in specified color [0-0xFFFF]\r\n",
        prvLcdCommand, /* The function to run. */
        -1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t prvMemAccessCommandDefinition =
{ (const int8_t * const ) "mem", /* The command string to type. */
        (const int8_t * const ) "mem -a [-w] [-r] \r\n"
        "Read or write to memory\r\n"
        "\t-a adress : Set a adress in hex\r\n"
        "\t-w data : Writes data to specified address in hex\r\n"
        "\t-r : Reads data from specified address in hex\r\n",
        prvMemAccessCommand, /* The function to run. */
        -1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t prvSDIOCommandDefinition =
{ (const int8_t * const ) "sdio", /* The command string to type. */
        (const int8_t * const ) "sdio [-mount] [-fread] [-fopen] [-fclose] \r\n"
        "SDIO interface\r\n"
        "\t-a adress : Set a adress in hex\r\n"
        "\t-w data : Writes data to specified address in hex\r\n"
        "\t-r : Reads data from specified address in hex\r\n",
        prvSDIOCommand, /* The function to run. */
        -1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t prvTimeCommandDefinition =
{ (const int8_t * const ) "time", /* The command string to type. */
        (const int8_t * const ) "time [-set] [-get]\r\n"
        "Gets or sets time."
        "\t-get : Get current system time\r\n"
        "\t-set thh tmm tss : Set system time to thh-hours:tmm-minutes:tss-seconds\r\n",
        prvTimeCommand, /* The function to run. */
        -1 /* No parameters are expected. */
};

void vRegisterCLICommands(void)
{
    FreeRTOS_CLIRegisterCommand(&prvDummyCommandDefinition);
    FreeRTOS_CLIRegisterCommand(&prvLcdCommandDefinition);
    FreeRTOS_CLIRegisterCommand(&prvMemAccessCommandDefinition);
    FreeRTOS_CLIRegisterCommand(&prvRunTimeStatsCommandDefinition);
    FreeRTOS_CLIRegisterCommand(&prvTaskStatsCommandDefinition);
    FreeRTOS_CLIRegisterCommand(&prvSDIOCommandDefinition);
    FreeRTOS_CLIRegisterCommand(&prvTimeCommandDefinition);
}

portBASE_TYPE prvDummyCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString)
{
    const int8_t * const dummyPrint =
            (int8_t*) "This is dummy command execution\r\n\r\n";
    (void) xWriteBufferLen;
    (void) pcCommandString;

    //configASSERT(pcWriteBuffer);

    strcpy((char*) pcWriteBuffer, (char*) dummyPrint);

    return pdFALSE;
}

portBASE_TYPE prvLcdCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString)
{
    static const char * ErrorPrint;
    portBASE_TYPE xReturn = pdFALSE;
    char * pcParameterString;
    char cmdChar[20];
    portBASE_TYPE xParameterStringLength = 0;
    portBASE_TYPE xParameterNumber = 0;
    u8 line = 0, column = 0;

    lcdMsg.message = messageTab;
    memset(lcdMsg.message, 0x00, 20);
    memset(cmdChar, 0x00, 20);
    memset(pcWriteBuffer, 0x00, configCOMMAND_INT_MAX_OUTPUT_SIZE);

    /*
     * Obtain first parameter = command
     */

    /* Obtain the parameter string. */
    xParameterNumber = 1;
    pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
            xParameterNumber, /* Return the next parameter. */
            &xParameterStringLength /* Store the parameter string length. */
    );

    /*
     * Get cmd parameter only
     */
    memcpy(cmdChar, pcParameterString, xParameterStringLength);

    if (strcmp(cmdChar, "-text") == 0)
    {
        char message[LCD_LINE_SIZE];
        memset(message, 0x00, LCD_LINE_SIZE);
        /* Obtain the parameter string. */
        xParameterNumber = 2;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        if (pcParameterString != NULL && xParameterStringLength <= 2)
        {
            line = atoi(pcParameterString);
        }
        else
        {
            sprintf(pcWriteBuffer, "Error! Invalid parameter line\r\n");
            return pdFALSE;
        }

        xParameterNumber = 3;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        if (pcParameterString != NULL && xParameterStringLength <= 2)
        {
            column = atoi(pcParameterString);
        }
        else
        {
            sprintf(pcWriteBuffer, "Error! Invalid parameter column\r\n");
            return pdFALSE;
        }

        char * first = strchr(pcCommandString, '\'');
        char * last = strrchr(pcCommandString, '\'');

        if (first == NULL || last == NULL )
        {
            sprintf(pcWriteBuffer, "Error! Wrong sentence marks!\r\n");
            return pdFALSE;
        }

        memcpy(message, first + 1, last - first - 1);

        if (pdPASS == LCD_Cmd(DISPLAY_LINE, message, line, column))
        {
            sprintf(pcWriteBuffer, "Command OK\r\n");
        }
        else
        {
            sprintf(pcWriteBuffer, "Command NOK\r\n");
        }
        xReturn = pdFALSE;
    }
    else if (strcmp(cmdChar, "-circle") == 0)
    {
        u8 x = 0;
        u16 y = 0, rad = 0;

        xParameterNumber = 2;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        if (pcParameterString != NULL && xParameterStringLength <= 3)
        {
            x = atoi(pcParameterString);
        }
        else
        {
            sprintf(pcWriteBuffer, "Error! Invalid parameter column\r\n");
            return pdFALSE;
        }

        xParameterNumber = 3;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        if (pcParameterString != NULL && xParameterStringLength <= 3)
        {
            y = atoi(pcParameterString);
        }
        else
        {
            sprintf(pcWriteBuffer, "Error! Invalid parameter column\r\n");
            return pdFALSE;
        }

        xParameterNumber = 4;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        if (pcParameterString != NULL && xParameterStringLength <= 3)
        {
            rad = atoi(pcParameterString);
        }
        else
        {
            sprintf(pcWriteBuffer, "Error! Invalid parameter column\r\n");
            return pdFALSE;
        }

        if (pdPASS == LCD_Cmd(DRAW_CIRCLE, x, y, rad))
        {
            sprintf(pcWriteBuffer, "Command OK\r\n");
        }
        else
        {
            sprintf(pcWriteBuffer, "Command NOK\r\n");
        }
        xReturn = pdFALSE;
    }
    else if (strcmp(cmdChar, "-text_color") == 0)
    {
        u16 color = 0;

        xParameterNumber = 2;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        if (pcParameterString != NULL )
        {
            color = (u16) strtoul(pcParameterString, NULL, 0);
        }
        else
        {
            sprintf(pcWriteBuffer, "Error! Invalid parameter color\r\n");
            return pdFALSE;
        }

        if (pdPASS == LCD_Cmd(SET_TEXT_COLOR, color))
        {
            sprintf(pcWriteBuffer, "Command OK\r\n");
        }
        else
        {
            sprintf(pcWriteBuffer, "Command NOK\r\n");
        }

        xReturn = pdFALSE;
    }
    else if (strcmp(cmdChar, "-back_color") == 0)
    {
        u16 color = 0;

        xParameterNumber = 2;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        if (pcParameterString != NULL )
        {
            color = (u16) strtoul(pcParameterString, NULL, 0);
        }
        else
        {
            sprintf(pcWriteBuffer, "Error! Invalid parameter color\r\n");
            return pdFALSE;
        }

        if (pdPASS == LCD_Cmd(SET_BACK_COLOR, color))
        {
            sprintf(pcWriteBuffer, "Command OK\r\n");
        }
        else
        {
            sprintf(pcWriteBuffer, "Command NOK\r\n");
        }

        xReturn = pdFALSE;
    }
    else if (strcmp(cmdChar, "-clear") == 0)
    {
        u16 color = 0;

        xParameterNumber = 2;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        if (pcParameterString != NULL )
        {
            color = (u16) strtoul(pcParameterString, NULL, 0);
        }
        else
        {
            sprintf(pcWriteBuffer, "Error! Invalid parameter color\r\n");
            return pdFALSE;
        }

        if (pdPASS == LCD_Cmd(CLEAR, color))
        {
            sprintf(pcWriteBuffer, "Command OK\r\n");
        }
        else
        {
            sprintf(pcWriteBuffer, "Command NOK\r\n");
        }

        xReturn = pdFALSE;
    }
    else
    {
        sprintf(pcWriteBuffer, "Error! Unsupported command!\r\n");
        xReturn = pdFALSE;
    }

    return xReturn;
}

portBASE_TYPE prvMemAccessCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString)
{
    portBASE_TYPE xReturn = pdFALSE;
    char * pcParameterString;
    char cmd[20];
    portBASE_TYPE xParameterStringLength = 0;
    static portBASE_TYPE xParameterNumber = 0;
    static u32 address = 0;
    u32 data = 0;

    memset(cmd, 0x00, 20);
    memset(pcWriteBuffer, 0x00, configCOMMAND_INT_MAX_OUTPUT_SIZE);

    if (0 == xParameterNumber)
    {
        /*
         * We obtain first command
         */
        xParameterNumber = 1;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        memcpy(cmd, pcParameterString, xParameterStringLength);

        if (strcmp(cmd, "-a") == 0)
        {
            /*
             * setup address
             */
            xParameterNumber++;
            pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(
                    pcCommandString, /* The command string itself. */
                    xParameterNumber, /* Return the next parameter. */
                    &xParameterStringLength /* Store the parameter string length. */
            );

            //memcpy(cmd,pcParameterString,xParameterStringLength);

            if (NULL != pcParameterString)
            {
                address = (u32) strtoul(pcParameterString, NULL, 0);
                xReturn = pdPASS;
            }
            else
            {
                sprintf(pcWriteBuffer, "Address is not valid!\r\n");
                xReturn = pdFALSE;
            }
        }
    }
    else
    {
        xParameterNumber++;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
                xParameterNumber, /* Return the next parameter. */
                &xParameterStringLength /* Store the parameter string length. */
        );

        if (NULL != pcParameterString)
        {
            memcpy(cmd, pcParameterString, xParameterStringLength);

            if (strcmp(cmd, "-w") == 0)
            {
                xParameterNumber++;
                pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(
                        pcCommandString, /* The command string itself. */
                        xParameterNumber, /* Return the next parameter. */
                        &xParameterStringLength /* Store the parameter string length. */
                );

                //memcpy(cmd, pcParameterString, xParameterStringLength);

                data = (u32) strtoul(pcParameterString, NULL, 0);

                *(vu32 *) (address) = (data);

                sprintf(pcWriteBuffer, "Write to address 0x%x successful",
                        address);

                xParameterNumber = 0;
                xReturn = pdFALSE;
            }
            else if (strcmp(cmd, "-r") == 0)
            {
                data = *(vu32*) (address);
                //data = 20;

                sprintf(pcWriteBuffer, "Read address 0x%x = 0x%x", address,
                        data);
                xParameterNumber = 0;
                xReturn = pdFALSE;
            }
        }
        else
        {
            xParameterNumber = 0;
            xReturn = pdFALSE;
        }
    }

    return xReturn;
}

portBASE_TYPE prvTaskStatsCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString)
{
    const int8_t * const pcTaskTableHeader =
            (int8_t *) "Task          State  Priority  Stack  #\r\n************************************************\r\n";

    /* Remove compile time warnings about unused parameters, and check the
     write buffer is not NULL.  NOTE - for simplicity, this example assumes the
     write buffer length is adequate, so does not check for buffer overflows. */
    (void) pcCommandString;
    (void) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Generate a table of task stats. */
    strcpy((char *) pcWriteBuffer, (char *) pcTaskTableHeader);
    vTaskList(pcWriteBuffer + strlen((char *) pcTaskTableHeader));

    /* There is no more data to return after this single string, so return
     pdFALSE. */
    return pdFALSE;
}

portBASE_TYPE prvRunTimeStatsCommand(int8_t *pcWriteBuffer,
        size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    const int8_t * const pcStatsTableHeader =
            (int8_t *) "Task            Abs Time      % Time\r\n****************************************\r\n";

    /* Remove compile time warnings about unused parameters, and check the
     write buffer is not NULL.  NOTE - for simplicity, this example assumes the
     write buffer length is adequate, so does not check for buffer overflows. */
    (void) pcCommandString;
    (void) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Generate a table of task stats. */
    strcpy((char *) pcWriteBuffer, (char *) pcStatsTableHeader);
    vTaskGetRunTimeStats(pcWriteBuffer + strlen((char *) pcStatsTableHeader));

    /* There is no more data to return after this single string, so return
     pdFALSE. */
    return pdFALSE;
}


portBASE_TYPE prvSDIOCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString)
{
    portBASE_TYPE xReturn = pdFALSE;
    char * pcParameterString;
    char cmd[20];
    static char filename[20];
    char buffer[1024];
    portBASE_TYPE xParameterStringLength = 0;
    static portBASE_TYPE xParameterNumber = 0;
    static FATFS fs;
    static FIL file;
    FRESULT res;
    UINT byteRead = 0;

    memset(cmd, 0x00, 20);
    memset(pcWriteBuffer, 0x00, configCOMMAND_INT_MAX_OUTPUT_SIZE);
    memset(buffer, 0x00, 1024);
    memset(filename, 0x00, 20);

    //if (0 == xParameterNumber)
    //{
    /*
     * We obtain first command
     */
    xParameterNumber = 1;
    pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
    xParameterNumber, /* Return the next parameter. */
    &xParameterStringLength /* Store the parameter string length. */
    );

    memcpy(cmd, pcParameterString, xParameterStringLength);

    if (strcmp(cmd, "-fread") == 0)
    {

        // Get file name
        xParameterNumber++;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
        xParameterNumber, /* Return the next parameter. */
        &xParameterStringLength /* Store the parameter string length. */
        );

        memcpy(filename, pcParameterString, xParameterStringLength);
        res = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);

        if (FR_OK != res)
        {
            f_close(&file);
            sprintf(pcWriteBuffer, "File open failed with status %d\n", res);
        }
        else
        {
            f_read(&file, buffer, 1024, &byteRead);
            memcpy(pcWriteBuffer, buffer, strlen(buffer));
            f_close(&file);
        }
        xReturn = pdFALSE;

    }
    else if (strcmp(cmd, "-mount") == 0)
    {
        res = f_mount(1, &fs);
        if (FR_OK == res)
        {
            sprintf(pcWriteBuffer, "File system mount success!\n");
        }
        else
        {
            sprintf(pcWriteBuffer, "File system mount failed!\n");
        }
        xReturn = pdFALSE;
    }
    else if (strcmp(cmd, "-fopen") == 0)
    {
        xReturn = pdFALSE;
    }
    else if (strcmp(cmd, "-fclose") == 0)
    {
        // Get file name
        xParameterNumber++;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
        xParameterNumber, /* Return the next parameter. */
        &xParameterStringLength /* Store the parameter string length. */
        );

        memcpy(filename, pcParameterString, xParameterStringLength);

        f_close(&file);
        sprintf(pcWriteBuffer, "File &s closed\n", filename);
    }
    //}

    return xReturn;
}


portBASE_TYPE prvTimeCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString)
{
    portBASE_TYPE xReturn = pdFALSE;
    char * pcParameterString;
    char cmd[20];
    portBASE_TYPE xParameterStringLength = 0;
    static portBASE_TYPE xParameterNumber = 0;
    uint32_t thh=0,tmm=0,tss=0;

    memset(cmd, 0x00, 20);
    memset(pcWriteBuffer, 0x00, configCOMMAND_INT_MAX_OUTPUT_SIZE);

    xParameterNumber = 1;
    pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
    xParameterNumber, /* Return the next parameter. */
    &xParameterStringLength /* Store the parameter string length. */
    );

    memcpy(cmd,pcParameterString,xParameterStringLength);

    if (strcmp(cmd, "-set") == 0)
    {

        xParameterNumber++;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
        xParameterNumber, /* Return the next parameter. */
        &xParameterStringLength /* Store the parameter string length. */
        );

        thh = atoi(pcParameterString);

        xParameterNumber++;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
        xParameterNumber, /* Return the next parameter. */
        &xParameterStringLength /* Store the parameter string length. */
        );

        tmm = atoi(pcParameterString);

        xParameterNumber++;
        pcParameterString = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
        xParameterNumber, /* Return the next parameter. */
        &xParameterStringLength /* Store the parameter string length. */
        );

        tss = atoi(pcParameterString);

        RTC_SetTime(thh,tmm,tss);

        sprintf(pcWriteBuffer,"Time set to %02d:%02d:%02d\n\r",thh,tmm,tss);

        xParameterNumber = 0;
        xReturn = pdFALSE;
    }
    else if (strcmp(cmd, "-get") == 0)
    {
        RTC_GetTime(&thh,&tmm,&tss);

        sprintf(pcWriteBuffer,"Time %02d:%02d:%02d\n\r",thh,tmm,tss);

        xParameterNumber = 0;
        xReturn = pdFALSE;
    }

    return xReturn;
}

#endif /* CLI_COMMANDS_C_ */
