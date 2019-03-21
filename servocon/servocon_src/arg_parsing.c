#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "dandy_platform.h"
#include "servocon_main.h"


#define TIME_LIMIT_INIT      0     // wait for connection of RM(unit:[sec])

ARGUMENT_OPTION     g_Arg;

////////////////////////////////////////////////////////////////////////////////
// ARGUMENT TYPE
//
enum ARGUMENT_TYPE
{
    ARG_HELP,
    ARG_PRINT_LEV,    
    ARG_TIMELIMIT,
    ARG_NOVGA,
    ARG_SINGLE,
    ARG_HARD_ESTOP,
    ARG_ECATSTAT_DISP,
    ARG_DISP_LINE,
    ARG_NO_TP_ESTOP,
    ARG_NO_SHOCKSENSOR,
    ARG_NO_CART_ESTOP
};

////////////////////////////////////////////////////////////////////////////////
// DispUsage()
//
void DispUsage(void)
{
    static const char szHelp[] = 
        "\n"
        "SERVOCON is control module of motor & IO device of dandy2015.\n"   
        "Copyright(c) 2013 DSME Co.Ltd. All Rights Reserved.\n\n"
        "USAGE  : servocon_qnx(_win) [-help] [-time <time>] [-print <opt>]\n"
        "         (identical '/' command instead of '-')"
        "\n\n"
        "-help  : Help message option of servocon process\n"
        "         This option is prior to all the options.\n"
        "         (identical '-?' command instead of '-help')\n"
        "\n"
        "-time  : Initialization wait time option\n"
        "         if <time> second is <= 0, SERVOCON waits infinitly.\n"
        "         Default option is Inf.(= 0)\n"
        "         (identical '-t' command instead of '-time')\n"
        "\n"
        "-print : Message option control disply level\n"
        "         (identical '-p' command instead of '-print')\n"
        "         <opt> is disply level. (default option : 'v')\n"
        "           v : VERBOSE, MESSAGE, ERROR, WARNING, ALERT, NOTIFY\n"
        "           m : MESSAGE, ERROR, WARNING, ALERT, NOTIFY\n"
        "           w : ERROR, WARNING, ALERT, NOTIFY\n"
        "           e : WARNING, ALERT, NOTIFY\n"
        "\n"
        "-novga  : Prevent VGA display\n"
        "         (identical '-nv' command instead of '-novga')\n"
        "\n"
        "-single : Process execution without sibling process\n"
        "         (identical '-s' command instead of '-single')\n"
        "\n"
        "-hestop : Hard E-Stop mode without TE action\n"
        "         (identical '-he' command instead of '-hestop')\n"
        "\n"
        "-estat  : EtherCAT Statistics Display\n"
        "         (identical '-e' command instead of '-estat')\n"
        "\n"
        "-line   : Specify VGA display First line\n"
        "         (identical '-l' command instead of '-line')\n"
        "\n"
        "-notp   : Ignore TP ESTOP\n"
        "         (identical '-ntp' command instead of '-notp')\n"
        "\n"
        "-nocart : Ignore Cart ESTOP\n"
        "         (identical '-nc' command instead of '-nocart')\n"
        "\n"
        "-noshock: Ignore Shock Sensor\n"
        "         (identical '-ns' command instead of '-noshock')\n"
        ;

    puts(szHelp);
}


////////////////////////////////////////////////////////////////////////////////
// IsNumber() : check the input value that is number
//
// return : 0 is number, else not number
//          
int IsNumber(const char* pszVal)
{
    int nLen;
    int i;

    nLen = strlen(pszVal);

    for (i = 0; i < nLen; i++)
    {
        if (pszVal[i] < 48 || pszVal[i] > 57)
        {
            return -1;
        }
    }

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
// ParseArgument()
//
BOOL ParseArgument(int nArgc, char* rgpszArgv[])
{
#pragma pack(push, 1)
    struct arg_info
    {
        const char*    pszArg;     // long-form argument
        const char*    pszArtAlt;  // shrot-form argument
        BOOL           bAssign;    // assign argument exist?
        int            nArgType;
    }; 
#pragma pack(pop)


    static const struct arg_info rgpszArgInfo[] = {
        // long-form  short-form  extra-option  identifier
        {"help",       "?",        FALSE,     ARG_HELP},        
        {"print",      "p",        TRUE,      ARG_PRINT_LEV},
        {"time",       "t",        TRUE,      ARG_TIMELIMIT},
        {"novga",      "nv",       FALSE,     ARG_NOVGA},
        {"single",     "s",        FALSE,     ARG_SINGLE},
        {"hestop",     "he",       FALSE,     ARG_HARD_ESTOP},
        {"estat",      "e",        FALSE,     ARG_ECATSTAT_DISP},
        {"line",       "l",        TRUE,      ARG_DISP_LINE},
        {"notp",       "ntp",      FALSE,     ARG_NO_TP_ESTOP},
        {"nocart",     "nc",       FALSE,     ARG_NO_CART_ESTOP},
        {"noshock",    "ns",       FALSE,     ARG_NO_SHOCKSENSOR}};

    int i, j;
    int nArgInfoCount;
    const char* pszArg;

    g_Arg.nVGADispLine = -1;

    ////////////////////////////////////////////////////////////////////////////
    // Argument Parsing...
    //
    
    // justified argument_info count
    nArgInfoCount = (int) (sizeof(rgpszArgInfo) / sizeof(rgpszArgInfo[0])); 

    // find '-' or '/'
    for (i = 0; i < nArgc; i++)
    {
        pszArg = rgpszArgv[i];  // i-th input argument

        if (rgpszArgv[i][0] == '-' || rgpszArgv[i][0] == '/')
        {
            pszArg++;  // skip '-' or '/'

            for (j = 0; j < nArgInfoCount; j++)
            {
                if(strcmp(pszArg, rgpszArgInfo[j].pszArg) == 0 || 
                   strcmp(pszArg, rgpszArgInfo[j].pszArtAlt) == 0)
                {
                    // process the argument
                    switch (rgpszArgInfo[j].nArgType)
                    {
                    case ARG_HELP: 
                        DispUsage();
                        return FALSE;
    
                    case ARG_TIMELIMIT:
                        if(nArgc >= i+2)  // check count of input arguments
                        {
                             g_nTime_Limit = atoi(rgpszArgv[i+1]);
                        }
                        break;
    
                    case ARG_PRINT_LEV:
                        if(nArgc >= i+2)
                        {
                            if(rgpszArgv[i+1][1] == 0)  // check count of characters
                            {
                                g_chPrintLev = rgpszArgv[i+1][0];
                            }
                        }
                        break; 
                    case ARG_NOVGA:
                        g_Arg.bNoVGA = TRUE;
                        break;
                    
                    case ARG_HARD_ESTOP:
                        g_Arg.bHardEstop = TRUE;
                        break;

                    case ARG_SINGLE:
                        g_Arg.bSingleExec = TRUE;
                        break;

                    case ARG_ECATSTAT_DISP:
                        g_Arg.bEcatStatDisplay = TRUE;
                        break;

                    case ARG_DISP_LINE:
                        if(nArgc >= i+2)  // check count of input arguments
                        {
                             g_Arg.nVGADispLine = atoi(rgpszArgv[i+1]);
                        }
                        break;

                    case ARG_NO_TP_ESTOP:
                        g_Arg.bNoTPEstop = TRUE;
                        break;
                    
                    case ARG_NO_CART_ESTOP:
                        g_Arg.bNoCartEstop = TRUE;
                        break;

                    case ARG_NO_SHOCKSENSOR:
                        g_Arg.bNoShockSensor = TRUE;
                        break;
                    }
                }

            } // end for
        } // end if
    } // end for
        
  
    return TRUE; 

}

