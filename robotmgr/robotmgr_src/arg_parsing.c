/////////////////////////////////////////////////////////////////////////////
//
//  arg_parsing.c: argument parsing
//                                            2013.04.11  Ryu SinWook


///////////////////////////////////////

#include "robotmgr_main.h"
#include "ipc_jobshm.h"

///////////////////////////////////////

#if defined(_MSC_VER)
#pragma warning(push, 4)
#endif

///////////////////////////////////////
//Global_variables

int g_nWaitInitSec;
int g_nWaitConnSec;


/////////////////////////////////////////////////////////////////////////////
//
//  ARGUMENT type
//

enum _ARGUMENT_TYPE
{
    ARG_HELP,
    ARG_VERBOSE,
    ARG_QUIET,
    ARG_VGA,
    ARG_CONFIG,
    ARG_KEYIN,
    ARG_MINIT,
    ARG_WAITINIT,
    ARG_WAITCONN,
    ARG_DEBUG,
    ARG_CLEAN,
    ARG_CONF_IGNORE,
    ARG_ROBOT,
    ARG_TRAJTIME,
    ARG_IOTIME,
    ARG_LEGACY,
    ARG_ASSEMBLE,
    ARG_DISASSEM,
    ARG_OUTFILE,
    ARG_MAPFILE,
    ARG_WELDER,
    ARG_WELDMAP,
    ARG_SENSOR,
    ARG_NOAUTOINIT,
    ARG_SHUTDOWN,
    ARG_NODEAD,
    ARG_SINGLE,

    NUM_OF_ARG,
};


/////////////////////////////////////////////////////////////////////////////
//
//  Function: DispUsage()
//

void DispUsage(void)
{
    static const char szProgName[] =
        "\nDandy-II Robot Manager program"
        "";
    static const char szCopyRight[] =
        "Copyright(c) 2013 DSME Co.Ltd. All Rights Reserved.\n"
        "";
    static const char szHelpDescript[] =
        "Function: System Management, Comm, Job File Handling, Stats..\n\n"
        "Usage: robotmgr [-?] [-v] [-q] [-cf<=file>] [-k] [-m]"
        " [-wi<=time>] [-wc<=time>] [-c] [-cfi] [-r<=type[,axis]>]"
        " [-tj<=time>] [-io<=time>] [-sd]\n"
        "\t('/' can be used instead of '-' for option prefix)\n"
        "Options:short\tlong\textra-opt  descript\n"
        "\t-?,\t-help\tFALSE\t : display this help messages\n"
        "\t-v,\t-verb\tFALSE\t : display all reference messages\n"
        "\t-q,\t-quiet\tFALSE\t : suppress display messages\n"
        "\t-nv,\t-novga\tFALSE\t : suppress VGA display messages\n"
        "\t-cf,\t-conf\tTRUE\t : load specified config file\n"
        "\t\t\t\t  (-cf=file is identical)\n"
        "\t-k,\t-key\tFALSE\t : keyboard input enable\n"
        "\t-mi,\t-minit\tFALSE\t : manual initialization enable\n"
        "\t-wi,\t-winit\tTRUE\t : set wait time for initialization\n"
        "\t\t\t\t  (-wi=time(sec) is identical)\n"
        "\t-wc,\t-wconn\tTRUE\t : set wait time for channel connection\n"
        "\t\t\t\t  (-wc=time(sec) is identical)\n"
        "\t-d\t-debug\tFALSE\t : display message for debug\n"
        "\t-c\t-clean\tFALSE\t : clean dirty files\n"
        "\t-cfi\t-ignor\tFALSE\t : ignore config (all default)\n"
        "\t-r,\t-robot\tTRUE\t : set robot type\n"
        "\t\t\t\t  (-r=type[,axis] is identical)\n"
        "\t\t\t\t    <type> robot type\n"
        "\t\t\t\t      -VOID    : non-motion robot\n"
        "\t\t\t\t      -10KG    : DSME 10kg welding robot\n"
        "\t\t\t\t      -DR6     : Doosan DR6 robot\n"
        "\t\t\t\t      -DANDY-II: DSME DANDY-II robot\n"
        "\t\t\t\t    <axis> start axis of the robot\n"
        "\t\t\t\t    <tool> welder, grinder, blower etc.\n"
        "\t-tj\t-traj\tTRUE\t : trajectory scan time (ms)\n"
        "\t\t\t\t  (-tj=time(ms) is identical)\n"
        "\t-l\t-legacy\tFALSE\t :\n\t\tcompatible with Dandy-1996 (legacy syntax compatible)\n"
        "\t-a\t-assem\tTRUE\t :\n\t\tassemble 'files' and the write as a .job file\n"
        "\t-d\t-disassem TRUE\t : disassemble 'files' each\n"
        "\t-o\t-out\tTRUE\t :\n\t\tspecified output file name if -assem option is specified\n"
        "\t  (default out file name is first source file name with .job extension)\n"
        "\t-m\t-map\tTRUE\t :\n\t\tspecified output file name if -assem option is specified\n"
        "\t  (default map file name is first source file name with .map extension)\n"
        "\t-nai\t-nainit\tFALSE\t : no automatically init retry process\n"
        "\t-sd\t-shutd\tFALSE\t : shutdown system after app termination\n"
        "\t-nd\t-nodead\tFALSE\t : in case of no deadman switch\n"
        "\t-s\t-single\tFALSE\t : in case of single run(no channel connection)\n"
        "";

    printf("%s (%s)", szProgName, PROJECT_NAME);
    printf(" [Ver: %s, Build: %s]\n", SYS_RM_VERSION, SYS_RM_BUILD);
    printf("%s", szCopyRight);
    printf("%s", szHelpDescript);
    printf("\t[Default Value] wi = %d(sec), wc = %d(sec)\n",
                            INIT_WAIT_TIME_LIMIT_SEC, WAITTIME_CONNECTION_SEC);
    printf("\tTotal %d program options can be used\n", NUM_OF_ARG);
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: ParseArgument()
//

#define ARG_MAX_KEY_LEN     48

BOOL ParseArgument(int nArgc, char* rgpszArgv[], ARGUMENT_OPTION* pArg)
{
    static const struct
    {
        const char*     pszArg;     // long-form argument
        const char*     pszArgAlt;  // short-form argument
        BOOL            bAssign;    // assign argument exist?
        int             nArgType;
    } rgpszArgInfo[] = {
       // long-form     short-form  extra-option   identifier      id-extra
        { "help",       "?",        FALSE,         ARG_HELP},
        { "help",       "h",        FALSE,         ARG_HELP},
        { "verb",       "v",        FALSE,         ARG_VERBOSE},
        { "quiet",      "q",        FALSE,         ARG_QUIET},
        { "novga",      "nv",       FALSE,         ARG_VGA},
        { "conf",       "cf",       TRUE,          ARG_CONFIG},
        { "key",        "k",        FALSE,         ARG_KEYIN},
        { "minit",      "mi",       FALSE,         ARG_MINIT},
        { "winit",      "wi",       TRUE,          ARG_WAITINIT},
        { "wconn",      "wc",       TRUE,          ARG_WAITCONN},
        { "debug",      "d",        FALSE,         ARG_DEBUG},
        { "clean",      "c",        FALSE,         ARG_CLEAN},
        { "ignor",      "cfi",      FALSE,         ARG_CONF_IGNORE},
        { "robot",      "r",        TRUE,          ARG_ROBOT},
        { "traj",       "tj",       TRUE,          ARG_TRAJTIME},
        { "io",         "io",       TRUE,          ARG_IOTIME},
        { "legacy",     "l",        FALSE,         ARG_LEGACY},
        { "assem",      "a",        FALSE,         ARG_ASSEMBLE},
        { "disassem",   "d",        FALSE,         ARG_DISASSEM},
        { "out",        "o",        FALSE,         ARG_OUTFILE},
        { "map",        "m",        FALSE,         ARG_MAPFILE},
        { "welder",     "w",        TRUE,          ARG_WELDER},
        { "weldmap",    "wm",       TRUE,          ARG_WELDMAP},
        { "sensor",     "sen",      TRUE,          ARG_SENSOR},
        { "nainit",     "nai",      FALSE,         ARG_NOAUTOINIT},
        { "shutd",      "sd",       FALSE,         ARG_SHUTDOWN},
        { "nodead",     "nd",       FALSE,         ARG_NODEAD},
        { "single",     "s",        FALSE,         ARG_SINGLE},
        { NULL, NULL, FALSE, -1}};

    static char rgszRobotTypeName[MAX_ROBOT_COUNT][ROBOT_NAME_LEN];
        
    char szBuffer[256];
    const char* pszArg;
    const char* pszVal;
    const char* pszNext;
    char* pszEndStr;

    static int nRobot;
    static int nAxis;

    static char szKey[ARG_MAX_KEY_LEN];
    static int nLen;
    static int nTimeArgToInt;
    static int iArg, i;
    static BOOL fExtraOption;

    BOOL bNextOutFileName;
    BOOL bNextMapFileName;
    
    ///////////////////////////////////

    fExtraOption = FALSE;
    nTimeArgToInt = 0;
    g_nWaitInitSec = INIT_WAIT_TIME_LIMIT_SEC;
    g_nWaitConnSec = WAITTIME_CONNECTION_SEC;

    bNextOutFileName = bNextMapFileName = FALSE;


    ///////////////////////////////////
    //
    // argument parsing...
    //
    
    pArg->nFileCount = 0;

    for (iArg = 1; iArg < nArgc; iArg++)
    {
        pszArg = rgpszArgv[iArg];

        ///////////////////////////////
        //
        // all arguments start wih '-' or '/'
        //

        if (pszArg[0] != '-' && pszArg[0] != '/')
        {
            //printf("invalid argument : '%s'\n", pszArg);
            //return FALSE;

            // out file
            if (bNextOutFileName)
            {
                pArg->pszOutFileName = pszArg;
                bNextOutFileName = FALSE;
            }
            else if (bNextMapFileName)
            {
                pArg->pszMapFileName = pszArg;
                bNextMapFileName = FALSE;
            }
            else
            {
                // must be file name
                if (pArg->nFileCount >= COMPILE_FILE_COUNT)
                {
                    printf("too many file specified : %d files < %d\n",
                           pArg->nFileCount,
                           COMPILE_FILE_COUNT);

                    return FALSE;
                }

                pArg->rgpszFiles[pArg->nFileCount++] = pszArg;
            }

            continue;
        }
        else
        {
            bNextOutFileName = bNextMapFileName = FALSE;
        }

        pszArg++;   // skip first '-' or '/'

        ///////////////////////////////
        //
        //  find '=' for 'key=value' expression
        //

        pszVal = strchr(pszArg, '=');

        if (pszVal == NULL)
        {
            nLen = strlen(rgpszArgv[iArg]);
        }
        else
        {
            nLen = pszVal - pszArg;
            pszVal++;   // skip '='
            fExtraOption = TRUE;
        }

        ///////////////////////////////
        //
        // copy the key to 'szKey'
        //

        if (nLen >= ARG_MAX_KEY_LEN)
            nLen = ARG_MAX_KEY_LEN - 1;
        
        if (fExtraOption == FALSE)
        {
            CRT_strcpy(szKey, nLen, pszArg);
        }
        else if(fExtraOption == TRUE)
        {
            CRT_strcpy(szKey, nLen+10, pszArg);
        }

        szKey[nLen] = 0;

        ///////////////////////////////
        //
        // find argument
        //

        for (i = 0; rgpszArgInfo[i].pszArg != NULL; i++)
        {
            if (STR_IGCASE_CMP(szKey, rgpszArgInfo[i].pszArg) == 0 ||
                (rgpszArgInfo[i].pszArgAlt != NULL &&
                 STR_IGCASE_CMP(szKey, rgpszArgInfo[i].pszArgAlt) == 0))
            {
                // found
                break;
            }
        }

        // found?
        if (rgpszArgInfo[i].pszArg == NULL)
        {
            // not found the argument
            printf("unknown argument : '%s'\n", pszArg);
            return FALSE;
        }

        // assign mark check...
        if (rgpszArgInfo[i].bAssign == TRUE && pszVal == NULL)
        {
            printf("value for the argument missing : '%s'\n", pszArg);
            return FALSE;
        }
        else if (rgpszArgInfo[i].bAssign == TRUE && pszVal != NULL)
        {
            // convert string to integer
            printf("extra value for the argument : '%s(%s)'\n", pszArg, pszVal);
            nTimeArgToInt = atoi(pszVal);
        }
        
        // protect negative value input
        if(rgpszArgInfo[i].bAssign == TRUE && pszVal != NULL &&
           nTimeArgToInt < 0)
        {
            printf("argument error (value must be positive): '%s'\n", pszArg);
            return FALSE;
        }

        ///////////////////////////////
        //
        // process the argument
        //

        switch (rgpszArgInfo[i].nArgType)
        {
        case ARG_HELP :
            DispUsage();
            pArg->bHelp = TRUE;
            return FALSE;
       
        case ARG_VERBOSE :
            pArg->bVerbose = TRUE;
            break;

        case ARG_QUIET :
            pArg->bQuiet = TRUE;
            break;

        case ARG_VGA :
            pArg->bVGA = TRUE;
            break;

        case ARG_CONFIG :
            DANDY_ASSERT(pszVal != NULL);
            pArg->bManualConfig = TRUE;
            pArg->pszConfigName = pszVal;
            break;

        case ARG_KEYIN :
            pArg->fKeyIn = TRUE;
            printf("-->> RM Key Input Enabled! <<--\n");
            break;
        
        case ARG_MINIT :
            pArg->bManualInit = TRUE;
            printf("-->> Manual Init Enabled! <<--\n");
            break;

        case ARG_WAITINIT :
            DANDY_ASSERT(pszVal != NULL);
            pArg->nWaitInitTime = nTimeArgToInt;
            printf("-->> Wait Init Time sets to %d sec! <<--\n",
                                                          pArg->nWaitInitTime);
            g_nWaitInitSec = pArg->nWaitInitTime;
            break;

        case ARG_WAITCONN :
            DANDY_ASSERT(pszVal != NULL);
            pArg->nWaitConnTime = nTimeArgToInt;
            printf("-->> Wait Connection Time sets to %d sec! <<--\n",
                                                          pArg->nWaitConnTime);
            g_nWaitConnSec = pArg->nWaitConnTime;
            break;

        case ARG_DEBUG :
            printf("-->> Display Debug Message! <<--\n");
            pArg->fDebug =  TRUE;
            break;

        case ARG_CLEAN :
            pArg->fClean =  TRUE;
            break;

        case ARG_CONF_IGNORE :
            pArg->fCfIgnore = TRUE;
            break;

        case ARG_ROBOT :
            DANDY_ASSERT(pszVal != NULL);
            pArg->fRobotConfig = TRUE;
            // robot=<type>[,start_axis]
            // robot=dr6    : start_axis = 0
            // robot=dr6, 2 : start_axis = 2
            nRobot = pArg->nRobotCount++;
            DANDY_ASSERT(nRobot >= 0);

            if (pszVal == NULL)
            {
                printf("Invalid argument : %s\n", rgpszArgv[iArg]);
                return FALSE;
            }

            if (nRobot >= MAX_ROBOT_COUNT)
            {
                printf("Too many robot specified\n");
                return FALSE;
            }

            pszNext = pszVal;

            // robot type
            pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, 256);

            if (pszNext == NULL || szBuffer[0] == 0)
            {
                printf("Illegal Robot Type = '%s'\n", pszVal);
                return FALSE;
            }

            CRT_strcpy(rgszRobotTypeName[nRobot], ROBOT_NAME_LEN, szBuffer);
            rgszRobotTypeName[nRobot][ROBOT_NAME_LEN-1] = 0;

            pArg->rgpszRobotTypeName[nRobot] = rgszRobotTypeName[nRobot];

            // start axis
            pszNext = PARAM_ParseArrayConfig(pszNext, szBuffer, 256);

            if (pszNext != NULL && szBuffer[0] != 0)
            {
                nAxis = strtol(szBuffer, &pszEndStr, 0);

                if (*pszEndStr != 0 || (nAxis < 0 || nAxis >= ROB_AXIS_COUNT))
                {
                    printf("Invalid start axis specified : %s\n", szBuffer);
                    return FALSE;
                }

                pArg->rgnStartAxis[nRobot] = nAxis;
            }
            break;

        case ARG_TRAJTIME :
            DANDY_ASSERT(pszVal != NULL);
            pArg->nTrajTime = atoi(pszVal);
            printf("-->> Trajectory Scan Time sets to %d mili-sec! <<--\n",
                                                          pArg->nTrajTime);
            pArg->fTrajTime = TRUE;
            break;

        case ARG_IOTIME :
            DANDY_ASSERT(pszVal != NULL);
            pArg->nIoTime = atoi(pszVal);
            printf("-->> I/O Scan Time sets to %d mili-sec! <<--\n",
                                                          pArg->nIoTime);
            pArg->fIoTime = TRUE;
            break;

        case ARG_LEGACY :
            pArg->bLegacySyntax = TRUE;
            g_nAssembleOpt = JOBASM_AF_DANDY1996;
            break;

        case ARG_ASSEMBLE :
        case ARG_DISASSEM :
            if (pArg->nCompileType != COMPILE_TYPE_UNKNOWN)
            {
                printf("* conflict option specified. "
                       "please check -assem and -disassem option.\n"
                       "\n");

                DispUsage();
                return FALSE;
            }

            if (rgpszArgInfo[i].nArgType == ARG_ASSEMBLE)
            {
                pArg->bAssemble = TRUE;
                pArg->nCompileType = COMPILE_TYPE_ASSEMBLE;
                printf("-->> assembling start! <<--\n");
            }
            else
            {
                pArg->bDisassem = TRUE;
                pArg->nCompileType = COMPILE_TYPE_DISASSEM;
                printf("-->> disassembling start! <<--\n");
            }
            break;

        case ARG_OUTFILE :
            if (pArg->pszOutFileName != NULL)
            {
                printf("output file duplicated : '%s'\n",
                       pArg->pszOutFileName);

                return FALSE;
            }

            pArg->bOutFile = TRUE;
            bNextOutFileName = TRUE;
            break;

        case ARG_MAPFILE :
            if (pArg->pszMapFileName != NULL)
            {
                printf("map file duplicated : '%s'\n",
                       pArg->pszMapFileName);

                return FALSE;
            }

            pArg->bMapFile = TRUE;
            bNextMapFileName = TRUE;
            break;

        case ARG_WELDER :
            DANDY_ASSERT(pszVal != NULL);
            pArg->fWelder = TRUE;
            break;

        case ARG_WELDMAP :
            DANDY_ASSERT(pszVal != NULL);
            pArg->fWeldMap = TRUE;
            break;

        case ARG_SENSOR :
            DANDY_ASSERT(pszVal != NULL);
            pArg->fSensorConfig = TRUE;
            break;

        case ARG_NOAUTOINIT :
            pArg->fNoAutoInit = TRUE;
            printf("-->> auto init process Disabled <<--\n");
            break;

        case ARG_SHUTDOWN :
            pArg->fShutdown = TRUE;
            break;

        case ARG_NODEAD:
            pArg->bNoDeadMan = TRUE;
            break;

        case ARG_SINGLE:
            pArg->bSingleRun = TRUE;
            printf("-->> Single Run Mode Enabled! <<--\n");
            break;

        default :
            // unreachable here!
            DANDY_ASSERT(FALSE);
            pArg->bHelp = FALSE;
            pArg->bVerbose = FALSE;
            pArg->bQuiet = FALSE;
            pArg->fKeyIn = FALSE;
            pArg->bManualInit = FALSE;
            pArg->nWaitInitTime = INIT_WAIT_TIME_LIMIT_SEC;
            pArg->nWaitConnTime = WAITTIME_CONNECTION_SEC;
        }
    }

    return TRUE;
}
