/////////////////////////////////////////////////////////////////////////////
//
//  conf_load.c: Parameter Config Information Load & Management
//                                            2013.06.20  Ryu SinWook
#define _USE_MATH_DEFINES
#include <math.h>

///////////////////////////////////////

#include "robotmgr_main.h"

///////////////////////////////////////

int g_fConfigLoadCheck      = 0;
int g_nConfigLoadOption     = 0;

///////////////////////////////////////

#ifndef UNREFERENCED_PARAMETER
#define UNREFERENCED_PARAMETER(__var)       (__var = __var)
#endif


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_ParseSection()
//

static int _loc_SYSC_ParseSection(const char* pszSectionName, int* pnSectionIndex)
{
    static const struct
    {
        int             nSectionType;
        const char*     pszSectionName;
        int             nNameLen;
    } rgConfigSections[] = {
        {CONFIG_SECTION_GLOBAL, "global", 6},
        {CONFIG_SECTION_ROBOT,  "robot",  5},
        {CONFIG_SECTION_AXIS,   "axis",   4},
        {CONFIG_SECTION_MOTOR,  "motor",  5},
        {CONFIG_SECTION_WELDER, "welder", 6},
        {CONFIG_SECTION_SENSOR, "sensor", 6},
        {CONFIG_SECTION_STATIS, "statis", 6},
        {CONFIG_SECTION_PARAM,  "param",  5},
    };
    
    const char* pszIndex;
    char* pszEndStr;

    int nSectionType;
    int nSectionIndex;
    int i, nCount;

    nCount = sizeof(rgConfigSections) / sizeof(rgConfigSections[0]);

    nSectionType = -1;
    pszIndex = NULL;

    for (i = 0; i < nCount; i++)
    {
        if (strnicmp(pszSectionName,
                     rgConfigSections[i].pszSectionName,
                     rgConfigSections[i].nNameLen) == 0)
        {
            nSectionType = rgConfigSections[i].nSectionType;
            pszIndex = pszSectionName + rgConfigSections[i].nNameLen;
            break;
        }
    }

    if (i >= nCount)
    {
        VERBOSE_ERROR("Unknown section name is specified : '[%s]'\n",
                      pszSectionName);

        return RESULT_ERROR;
    }

    // find configuration index

    if (nSectionType == CONFIG_SECTION_ROBOT ||
        nSectionType == CONFIG_SECTION_AXIS ||
        nSectionType == CONFIG_SECTION_MOTOR ||
        nSectionType == CONFIG_SECTION_WELDER ||
        nSectionType == CONFIG_SECTION_PARAM ||
        nSectionType == CONFIG_SECTION_STATIS)
    {
        // robot/axis/welder/param sections have index number
        // i.e., robot1, robot2, axis0, axis1, welder0, welder1, ...

        // valid number?
        if (!isdigit(*pszIndex))
        {
            VERBOSE_ERROR("invalid section number specified : '%s'\n",
                          pszSectionName);

            return RESULT_ERROR;
        }

        nSectionIndex = (int) strtol(pszIndex, &pszEndStr, 10);

        if (*pszEndStr != 0)
        {
            VERBOSE_ERROR("invalid section number specified : '%s'\n",
                          pszSectionName);

            return RESULT_ERROR;
        }
    }
    else
    {
        nSectionIndex = 0;
    }

    switch (nSectionType)
    {
        // check configuration range
    case CONFIG_SECTION_ROBOT :
        // robot1 ~ robot4 are valid range
        if (nSectionIndex <= 0 || nSectionIndex > MAX_ROBOT_COUNT)
        {
            VERBOSE_ERROR("invalid robot number specified : '%d' (%d ~ %d is valid)\n",
                          nSectionIndex, 1, MAX_ROBOT_COUNT);

            return RESULT_ERROR;
        }

        nSectionIndex--;    // user uses from 1, internally uses from zero
        break;

    case CONFIG_SECTION_AXIS :
        // axis0 ~ axis31 are valid range
        if (nSectionIndex < 0 || nSectionIndex >= MAX_AXIS_COUNT)
        {
            VERBOSE_ERROR("invalid axis number specified : '%d' (%d ~ %d is valid)\n",
                          nSectionIndex, 0, MAX_AXIS_COUNT-1);

            return RESULT_ERROR;
        }
        break;

    case CONFIG_SECTION_MOTOR :
        // motor0 ~ motor31 are valid range
        if (nSectionIndex < 0 || nSectionIndex >= MAX_MOTOR_COUNT)
        {
            VERBOSE_ERROR("invalid motor number specified : '%d' (%d ~ %d is valid)\n",
                          nSectionIndex, 0, MAX_MOTOR_COUNT-1);

            return RESULT_ERROR;
        }
        break;

    case CONFIG_SECTION_WELDER :
        // welder0 ~ welder31 are valid range
        if (nSectionIndex < 0 || nSectionIndex >= MAX_WELDER_COUNT)
        {
            VERBOSE_ERROR("invalid welder number specified : '%d' (%d ~ %d is valid)\n",
                          nSectionIndex, 0, MAX_WELDER_COUNT-1);

            return RESULT_ERROR;
        }
        break;

    case CONFIG_SECTION_PARAM :
        // param1 ~ param4 are valid range
        if (nSectionIndex <= 0 || nSectionIndex > MAX_ROBOT_COUNT)
        {
            VERBOSE_ERROR("invalid param number specified : '%d' (%d ~ %d is valid)\n",
                          nSectionIndex, 1, MAX_ROBOT_COUNT);

            return RESULT_ERROR;
        }

        nSectionIndex--;    // user uses from 1, internally uses from zero
        break;

    case CONFIG_SECTION_STATIS :
        // statis1 ~ statis4 are valid range
        if (nSectionIndex <= 0 || nSectionIndex > MAX_ROBOT_COUNT)
        {
            VERBOSE_ERROR("invalid statis number specified : '%d' (%d ~ %d is valid)\n",
                          nSectionIndex, 1, MAX_ROBOT_COUNT);

            return RESULT_ERROR;
        }

        nSectionIndex--;    // user uses from 1, internally uses from zero
        break;
    }

    if (pnSectionIndex != NULL)
        *pnSectionIndex = nSectionIndex;

    return nSectionType;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_LoadConfigCallback()
//

typedef struct
{
    int     nSectionType;
    int     nSectionIndex;
} LOAD_CONFIG_PARAM;

static int _loc_SYSC_LoadConfigCallback(int nCallCount, void* pParam, int nType,
                                        const char* pszKey, const char* pszValue)
{
    LOAD_CONFIG_PARAM* pConfigSec;

    int nResult = 0;
    int nSectionType;
    int nSectionIndex;

    nCallCount = nCallCount;
    
    pConfigSec = (LOAD_CONFIG_PARAM*) pParam;
    DANDY_ASSERT(pConfigSec != NULL);

    ///////////////////////////////////
    // Section

    if (nType == CONFIG_TYPE_SECTION)
    {
        nSectionType = _loc_SYSC_ParseSection(pszKey, &nSectionIndex);

        if (nSectionType == -1 || nSectionIndex == -1)
        {
            return CONFIG_ENUM_SYNTAX;
        }

        pConfigSec->nSectionType  = nSectionType;
        pConfigSec->nSectionIndex = nSectionIndex;

        return CONFIG_ENUM_OK;
    }

    if (nType != CONFIG_TYPE_KEY)
    {
        return CONFIG_ENUM_OK;
    }

    ///////////////////////////////////
    // Key 

    nSectionType  = pConfigSec->nSectionType;
    nSectionIndex = pConfigSec->nSectionIndex;

    switch (nSectionType)
    {
        if(g_nConfigLoadOption == OPTION_SYSCONFIG_LOAD)
        {
            case CONFIG_SECTION_GLOBAL :
                DANDY_ASSERT(nSectionIndex == 0);
                nResult = SYSC_LoadConfigGlobal(pszKey, pszValue);
                break;

            case CONFIG_SECTION_ROBOT :
                DANDY_ASSERT(nSectionIndex >= 0 && nSectionIndex < MAX_ROBOT_COUNT);
                nResult = SYSC_LoadConfigRobot(nSectionIndex, pszKey, pszValue);
                break;

            case CONFIG_SECTION_AXIS :
                DANDY_ASSERT(nSectionIndex >= 0 && nSectionIndex < MAX_AXIS_COUNT);
                nResult = SYSC_LoadConfigAxis(nSectionIndex, pszKey, pszValue);
                break;

            case CONFIG_SECTION_MOTOR :
                DANDY_ASSERT(nSectionIndex >= 0 && nSectionIndex < MAX_MOTOR_COUNT);
                nResult = SYSC_LoadConfigMotor(nSectionIndex, pszKey, pszValue);
                break;
                
            case CONFIG_SECTION_WELDER :
                DANDY_ASSERT(nSectionIndex >= 0 && nSectionIndex < MAX_WELDER_COUNT);
                nResult = SYSC_LoadConfigWelder(nSectionIndex, pszKey, pszValue);
                break;

            case CONFIG_SECTION_SENSOR :
                //DANDY_ASSERT(nSectionIndex >= 0 && nSectionIndex < MAX_SENSOR_COUNT);
                //nResult = SYSC_LoadConfigSensor(pszKey, pszValue);
                break;
        }
        else if(g_nConfigLoadOption == OPTION_USERPARAM_LOAD)
        {
            case CONFIG_SECTION_PARAM :
                DANDY_ASSERT(nSectionIndex >= 0 && nSectionIndex < MAX_ROBOT_COUNT);
                nResult = SYSC_LoadConfigRobotParameter(nSectionIndex, pszKey, pszValue);
                break;
        }
        else if(g_nConfigLoadOption == OPTION_STATISTIC_LOAD)
        {
            case CONFIG_SECTION_STATIS :
                DANDY_ASSERT(nSectionIndex >= 0 && nSectionIndex < MAX_ROBOT_COUNT);
                nResult = SYSC_LoadSystemStatistics(nSectionIndex, pszKey, pszValue);
                break;
        }

    default :
        // unknown section
        nResult = 0;
    }

    return (nResult == -1) ? CONFIG_ENUM_SYNTAX : CONFIG_ENUM_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_FindOsNameConfigFile()
//

char* _loc_SYSC_FindOsNameConfigFile(const char* pszDirectory,
                                     const char* pszBaseName, const char* pszExt,
                                     char* pszPathName)
{
#if defined(_WIN32)
    static const char* rgpszOSName[] = { ".win", ".dos", NULL};

    int i;

    for (i = 0; rgpszOSName[i] != NULL; i++)
    {
        if (FindFullPathName(pszBaseName, rgpszOSName[i], pszExt,
                             pszDirectory, pszPathName, 256) != -1)
        {
#if defined(SYSC_VERBOSE)
        VERBOSE_VERBOSE("Found '%s' config file\n", pszBaseName);
        VERBOSE_VERBOSE("Found '%s' system Name\n", rgpszOSName[i]);
        VERBOSE_VERBOSE("Found '%s' path Name\n", pszPathName);
#endif
            return pszPathName;
        }
    }

    return NULL;

#else
    char szUnixName[128];
    struct utsname sysinfo;

    // check os dependant config file name
    // <pszDirectory>/<pszBaseName>.<`uname`>.<pszExt>

    if (uname(&sysinfo) == -1)
    {
        VERBOSE_ERROR("Fails to get uname : %d\n", errno);
        return NULL;
    }

    szUnixName[0] = '.';        // ".qnx"
    strcpy(szUnixName + 1, sysinfo.sysname);
    strlwr(szUnixName);

    if (FindFullPathName(pszBaseName, szUnixName, pszExt,
                         pszDirectory, pszPathName, 256) == -1)
    {
#if defined(SYSC_VERBOSE)
        VERBOSE_VERBOSE("Found '%s' file Name\n", pszBaseName);
        VERBOSE_VERBOSE("Found '%s' system Name\n", szUnixName);
        VERBOSE_VERBOSE("Found '%s' path Name\n", pszPathName);
#endif
        return pszPathName;
    }

    return NULL;
#endif
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_SYSC_FindConfigFile()
//

#if defined(_WIN32)
static const char* s_rgpszConfigSearchDirectory[] = { ".", "..", NULL};
static const char* s_rgpszConfigSearchExtension[] = { ".conf", ".cfg", ".ini", NULL};

#else
static const char* s_rgpszConfigSearchDirectory[] = { ".", "~", "..", NULL};
static const char* s_rgpszConfigSearchExtension[] = { ".conf", ".cfg", NULL};
#endif

static char* _loc_SYSC_FindConfigFile(char* pszFileName)
{
    static char* pszConfig = NULL;
    int i, j;

    ///////////////////////////////////
    // try to find the file is defined in environment variable
#if defined(_WIN32)
    size_t pRquiredSize = 0;
    getenv_s(&pRquiredSize, NULL, 0, CONFIG_ENV_NAME);
    if(pRquiredSize != 0)
    {
        pszConfig = (char *)DEBUG_MALLOC(pRquiredSize* sizeof(char));
    }
    else
    {
        pszConfig = (char *)DEBUG_MALLOC(CONF_FILENAME_LEN);
    }
    getenv_s(&pRquiredSize, pszConfig, CONF_FILENAME_LEN, CONFIG_ENV_NAME);
#else
    pszConfig = getenv(CONFIG_ENV_NAME);
#endif

    if (pszConfig != NULL && sizeof(pszConfig) > 4)
    {
        VERBOSE_VERBOSE("config file is specifed in environment : %s\n",
                         CONFIG_ENV_NAME);
        //VERBOSE_VERBOSE("pszConfig : \"%s\",size: %d\n", pszConfig, sizeof(pszConfig));
    }

    if ((pszConfig != NULL && sizeof(pszConfig) > 4) &&
        CRT_access(pszConfig, 04) == 0)
    {
        // found the config file which is described in environment variable
        CRT_strcpy(pszFileName, CONF_FILENAME_LEN, pszConfig);
        DEBUG_FREE(pszConfig);
        pszConfig = NULL;
        return pszFileName;
    }

    if (pszConfig != NULL && sizeof(pszConfig) > 4)
    {
        // config file was defined in environment variable, but could not access
        VERBOSE_ERROR("Cannot read config file %s\n", pszConfig);
    }

    ///////////////////////////////////
    // try to access OS dependant configuration file
    //  searching order :
    //      (osn = operating system name, getting from uname(). osn=win in Windows )
    //          ./DEF_CONFIG_BASENAME.osn.conf
    //          ./DEF_CONFIG_BASENAME.osn.cfg
    //          ~/DEF_CONFIG_BASENAME.osn.conf          (unix only, not in Windows)
    //          ~/DEF_CONFIG_BASENAME.osn.cfg           (unix only, not in Windows)
    //          /etc/DEF_CONFIG_BASENAME.osn.conf       (unix only, not in Windows)
    //          /etc/DEF_CONFIG_BASENAME.osn.cfg        (unix only, not in Windows)
    //          ../DEF_CONFIG_BASENAME.osn.conf
    //          ../DEF_CONFIG_BASENAME.osn.cfg
    //
    DEBUG_FREE(pszConfig);    //important for memory leak
    pszConfig = NULL;

    for (i = 0; pszConfig == NULL && s_rgpszConfigSearchDirectory[i] != NULL; i++)
    {
        for (j = 0; pszConfig == NULL && s_rgpszConfigSearchExtension[j] != NULL; j++)
        {
            // add os name (.win, .qnx, ...)
            pszConfig = _loc_SYSC_FindOsNameConfigFile(s_rgpszConfigSearchDirectory[i],
                                                       DEF_CONFIG_BASENAME,
                                                       s_rgpszConfigSearchExtension[j],
                                                       pszFileName);
            if (pszConfig != NULL)
            {
#if defined (SYSC_VERBOSE)
                VERBOSE_VERBOSE("Config String: %s\n", pszConfig);
#endif
                break;
            }
        }
    }

    if (pszConfig != NULL && 
        strncmp(pszConfig, DEF_CONFIG_BASENAME, sizeof(DEF_CONFIG_BASENAME)) == 0)
    {
        CRT_strcpy(pszFileName, CONF_FILENAME_LEN, pszConfig);
        pszConfig = NULL;
        return pszFileName;
    }
    
    ///////////////////////////////////
    // try to access default configuration file (not related OS name)
    //  searching order :
    //          ./DEF_CONFIG_BASENAME.conf
    //          ./DEF_CONFIG_BASENAME.cfg
    //          ~/DEF_CONFIG_BASENAME.conf      (unix only, not in Windows)
    //          ~/DEF_CONFIG_BASENAME.cfg       (unix only, not in Windows)
    //          /etc/DEF_CONFIG_BASENAME.conf   (unix only, not in Windows)
    //          /etc/DEF_CONFIG_BASENAME.cfg    (unix only, not in Windows)
    //          ../DEF_CONFIG_BASENAME.conf
    //          ../DEF_CONFIG_BASENAME.cfg
    //
    //DEBUG_FREE(pszConfig);
    pszConfig = NULL;

    for (i = 0; pszConfig == NULL && s_rgpszConfigSearchDirectory[i] != NULL; i++)
    {
        for (j = 0; pszConfig == NULL && s_rgpszConfigSearchExtension[j] != NULL; j++)
        {
            if (FindFullPathName(DEF_CONFIG_BASENAME, NULL,
                                 s_rgpszConfigSearchExtension[j],
                                 s_rgpszConfigSearchDirectory[i],
                                 pszFileName, 256) != -1)
            {
                pszConfig = pszFileName;
#if defined (SYSC_VERBOSE)
                VERBOSE_VERBOSE("Default Config String: %s\n", pszConfig);
#endif
            }
        }
    }

    return pszConfig;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_LoadConfig()
//

int SYSC_LoadConfig(const char* pszConfigFileName, int g_nConfigLoadOption)
{
#if defined(_DEBUG)
    static int nLoadCount = 0;
#endif

    char szConfig[CONF_FILENAME_LEN];

    LOAD_CONFIG_PARAM param;
    int nResult;

    if (pszConfigFileName == NULL)
    {
        // if user was not specified the config file name,
        //      find default config file.
        pszConfigFileName = _loc_SYSC_FindConfigFile(szConfig);
#if defined(SYSC_VERBOSE)
        VERBOSE_VERBOSE("Env Config File Name: %s\n", pszConfigFileName);
#endif
    }

    if (pszConfigFileName == NULL)
    {
        // use default file name if could not find config file
        pszConfigFileName = DEF_CONFIG_BASENAME ".conf";
#if defined(SYSC_VERBOSE)
        VERBOSE_VERBOSE("Default Config File Name: %s\n", pszConfigFileName);
#endif
    }

    ///////////////////////////////////

#if defined(_DEBUG)
    nLoadCount++;
#endif

    DANDY_ASSERT(pszConfigFileName != NULL);

    // clear all configurations
    if(g_nConfigLoadOption == OPTION_SYSCONFIG_LOAD)
    {
        SYSC_ClearConfig();
    }

    // load configurations from file
    VERBOSE_VERBOSE("Try to load configuration from '%s' file\n",
                     pszConfigFileName);
    
    if(g_nConfigLoadOption == OPTION_SYSCONFIG_LOAD)
    {
        param.nSectionType  = CONFIG_SECTION_GLOBAL;
        param.nSectionIndex = 0;
    }
    else if(g_nConfigLoadOption == OPTION_USERPARAM_LOAD)
    {
        param.nSectionType  = CONFIG_SECTION_PARAM;
        param.nSectionIndex = 0;
    }
    else if(g_nConfigLoadOption == OPTION_STATISTIC_LOAD)
    {
        param.nSectionType  = CONFIG_SECTION_STATIS;
        param.nSectionIndex = 0;
    }

    nResult = CONF_EnumConfigFile(pszConfigFileName, TRUE,
                                  _loc_SYSC_LoadConfigCallback,
                                  (void*) &param);

    if(g_nConfigLoadOption == OPTION_SYSCONFIG_LOAD ||
       g_nConfigLoadOption == OPTION_USERPARAM_LOAD)
    {
        if (nResult == CONFIG_ENUM_NO_FILE)
        {
            VERBOSE_ERROR("Fails to load the configurations\n");
            return RESULT_ERROR;
        }
        else if (nResult == CONFIG_ENUM_SYNTAX)
        {
            VERBOSE_ERROR("configuration file syntax error : %s\n",
                           pszConfigFileName);
            
            g_fConfigLoadCheck = RESULT_ERROR;

            return RESULT_ERROR;
        }
        else if (nResult != CONFIG_ENUM_OK)
        {
            VERBOSE_WARNING("configuration file error : %d\n", nResult);

            return RESULT_ERROR;
        }
    }
    else if(g_nConfigLoadOption == OPTION_STATISTIC_LOAD)
    {
        if (nResult == CONFIG_ENUM_NO_FILE)
        {
            VERBOSE_WARNING("Fails to load statistics\n");
            g_fLoadStatFile = RESULT_ERROR;

            return RESULT_ERROR;
        }
        else if (nResult == CONFIG_ENUM_SYNTAX)
        {
            VERBOSE_WARNING("statistics file syntax error : %s\n",
                            pszConfigFileName);
            g_fLoadStatFile = RESULT_ERROR;

            return RESULT_ERROR;
        }
        else if (nResult != CONFIG_ENUM_OK)
        {
            VERBOSE_WARNING("statistics file error : %d\n", nResult);
            
            return RESULT_ERROR;
        }
    }

    if(g_nConfigLoadOption == OPTION_SYSCONFIG_LOAD)
    {
        VERBOSE_VERBOSE("Load Read-Only Configuration Parameters Load Done!\n");
    }
    else if(g_nConfigLoadOption == OPTION_USERPARAM_LOAD)
    {
        VERBOSE_VERBOSE("Load Writable User Parameters Load Done!\n");
    }
    else if(g_nConfigLoadOption == OPTION_STATISTIC_LOAD)
    {
        VERBOSE_VERBOSE("Load Statistics Data Load Done!\n");
        g_fLoadStatFile = RESULT_OK;
    }

    VERBOSE_VERBOSE("Successfully configuration is loaded\n");

    g_fConfigLoadCheck = RESULT_OK;

    return RESULT_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_SetTrajUpdateTime()
//

void SYSC_SetTrajUpdateTime(int nTrajTime)
{
    g_nTrajUpdateTime = nTrajTime;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_SetIoUpdateTime()
//

void SYSC_SetIoUpdateTime(int nIoTime)
{
    g_nIoTime = nIoTime;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: SYSC_ExecLocale()
//

int SYSC_ExecLocale(void)
{
    const char* pszLocale;

    pszLocale = setlocale(LC_ALL, "");

    if (pszLocale != NULL)
        VERBOSE_VERBOSE("current locale is '%s'\n", pszLocale);
    else
        VERBOSE_VERBOSE("current locale was undefined\n");

    if (g_pszLocale != NULL)
    {
        pszLocale = setlocale(LC_ALL, g_pszLocale);

        if (pszLocale != NULL)
        {
            VERBOSE_VERBOSE("System locale changed to '%s' by '%s'\n",
                             pszLocale, g_pszLocale);
        }
        else
        {
            VERBOSE_ERROR("System locale setting '%s' failure\n", g_pszLocale);
            return RESULT_ERROR;
        }
    }

    return 0;
}

