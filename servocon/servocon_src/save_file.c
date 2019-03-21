/////////////////////////////////////////////////////////////////////////////
//
//  save_file.c: System Data Save To File
//                                            2014.03.19  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES
#include "service.h"
#include <math.h>

///////////////////////////////////////


///////////////////////////////////////
//Global_variable
CURRENT_TIME    curr_time;

/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
//
//  Function: Fn_GetSystemTime(void)
//      - Function to get system time
void Fn_GetSystemTime(void)
{
    struct tm tm;

    TIME_GetLocalTime(&tm);

    curr_time.nYear   = tm.tm_year + 1900;
    curr_time.nMonth  = tm.tm_mon + 1;
    curr_time.nDay    = tm.tm_mday;
    curr_time.nHour   = tm.tm_hour;
    curr_time.nMinute = tm.tm_min;
    curr_time.nSec    = tm.tm_sec;
}


///////////////////////////////////////
//
//  Function: SERV_ArcSensorDataFileSave(void)
//      - Service: SC_ARCSENS_FILE_SAVE

#define ARCSENS_MOVAVR_LOG_FILENAME   "mdata"
#define ARCSENS_DELTA_LOG_FILENAME    "cdata"
#define ARCSENS_RDATA_LOG_FILENAME    "rdata"
#define ARCSENS_RDATA2_LOG_FILENAME   "rdata2"
#define ARCSENS_RDATA3_LOG_FILENAME   "rdata3"
#define ARCSENS_WEAVPOS_FILENAME      "weavpos"
#define ARCSENS_WELDCOND_FILENAME     "weldcond"
#define POSITION_LOG_FILENAME         "pos"
#define IO_LOG_FILENAME               "io"
#define ARCSENS_FILE_EXTENT           ".dat"

int  SERV_ArcSensorDataFileSave(int nOpt)
{
    unsigned     iIdx;
    unsigned     iIdx2;
    char         szSysTime[SYSTIME_DATA_LEN];
    
    FILE*   fp_SensMovA = 0;
    FILE*   fp_SensDelt = 0;
    FILE*   fp_SensRdat = 0;
    FILE*   fp_WeavPos  = 0;
    FILE*   fp_WeldCond = 0;
    FILE*   fp_Position = 0;
    FILE*   fp_IO       = 0;
    FILE*   fp_SensRdat2= 0;
    FILE*   fp_SensRdat3= 0;

    char szArcSensMovavrFileName[PATH_NAME_BUFFER_SIZE]   = "";
    char szArcSensDeltaFileName[PATH_NAME_BUFFER_SIZE]    = "";
    char szArcSensRDataFileName[PATH_NAME_BUFFER_SIZE]    = "";
    char szArcSensWeavPosFileName[PATH_NAME_BUFFER_SIZE]  = "";
    char szArcSensWeldCondFileName[PATH_NAME_BUFFER_SIZE] = "";
    char szPositionLogFileName[PATH_NAME_BUFFER_SIZE]     = "";
    char szIOLogFileName[PATH_NAME_BUFFER_SIZE]           = "";

    char szArcSensRData2FileName[PATH_NAME_BUFFER_SIZE]   = "";
    char szArcSensRData3FileName[PATH_NAME_BUFFER_SIZE]   = "";
    char szRDataCopyFileName[PATH_NAME_BUFFER_SIZE]       = "";
    char szRDataCopyCmdName[PATH_NAME_BUFFER_SIZE]        = "";

#if 0
#if defined(__QNXNTO__)
    char szSensDataDir[PATH_NAME_BUFFER_SIZE] = "/works/sdata/";
#else
    char szSensDataDir[PATH_NAME_BUFFER_SIZE] = "./sdata/";
#endif
#endif
    char szSensDataDir[PATH_NAME_BUFFER_SIZE];

#if defined(_MSC_VER)
    errno_t errno;
#endif

    memcpy(szSensDataDir, g_pszSensDir, PATH_NAME_BUFFER_SIZE);

    Fn_GetSystemTime();

    CRT_sprintf(szSysTime, SYSTIME_DATA_LEN,
                "%04d%02d%02d_%02d%02d%02d_",
                curr_time.nYear,
                curr_time.nMonth,
                curr_time.nDay,
                curr_time.nHour,
                curr_time.nMinute,
                curr_time.nSec);

    /* Open File */
    if(nOpt == 0 || nOpt == 1)
    {
        if(!fp_SensMovA)
        {
            memcpy(szArcSensMovavrFileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szArcSensMovavrFileName, PATH_NAME_BUFFER_SIZE, szSysTime);
            CRT_strcat(szArcSensMovavrFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_MOVAVR_LOG_FILENAME);
            CRT_strcat(szArcSensMovavrFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);
            
            remove(szArcSensMovavrFileName);

#if defined(__QNXNTO__)
            fp_SensMovA  = fopen(szArcSensMovavrFileName, "w+t");
#else
            errno=fopen_s(&fp_SensMovA , szArcSensMovavrFileName, "w+t");
#endif
            if (fp_SensMovA == NULL)
            {
                VERBOSE_ERROR("Cannot open ArcSensor Moving Average file for write : '%s'\n",
                              szArcSensMovavrFileName);

                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

                return RESULT_ERROR;
            }
        }

        if(!fp_SensDelt)
        {
            memcpy(szArcSensDeltaFileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szArcSensDeltaFileName, PATH_NAME_BUFFER_SIZE, szSysTime);
            CRT_strcat(szArcSensDeltaFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_DELTA_LOG_FILENAME);
            CRT_strcat(szArcSensDeltaFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);

            remove(szArcSensDeltaFileName);

#if defined(__QNXNTO__)
            fp_SensDelt  = fopen(szArcSensDeltaFileName,  "w+t");
#else
            errno=fopen_s(&fp_SensDelt , szArcSensDeltaFileName,  "w+t");
#endif
            if (fp_SensDelt == NULL)
            {
                VERBOSE_ERROR("Cannot open ArcSensor Delta Data file for write : '%s'\n",
                              szArcSensDeltaFileName);

                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

                return RESULT_ERROR;
            }
        }

        if(!fp_SensRdat)
        {
            memcpy(szArcSensRDataFileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szArcSensRDataFileName, PATH_NAME_BUFFER_SIZE, szSysTime);
            CRT_strcat(szArcSensRDataFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_RDATA_LOG_FILENAME);
            CRT_strcat(szArcSensRDataFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);

            remove(szArcSensRDataFileName);

#if defined(__QNXNTO__)
            fp_SensRdat  = fopen(szArcSensRDataFileName,  "w+t");
#else
            errno=fopen_s(&fp_SensRdat , szArcSensRDataFileName,  "w+t");
#endif
            if (fp_SensRdat == NULL)
            {
                VERBOSE_ERROR("Cannot open ArcSensor Real Data file for write : '%s'\n",
                              szArcSensRDataFileName);

                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

                return RESULT_ERROR;
            }
        }

        if(!fp_SensRdat2)
        {
            memcpy(szArcSensRData2FileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szArcSensRData2FileName, PATH_NAME_BUFFER_SIZE, szSysTime);
            CRT_strcat(szArcSensRData2FileName, PATH_NAME_BUFFER_SIZE, ARCSENS_RDATA2_LOG_FILENAME);
            CRT_strcat(szArcSensRData2FileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);

            remove(szArcSensRData2FileName);

#if defined(__QNXNTO__)
            fp_SensRdat2  = fopen(szArcSensRData2FileName,  "w+t");
#else
            errno=fopen_s(&fp_SensRdat2 , szArcSensRData2FileName,  "w+t");
#endif
            if (fp_SensRdat2 == NULL)
            {
                VERBOSE_ERROR("Cannot open ArcSensor Real Data2 file for write : '%s'\n",
                              szArcSensRData2FileName);

                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

                return RESULT_ERROR;
            }
        }

        if(!fp_SensRdat3)
        {
            memcpy(szArcSensRData3FileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szArcSensRData3FileName, PATH_NAME_BUFFER_SIZE, szSysTime);
            CRT_strcat(szArcSensRData3FileName, PATH_NAME_BUFFER_SIZE, ARCSENS_RDATA3_LOG_FILENAME);
            CRT_strcat(szArcSensRData3FileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);

            remove(szArcSensRData3FileName);

#if defined(__QNXNTO__)
            fp_SensRdat3  = fopen(szArcSensRData3FileName,  "w+t");
#else
            errno=fopen_s(&fp_SensRdat3 , szArcSensRData3FileName,  "w+t");
#endif
            if (fp_SensRdat2 == NULL)
            {
                VERBOSE_ERROR("Cannot open ArcSensor Real Data3 file for write : '%s'\n",
                              szArcSensRData3FileName);

                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

                return RESULT_ERROR;
            }
        }

        if(!fp_WeavPos)
        {
            memcpy(szArcSensWeavPosFileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szArcSensWeavPosFileName, PATH_NAME_BUFFER_SIZE, szSysTime);
            CRT_strcat(szArcSensWeavPosFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_WEAVPOS_FILENAME);
            CRT_strcat(szArcSensWeavPosFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);

            remove(szArcSensWeavPosFileName);

#if defined(__QNXNTO__)
            fp_WeavPos  = fopen(szArcSensWeavPosFileName,  "w+t");
#else
            errno=fopen_s(&fp_WeavPos , szArcSensWeavPosFileName,  "w+t");
#endif
            if (fp_WeavPos == NULL)
            {
                VERBOSE_ERROR("Cannot open ArcSensor Weaving Position file for write : '%s'\n",
                              szArcSensWeavPosFileName);

                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

                return RESULT_ERROR;
            }
        }

        if(!fp_WeldCond)
        {
            memcpy(szArcSensWeldCondFileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szArcSensWeldCondFileName, PATH_NAME_BUFFER_SIZE, szSysTime);
            CRT_strcat(szArcSensWeldCondFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_WELDCOND_FILENAME);
            CRT_strcat(szArcSensWeldCondFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);

            remove(szArcSensWeldCondFileName);

#if defined(__QNXNTO__)
            fp_WeldCond  = fopen(szArcSensWeldCondFileName,  "w+t");
#else
            errno=fopen_s(&fp_WeldCond , szArcSensWeldCondFileName,  "w+t");
#endif
            if (fp_WeldCond == NULL)
            {
                VERBOSE_ERROR("Cannot open ArcSensor Weld Condition file for write : '%s'\n",
                              szArcSensWeldCondFileName);

                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

                return RESULT_ERROR;
            }
        }
    }
    //if(nOpt == 0 || nOpt == 2)
    if(nOpt == 2)
    {
        if(!fp_Position)
        {
            memcpy(szPositionLogFileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szPositionLogFileName, PATH_NAME_BUFFER_SIZE, szSysTime);
            CRT_strcat(szPositionLogFileName, PATH_NAME_BUFFER_SIZE, POSITION_LOG_FILENAME);
            CRT_strcat(szPositionLogFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);

            remove(szPositionLogFileName);

#if defined(__QNXNTO__)
            fp_Position  = fopen(szPositionLogFileName,  "w+t");
#else
            errno=fopen_s(&fp_Position , szPositionLogFileName,  "w+t");
#endif
            if (fp_Position == NULL)
            {
                VERBOSE_ERROR("Cannot open Axis Postion Data file for write : '%s'\n",
                              szPositionLogFileName);

                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

                return RESULT_ERROR;
            }
        }
    }
    //if(nOpt == 0 || nOpt == 3)
    if(nOpt == 3)
    {
        if(!fp_IO)
        {
            memcpy(szIOLogFileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szIOLogFileName, PATH_NAME_BUFFER_SIZE, szSysTime);
            CRT_strcat(szIOLogFileName, PATH_NAME_BUFFER_SIZE, IO_LOG_FILENAME);
            CRT_strcat(szIOLogFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);

            remove(szIOLogFileName);

#if defined(__QNXNTO__)
            fp_IO  = fopen(szIOLogFileName,  "w+t");
#else
            errno=fopen_s(&fp_IO , szIOLogFileName,  "w+t");
#endif
            if (fp_IO == NULL)
            {
                VERBOSE_ERROR("Cannot open I/O Data file for write : '%s'\n",
                              szIOLogFileName);

                g_pShmem_sc->sysstate.fErrorState = TRUE;
                g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

                return RESULT_ERROR;
            }
        }
    }

    /* Write Data & Close File */
    if(nOpt == 0 || nOpt == 1)
    {
        if(g_pShmem_Task_te != NULL && g_pShmem_SysParam_rm != NULL)
        {
            // ArcSensor Moving Average Data
    	    for (iIdx = 0; iIdx < g_pShmem_Task_te->arc_save_num; iIdx++)
            {
    	    	fprintf(fp_SensMovA, "%10.3f %10.3f\n",
                        g_pShmem_Task_te->Mean_Moving_Weight[iIdx],
                        g_pShmem_Task_te->Mean_Moving_Ampere[iIdx]);
            }

    	    fclose(fp_SensMovA);
            VERBOSE_MESSAGE("ArcSensor Moving Average file write Done!: '%s'\n",
                             szArcSensMovavrFileName);
           

            // ArcSensor Delta Data
    	    for (iIdx = 0; iIdx< g_pShmem_Task_te->arc_save_num; iIdx++)
            {
    	    	fprintf(fp_SensDelt, "%10.3f %10.3f\n",
                        g_pShmem_Task_te->Delta_Z[iIdx],
                        g_pShmem_Task_te->Delta_T[iIdx]);
            }

    	    fclose(fp_SensDelt);
            VERBOSE_MESSAGE("ArcSensor Delta Data file write Done!: '%s'\n",
                             szArcSensDeltaFileName);


            // ArcSensor Real Data
    	    for (iIdx = 0;
                (iIdx < g_pShmem_Task_te->arc_save_num - g_pShmem_SysParam_rm->arcsensor[0].nStartSaveNodeNo) &&
                (iIdx < DEF_NODE_ADD);//g_pShmem_SysParam_rm->arcsensor[0].nSaveNodeCount);
                 iIdx++)
    	    {
    	    	//for(iIdx2 = 0; iIdx2 < RDATA_SIZE; iIdx2++)
                for(iIdx2 = 0; iIdx2 < g_pShmem_Task_te->n_rdata[iIdx]; iIdx2++)
                {
    	    		fprintf(fp_SensRdat, "%d %d", iIdx, (int)g_pShmem_Task_te->Rdata[iIdx][iIdx2]);
                    fprintf(fp_SensRdat,"\n");
                }
    	    	//fprintf(fp_SensRdat,"\n");
    	    }

    	    fclose(fp_SensRdat);
            VERBOSE_MESSAGE("ArcSensor Real Data file write Done!: '%s'\n",
                             szArcSensRDataFileName);


            // Weaving Position Data
    	    for (iIdx = 0; iIdx< g_pShmem_Task_te->arc_save_num; iIdx++)
            {
    	    	fprintf(fp_WeavPos, "%10.3f %10.3f %10.3f\n",
                        g_pShmem_Task_te->d_xw[iIdx],    
                        g_pShmem_Task_te->d_yw[iIdx],
                        g_pShmem_Task_te->d_zw[iIdx]);
            }

    	    fclose(fp_WeavPos);
            VERBOSE_MESSAGE("ArcSensor Weaving Position Data file write Done!: '%s'\n",
                             szArcSensWeavPosFileName);


            // ArcSensor Weld Condition Data
                // Job Name
            fprintf(fp_WeldCond, "# Job Name: %s\n\n",
                    g_pShmem_SysStatus_rm->szCurrJobFileName);
                
                // Command Welding Condition
            fprintf(fp_WeldCond, "# Commmand Welding Condition\n");
            fprintf(fp_WeldCond, "# Cmd Volt, Cmd Curr\n");
            fprintf(fp_WeldCond, "%10.3f, %10.3f\n\n",
                        g_pShmem_Task_te->Vw,
                        g_pShmem_Task_te->Iw);

                // Command Weaving Condition
            fprintf(fp_WeldCond, "# Commmand Weaving Condition\n");
            fprintf(fp_WeldCond, "# Pitch, Width, Speed, Dwell Even, Dwell Odd\n");
            fprintf(fp_WeldCond, "%10.3f, %10.3f, %10.3f, %10.3f, %10.3f\n\n",
                        g_pShmem_Task_te->pitch,
                        g_pShmem_Task_te->width,
                        g_pShmem_Task_te->speed,
                        g_pShmem_Task_te->dwl_e,
                        g_pShmem_Task_te->dwl_o);

                // Feedback Welding Condition
            fprintf(fp_WeldCond, "# Feedback Welding Condition\n");
            fprintf(fp_WeldCond, "# Feedback Volt, Feedback Curr\n");
            for (iIdx = 0; iIdx< g_pShmem_Task_te->arc_save_num; iIdx++)
            {
                fprintf(fp_WeldCond, "%d %10.3f %10.3f\n",
                        iIdx,
                        g_dbWeldVoltInVal[iIdx] * 100.0,
                        g_dbWeldCurrInVal[iIdx] * 100.0);
            }

    	    fclose(fp_WeldCond);
            VERBOSE_MESSAGE("ArcSensor Weld Condition Data file write Done!: '%s'\n",
                             szArcSensWeldCondFileName);

            // ArcSensor Real Data2
    	    for (iIdx = 0;
                (iIdx < g_pShmem_Task_te->arc_save_num - g_pShmem_SysParam_rm->arcsensor[0].nStartSaveNodeNo) &&
                (iIdx < DEF_NODE_ADD);//g_pShmem_SysParam_rm->arcsensor[0].nSaveNodeCount);
                 iIdx++)
    	    {
                fprintf(fp_SensRdat2, "%d", iIdx);

    	    	//for(iIdx2 = 0; iIdx2 < RDATA_SIZE; iIdx2++)
                for(iIdx2 = 0; iIdx2 < g_pShmem_Task_te->n_rdata[iIdx]; iIdx2++)
                {
    	    		fprintf(fp_SensRdat2, " %d", (int)g_pShmem_Task_te->Rdata[iIdx][iIdx2]);
                }

    	    	fprintf(fp_SensRdat2,"\n");
    	    }

    	    fclose(fp_SensRdat2);
            VERBOSE_MESSAGE("ArcSensor Real Data file2 write Done!: '%s'\n",
                             szArcSensRData2FileName);


            // ArcSensor Real Data3
    	    for (iIdx = 0;
                (iIdx < g_pShmem_Task_te->arc_save_num - g_pShmem_SysParam_rm->arcsensor[0].nStartSaveNodeNo) &&
                (iIdx < DEF_NODE_ADD);//g_pShmem_SysParam_rm->arcsensor[0].nSaveNodeCount);
                 iIdx++)
    	    {
    	    	for(iIdx2 = 0; iIdx2 < RDATA_SIZE; iIdx2++)
                {
    	    		fprintf(fp_SensRdat3, "%d %d", iIdx, (int)g_pShmem_Task_te->Rdata[iIdx][iIdx2]);
                    fprintf(fp_SensRdat3, "\n");
                }
    	    	//fprintf(fp_SensRdat,"\n");
    	    }

    	    fclose(fp_SensRdat3);
            VERBOSE_MESSAGE("ArcSensor Real Data file3 write Done!: '%s'\n",
                             szArcSensRData3FileName);


            // Copy Rdata to Copy File
            memcpy(szRDataCopyFileName,  szSensDataDir, strlen(szSensDataDir)+1);
            CRT_strcat(szRDataCopyFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_RDATA_LOG_FILENAME);
            CRT_strcat(szRDataCopyFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);

            CRT_sprintf(szRDataCopyCmdName,
                        PATH_NAME_BUFFER_SIZE,
                        "cp %s %s",
                        szArcSensRData3FileName,
                        szRDataCopyFileName);

            system(szRDataCopyCmdName);
            VERBOSE_MESSAGE("Rdata Copy Done!(%s)\n", szRDataCopyCmdName);
        }
        else
        {
            VERBOSE_WARNING("TE Shared Memory Not Loaded! ArcSensor Data file write Fail!\n");
            fclose(fp_SensMovA);
            fclose(fp_SensDelt);
            fclose(fp_SensRdat);
            fclose(fp_WeavPos);
            fclose(fp_WeldCond);
            fclose(fp_SensRdat2);
            fclose(fp_SensRdat3);
        }
    }

    //if(nOpt == 0 || nOpt == 2)
    if(nOpt == 2)
    {
        fclose(fp_Position);
    }

    //if(nOpt == 0 || nOpt == 3)
    if(nOpt == 3)
    {
        fclose(fp_IO);
    }

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SERV_ArcSensorRDataFileLoad()
//      - Service Name: SC_POS_FILE_WRITE

#define ARCSENS_RDATA_TEST_FILENAME     "rdata_test"

int  SERV_ArcSensorRDataFileLoad(void)
{
    unsigned     iIdx;
    unsigned     iIdx2;
    unsigned     nSave_Idx[DEF_NODE_ADD][RDATA_SIZE];
    
    FILE*   fp = 0;
    FILE*   fp_SensRdat = 0;

    char szArcSensRDataFileName[PATH_NAME_BUFFER_SIZE]    = "";
    char szArcSensRDataTestFileName[PATH_NAME_BUFFER_SIZE]    = "";
#if 0
#if defined(__QNXNTO__)
    char szSensDataDir[PATH_NAME_BUFFER_SIZE] = "/works/sdata/";
#else
    char szSensDataDir[PATH_NAME_BUFFER_SIZE] = "./sdata/";
#endif
#endif
    char szSensDataDir[PATH_NAME_BUFFER_SIZE];

    memcpy(szSensDataDir, g_pszSensDir, PATH_NAME_BUFFER_SIZE);

    /* Load Data From File */
    memcpy(szArcSensRDataFileName,  szSensDataDir, strlen(szSensDataDir)+1);
    CRT_strcat(szArcSensRDataFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_RDATA_LOG_FILENAME);
    CRT_strcat(szArcSensRDataFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);
    
#if defined(__QNXNTO__)
    fp  = fopen(szArcSensRDataFileName, "rt");
#else
    errno=fopen_s(&fp , szArcSensRDataFileName, "rt");
#endif
    if (fp == NULL)
    {
        VERBOSE_ERROR("Cannot open ArcSensor Rdata file for read : '%s'\n",
                      szArcSensRDataFileName);

        g_pShmem_sc->sysstate.fErrorState = TRUE;
        g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

        return RESULT_ERROR;
    }

    for (iIdx = 0; iIdx < DEF_NODE_ADD; iIdx++)
    {
        for(iIdx2 = 0; iIdx2 < RDATA_SIZE; iIdx2++)
        {
#if defined(__QNXNTO__)
            fscanf(fp, "%d", &nSave_Idx[iIdx][iIdx2]);
            fscanf(fp, "%d", (int*)&(g_pShmem_Task_te->Rdata[iIdx][iIdx2]));
#else
            fscanf_s(fp, "%d", &nSave_Idx[iIdx][iIdx2], 4096);
            fscanf_s(fp, "%d", (int*)&(g_pShmem_Task_te->Rdata[iIdx][iIdx2]), 4096);
#endif
        }
    }

    VERBOSE_MESSAGE("ArcSensor RData file read Done!: '%s'\n",
                     szArcSensRDataFileName);

    fclose(fp);

#if 0
    // ArcSensor Real Data
    for (iIdx = 0;
        (iIdx < g_pShmem_Task_te->arc_save_num - g_pShmem_SysParam_rm->arcsensor[0].nStartSaveNodeNo) &&
        (iIdx < DEF_NODE_ADD);
         iIdx++)
    {
    	for(iIdx2 = 0; iIdx2 < RDATA_SIZE; iIdx2++)
        {
    	    fscanf_s(fp, "%d %d", iIdx, (int)g_pShmem_Task_te->Rdata[iIdx][iIdx2], );
            fscanf_s(fp, "\n");
        }
    	//fprintf(fp,"\n");
    }
#endif

    /* Test Copy File */
    memcpy(szArcSensRDataTestFileName,  szSensDataDir, strlen(szSensDataDir)+1);
    CRT_strcat(szArcSensRDataTestFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_RDATA_TEST_FILENAME);
    CRT_strcat(szArcSensRDataTestFileName, PATH_NAME_BUFFER_SIZE, ARCSENS_FILE_EXTENT);
#if defined(__QNXNTO__)
    fp_SensRdat  = fopen(szArcSensRDataTestFileName,  "w+t");
#else
    errno=fopen_s(&fp_SensRdat , szArcSensRDataTestFileName,  "w+t");
#endif

    if (fp_SensRdat == NULL)
    {
        VERBOSE_ERROR("Cannot open ArcSensor Rdata test file for read : '%s'\n",
                      szArcSensRDataTestFileName);

        g_pShmem_sc->sysstate.fErrorState = TRUE;
        g_pShmem_sc->sysstate.nErrorCode = SVC_ERR_SENSDATA_SAVING;

        return RESULT_ERROR;
    }

    // ArcSensor Real Data
    for (iIdx = 0; (iIdx < DEF_NODE_ADD); iIdx++)
    {
        fprintf(fp_SensRdat, "%d", iIdx);

    	for(iIdx2 = 0; iIdx2 < RDATA_SIZE; iIdx2++)
        {
            fprintf(fp_SensRdat, " %d", (int)g_pShmem_Task_te->Rdata[iIdx][iIdx2]);
            //fprintf(fp_SensRdat, "\n");
        }
    	fprintf(fp_SensRdat,"\n");
    }

    fclose(fp_SensRdat);
    VERBOSE_MESSAGE("ArcSensor Real Data test file write Done!: '%s'\n",
                     szArcSensRDataTestFileName);

    return RESULT_OK;
}


FILE*   g_fp1_ShmAct  = 0;
FILE*   g_fp2_ShmTrg  = 0;
FILE*   g_fp3_EcatAct = 0;
FILE*   g_fp4_EcatTrg = 0;

#define SHM_JOINT_ACTUAL_FILENAME   "ShmAcutal.dat"
#define SHM_JOINT_TARGET_FILENAME   "ShmTarget.dat"
#define ECAT_JOINT_ACTUAL_FILENAME  "EcatAcutal.dat"
#define ECAT_JOINT_TARGET_FILENAME  "EcatTarget.dat"

///////////////////////////////////////
//
//  Function: SVC_PositionDataFileOpen()
//      - Service Name: SC_POS_FILE_WRITE

int SVC_PositionDataFileOpen(void)
{
    char szShmJointActualFileName[PATH_NAME_BUFFER_SIZE] = "";
    char szShmJointTargetFileName[PATH_NAME_BUFFER_SIZE] = "";
    char szEcatJointActualFileName[PATH_NAME_BUFFER_SIZE] = "";
    char szEcatJointTargetFileName[PATH_NAME_BUFFER_SIZE] = "";
    char szWorkDir[PATH_NAME_BUFFER_SIZE] = "./";

#if defined(_MSC_VER)
    errno_t errno;
#endif

    if(!g_fp1_ShmAct)
    {
        memcpy(szShmJointActualFileName,  szWorkDir, strlen(szWorkDir)+1);
        CRT_strcat(szShmJointActualFileName,  PATH_NAME_BUFFER_SIZE, SHM_JOINT_ACTUAL_FILENAME);

#if defined(__QNXNTO__)
        g_fp1_ShmAct  = fopen(szShmJointActualFileName,  "w+t");
#else
        errno=fopen_s(&g_fp1_ShmAct , szShmJointActualFileName,  "w+t");
#endif
        if (g_fp1_ShmAct == NULL)
        {
            VERBOSE_ERROR("Cannot open pos shm actual file for write : '%s'\n",
                          SHM_JOINT_ACTUAL_FILENAME);

            return SVC_ERR_POS_DATA_SAVING;
        }
    }
    if(!g_fp2_ShmTrg)
    {
        memcpy(szShmJointTargetFileName,  szWorkDir, strlen(szWorkDir)+1);
        CRT_strcat(szShmJointTargetFileName,  PATH_NAME_BUFFER_SIZE, SHM_JOINT_TARGET_FILENAME);

#if defined(__QNXNTO__)
        g_fp2_ShmTrg  = fopen(szShmJointTargetFileName,  "w+t");
#else
        errno=fopen_s(&g_fp2_ShmTrg , szShmJointTargetFileName,  "w+t");
#endif
        if (g_fp2_ShmTrg == NULL)
        {
            VERBOSE_ERROR("Cannot open pos shm target file for write : '%s'\n",
                          SHM_JOINT_TARGET_FILENAME);

            return SVC_ERR_POS_DATA_SAVING;
        }
    }
    if(!g_fp3_EcatAct)
    {
        memcpy(szEcatJointActualFileName, szWorkDir, strlen(szWorkDir)+1);
        CRT_strcat(szEcatJointActualFileName, PATH_NAME_BUFFER_SIZE, ECAT_JOINT_ACTUAL_FILENAME);

#if defined(__QNXNTO__)
        g_fp3_EcatAct = fopen(szEcatJointActualFileName, "w+t");
#else
        errno=fopen_s(&g_fp3_EcatAct, szEcatJointActualFileName, "w+t");
#endif
        if (g_fp3_EcatAct == NULL)
        {
            VERBOSE_ERROR("Cannot open pos ecat actual file for write : '%s'\n",
                          ECAT_JOINT_ACTUAL_FILENAME);

            return SVC_ERR_POS_DATA_SAVING;
        }
    }
    if(!g_fp4_EcatTrg)
    {
        memcpy(szEcatJointTargetFileName, szWorkDir, strlen(szWorkDir)+1);
        CRT_strcat(szEcatJointTargetFileName, PATH_NAME_BUFFER_SIZE, ECAT_JOINT_TARGET_FILENAME);

#if defined(__QNXNTO__)
        g_fp4_EcatTrg = fopen(szEcatJointTargetFileName, "w+t");
#else
        errno=fopen_s(&g_fp4_EcatTrg, szEcatJointTargetFileName, "w+t");
#endif
        if (g_fp4_EcatTrg == NULL)
        {
            VERBOSE_ERROR("Cannot open pos ecat target file for write : '%s'\n",
                          ECAT_JOINT_TARGET_FILENAME);

            return SVC_ERR_POS_DATA_SAVING;
        }
    }

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_PositionDataFileClose()
//      - Service Name: SC_POS_FILE_WRITE

int SVC_PositionDataFileClose(void)
{
    if(g_fp1_ShmAct != NULL)
    {
        fclose(g_fp1_ShmAct);
    }
    if(g_fp2_ShmTrg != NULL)
    {
        fclose(g_fp2_ShmTrg);
    }
    if(g_fp3_EcatAct != NULL)
    {
        fclose(g_fp3_EcatAct);
    }
    if(g_fp4_EcatTrg != NULL)
    {
        fclose(g_fp4_EcatTrg);
    }
    
    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SVC_SavePositionDataToFile()
//      - Service Name: SC_POS_FILE_WRITE

int SVC_SavePositionDataToFile(void)
{
    if(g_fp1_ShmAct != NULL)
    {
        fprintf(g_fp1_ShmAct, "%.20f %.20f %.20f %.20f %.20f %.20f\n",                 
                g_pShmem_sc->inputstate.dbActPos[0], g_pShmem_sc->inputstate.dbActPos[1],
                g_pShmem_sc->inputstate.dbActPos[2], g_pShmem_sc->inputstate.dbActPos[3],
                g_pShmem_sc->inputstate.dbActPos[4], g_pShmem_sc->inputstate.dbActPos[5]);
    }

    if(g_fp2_ShmTrg != NULL)
    {
        fprintf(g_fp2_ShmTrg, "%.20f %.20f %.20f %.20f %.20f %.20f\n",                 
                g_pShmem_sc->outputcmd.dbTrgPos[0], g_pShmem_sc->outputcmd.dbTrgPos[1],
                g_pShmem_sc->outputcmd.dbTrgPos[2], g_pShmem_sc->outputcmd.dbTrgPos[3],
                g_pShmem_sc->outputcmd.dbTrgPos[4], g_pShmem_sc->outputcmd.dbTrgPos[5]);
    }

    if(g_fp3_EcatAct != NULL)
    {
        fprintf(g_fp3_EcatAct, "%.20f %.20f %.20f %.20f %.20f %.20f\n",
                g_dbAct_Pos[0], g_dbAct_Pos[1], g_dbAct_Pos[2], g_dbAct_Pos[3], g_dbAct_Pos[4], g_dbAct_Pos[5]);
    }

    if(g_fp4_EcatTrg != NULL)
    {
        fprintf(g_fp4_EcatTrg, "%.20f %.20f %.20f %.20f %.20f %.20f\n",
                g_dbTrg_Pos[0], g_dbTrg_Pos[1], g_dbTrg_Pos[2], g_dbTrg_Pos[3], g_dbTrg_Pos[4], g_dbTrg_Pos[5]);
    }

    return RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
//
// SERV_PosFileWrite()
//
int SERV_PosFileWrite(void)
{
    if(g_fPosFileWrite == ON)
    {
        if(g_pShmem_Status_te != NULL)
        {
            if(g_pShmem_Status_te->run_mode != RUNMODE_NONE)
            {
                SVC_PositionDataFileOpen();
            }

            if(g_pShmem_Status_te->run_mode != RUNMODE_NONE)
            {
                SVC_SavePositionDataToFile();
            }

            if(g_pShmem_Status_te->run_mode == RUNMODE_NONE)
            {
                SVC_PositionDataFileClose();
            }
        }
    }
    else if(g_fPosFileWrite == OFF)
    {
        SVC_PositionDataFileClose();

        return RESULT_OK;
    }

    return RESULT_OK;
}

#if defined(_MSC_VER)
FILE*   p1_ActPos  = 0;

#define JOINT_ACTUAL_FILENAME   "CurrPos.Win.dat"

///////////////////////////////////////
//
//  Function: SYS_CurrPositionDataFileOpen()
//      - Service Name: 

int SYS_CurrPositionDataFileOpen(int nOpt)
{
    char szJointActualFileName[PATH_NAME_BUFFER_SIZE] = "";
    char szWorkDir[PATH_NAME_BUFFER_SIZE] = "./";
    int  iAxis;

    errno_t errno;

    memcpy(szJointActualFileName,  szWorkDir, strlen(szWorkDir)+1);
    CRT_strcat(szJointActualFileName,  PATH_NAME_BUFFER_SIZE, JOINT_ACTUAL_FILENAME);

    if(nOpt == OPT_READ)
    {
        errno=fopen_s(&p1_ActPos , szJointActualFileName,  "rt");
    }
    else if(nOpt == OPT_WRITE)
    {
        errno=fopen_s(&p1_ActPos , szJointActualFileName,  "wt");
    }

    if (p1_ActPos == NULL)
    {
        VERBOSE_ERROR("Cannot open pos actual pos file for write : '%s'\n",
                      JOINT_ACTUAL_FILENAME);

        for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
        {
            g_nAct_Pulse[iAxis] = 0;
            
            FUNC_SyncActualPosToTargetPos();
            g_nTrg_Pulse[iAxis] = g_nAct_Pulse[iAxis];
            
            FUNC_ConvertPosToPulse(iAxis, TARGET_POS_IDX);
        }

        return SVC_ERR_POS_DATA_SAVING;
    }
    
    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SYS_CurrPositionDataFileLoad()
//      - Service Name: 

int SYS_CurrPositionDataFileLoad(void)
{
    int iAxis;

    if(p1_ActPos != NULL)
    {
        if (g_pShmem_sc != NULL)
        {
            for (iAxis = 0; iAxis < g_nAxisCount; iAxis++)
            {
                fscanf_s(p1_ActPos, "%d", &g_nAct_Pulse[iAxis], sizeof(int));
                
                FUNC_SyncActualPosToTargetPos();
                g_nTrg_Pulse[iAxis] = g_nAct_Pulse[iAxis];
                
                FUNC_ConvertPosToPulse(iAxis, TARGET_POS_IDX);
            }

            VERBOSE_VERBOSE("Loaded CurrPos: %.2lf   %.2lf   %.2lf   %.2lf   %.2lf   %.2lf\n",
                            g_dbAct_Pos[0] * (180/M_PI),
                            g_dbAct_Pos[1] * (180/M_PI),
                            g_dbAct_Pos[2] * (180/M_PI), 
                            g_dbAct_Pos[3] * (180/M_PI),
                            g_dbAct_Pos[4] * (180/M_PI),
                            g_dbAct_Pos[5] * (180/M_PI));
        }
    }

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SYS_CurrPositionDataFileSave()
//      - Service Name: 

int SYS_CurrPositionDataFileSave(void)
{
    if(p1_ActPos != NULL)
    {
#if 0
        fprintf(p1_ActPos, "%.5lf %.5lf %.5lf %.5lf %.5lf %.5lf",                 
                g_dbAct_Pos[0], g_dbAct_Pos[1], g_dbAct_Pos[2], 
                g_dbAct_Pos[3], g_dbAct_Pos[4], g_dbAct_Pos[5]);
#endif
        fprintf(p1_ActPos, "%d %d %d %d %d %d",                 
                g_nAct_Pulse[0],
                g_nAct_Pulse[1],
                g_nAct_Pulse[2], 
                g_nAct_Pulse[3],
                g_nAct_Pulse[4],
                g_nAct_Pulse[5]);
        
        VERBOSE_VERBOSE("Saved CurrPos: %.2lf   %.2lf   %.2lf   %.2lf   %.2lf   %.2lf\n",
                            g_dbAct_Pos[0] * (180/M_PI),
                            g_dbAct_Pos[1] * (180/M_PI),
                            g_dbAct_Pos[2] * (180/M_PI),
                            g_dbAct_Pos[3] * (180/M_PI),
                            g_dbAct_Pos[4] * (180/M_PI),
                            g_dbAct_Pos[5] * (180/M_PI));
        
    }

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: SYS_CurrPositionDataFileClose()
//      - Service Name: 

int SYS_CurrPositionDataFileClose(void)
{
    if(p1_ActPos != NULL)
    {
        fclose(p1_ActPos);
        p1_ActPos  = NULL;
    }
        
    return RESULT_OK;
}

#endif