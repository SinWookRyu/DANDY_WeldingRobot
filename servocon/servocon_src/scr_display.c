/////////////////////////////////////////////////////////////////////////////
//
//  scr_display.c: Set Display Env. & VGA Display Thread
//                                            2013.11.11  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES

#include "servocon_main.h"
#include <math.h>

///////////////////////////////////////
//Global_variable

/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// DSP_SetVerbose()
//
void DSP_SetVerbose(void)
{
    switch(g_chPrintLev)
    {    
    case 'm':
        // -message argument
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ALERT,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_NOTIFY,  TRUE);
        break; 

    case 'w':
        // -warning argument
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ALERT,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_NOTIFY,  TRUE);
        break; 

    case 'e':
        // -error argument
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, FALSE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ALERT,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_NOTIFY,  TRUE);
        break; 

    case 'v':        
        // -verbose argument
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ALERT,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_NOTIFY,  TRUE);
        break; 

    default:        
        // -verbose argument
        VERBOSE_EnableOutput(VERBOSE_TYPE_VERBOSE, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_MESSAGE, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ERROR,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_WARNING, TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_ALERT,   TRUE);
        VERBOSE_EnableOutput(VERBOSE_TYPE_NOTIFY,  TRUE);
        break; 
    }

    // set vervose color
    VERBOSE_SetColor(VERBOSE_TYPE_VERBOSE, 
                     ECHO_COLOR_WHITE, 
                     ECHO_COLOR_BLACK);
}

/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: DSP_VGADisplyThreadRoutine()
//
#define DEF_FirstLine  8

#define POS_DATA_LEN  9
#define PORT_DATA_LEN 2

THREAD_ENTRY_TYPE DSP_VGADisplyThreadRoutine(void* pParam)
{
    static int s_nDSPPosX1, s_nDSPPosX2, s_nDSPPosX3;
    static int s_nCnt;
    static int s_nSlave, s_nPort;
    static int nFirstLine;
    static int nSecondLine;
    static int nThirdLine;
    static int nFourthLine;
    static int nFifthLine;
    static int nSixthLine;
    static int nSeventhLine;
    static int nEighthLine;
    static int nNinthLine;
    static int nTenthLine;
    static int nEleventhLine;
    static int nTwelvethLine;

    if(g_Arg.nVGADispLine == -1)
    {
        nFirstLine = DEF_FirstLine;
    }
    else
    {
        nFirstLine = g_Arg.nVGADispLine;
    }

    nSecondLine  = nFirstLine   + 1;
    nThirdLine   = nSecondLine  + 1;
    nFourthLine  = nThirdLine   + 1;
    nFifthLine   = nFourthLine  + 1;
    nSixthLine   = nFifthLine   + 1;
    nSeventhLine = nSixthLine   + 1;
    nEighthLine  = nSeventhLine + 1;
    nNinthLine   = nEighthLine  + 1;
    nTenthLine   = nNinthLine   + 1;
    nEleventhLine= nTenthLine   + 1;
    nTwelvethLine= nEleventhLine+ 1;

    while(g_fExit == FALSE && g_Arg.bNoVGA == FALSE)
	{
        s_nDSPPosX2 = 0;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_YELLOW, VGA_COLOR_RED));
        if(g_pShmem_sc != NULL)
        {
            s_nDSPPosX2 = 0;
            VGA_printf(s_nDSPPosX2, nFirstLine, "SVON: %d", g_pShmem_sc->outputstate.fServoOnOutState);
            s_nDSPPosX2 = 9;
            VGA_printf(s_nDSPPosX2, nFirstLine, "EST: %d", g_pShmem_sc->sysstate.fEStopState);
            s_nDSPPosX2 = 17;
            VGA_printf(s_nDSPPosX2, nFirstLine, "ERR: %d", g_pShmem_sc->sysstate.fErrorState);
        }
        
        s_nDSPPosX2 = 25;
        if(g_pShmem_sc != NULL)
        {
            VGA_printf(s_nDSPPosX2, nFirstLine, "Code:%6x", g_pShmem_sc->sysstate.nErrorCode);
        }
        
        s_nDSPPosX2 = 38;
        VGA_printf(s_nDSPPosX2, nFirstLine, "Tr:%7dns", g_nTimerNanoRes);

        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_YELLOW, VGA_COLOR_BLUE));
#if 1
        s_nDSPPosX2 = 52;
        VGA_printf(s_nDSPPosX2, nFirstLine, "N:%3d", g_nArcSensNodeIdx);

        s_nDSPPosX2 = 59;
        VGA_printf(s_nDSPPosX2, nFirstLine, "V:%5.1f", g_dbWeldVoltInVal[g_nArcSensNodeIdx] * 100.0);

        s_nDSPPosX2 = 68;
        VGA_printf(s_nDSPPosX2, nFirstLine, "C:%6.1f", g_dbWeldCurrInVal[g_nArcSensNodeIdx] * 100.0);
#endif

        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTCYAN, VGA_COLOR_BLACK));
        s_nDSPPosX1 = 0;

        /* Display Slave State(Servo & I/O) */
        VGA_printf(s_nDSPPosX1, nSecondLine,  "Cur_Pos  :");
        //VGA_printf(s_nDSPPosX1, nFirstLine, "Trg_Pos  :");
        VGA_printf(s_nDSPPosX1, nThirdLine,   "Cur_Pulse:");
        VGA_printf(s_nDSPPosX1, nFourthLine,  "Trg_Pulse:");
        VGA_printf(s_nDSPPosX1, nFifthLine,   "Status   :");
        VGA_printf(s_nDSPPosX1, nSixthLine,   "AlarmCode:");

        s_nDSPPosX1 = 12;

        for(s_nCnt = 0; s_nCnt < g_nAxisCount; s_nCnt++)
        {
            VGA_printf(s_nDSPPosX1, nSecondLine,
                       "%9.1f", g_dbAct_Pos[s_nCnt] * (180/M_PI));
            VGA_printf(s_nDSPPosX1, nThirdLine,
                       "%9d", g_nAct_Pulse[s_nCnt]);
            VGA_printf(s_nDSPPosX1, nFourthLine,
                       "%9d", g_nTrg_Pulse[s_nCnt]);
                       //"%8.1f", g_rgdbHwLimit[s_nCnt][LIMIT_POS] * (180/M_PI));
            VGA_printf(s_nDSPPosX1, nFifthLine,
                       "%9x", g_nReadStatusValue[s_nCnt]);
            VGA_printf(s_nDSPPosX1, nSixthLine,
                       "%9x", g_nErrCodeServo[s_nCnt]);
            //VGA_printf(s_nDSPPosX1, nSixthLine,
            //           "%9.1lf", g_dbActTorque[s_nCnt]);

            s_nDSPPosX1 = s_nDSPPosX1 + POS_DATA_LEN;

            if(s_nCnt == g_nAxisCount - 1)
            {
                s_nDSPPosX1 = 10;
            }
        }
        
        s_nDSPPosX2 = 0;
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_BLACK, VGA_COLOR_WHITE));
        VGA_printf(s_nDSPPosX2, nSeventhLine,
                       "                                                                            ");
        VGA_printf(s_nDSPPosX2, nEighthLine,
                       "                                                                            ");
        VGA_printf(s_nDSPPosX2, nNinthLine,
                       "                                                                            ");
        VGA_printf(s_nDSPPosX2, nSeventhLine, "D-In:");
        VGA_printf(s_nDSPPosX2, nEighthLine,  "D-Out:");
        VGA_printf(s_nDSPPosX2, nNinthLine,   "A-In:");
        s_nDSPPosX2 = 6;

#if 0
        if(g_pShmem_sc != NULL)
        {
            for(s_nPort = 0; s_nPort < ROBOT_DI_PORT_COUNT; s_nPort++)
            {
                VGA_printf(s_nDSPPosX2, nSixthLine, "%2d", g_pShmem_sc->inputstate.nDinPortVal[s_nPort]);
                s_nDSPPosX2 = s_nDSPPosX2 + PORT_DATA_LEN;
                if(s_nPort == 9 || s_nPort == 19 || s_nPort == 29)
                    s_nDSPPosX2 = s_nDSPPosX2 + PORT_DATA_LEN;
            }
        }
        s_nDSPPosX2 = 6;
#endif
        for(s_nSlave = 0; s_nSlave < ROBOT_DI_SLAVE_COUNT; s_nSlave++)
        {
            for(s_nPort = 0; s_nPort < SLAVE_DI_PORT_COUNT; s_nPort++)
            {
                VGA_printf(s_nDSPPosX2, nSeventhLine, "%2d", g_DinPortVal[s_nSlave][s_nPort]);
                s_nDSPPosX2 = s_nDSPPosX2 + PORT_DATA_LEN;
            }
            s_nDSPPosX2 = s_nDSPPosX2 + PORT_DATA_LEN;
        }
        s_nDSPPosX2 = 6;

        for(s_nSlave = 0; s_nSlave < ROBOT_DO_SLAVE_COUNT; s_nSlave++)
        {
            for(s_nPort = 0; s_nPort < SLAVE_DO_PORT_COUNT; s_nPort++)
            {
                VGA_printf(s_nDSPPosX2, nEighthLine, "%2d", g_DoutPortVal[s_nSlave][s_nPort]);
                s_nDSPPosX2 = s_nDSPPosX2 + PORT_DATA_LEN;
            }
            s_nDSPPosX2 = s_nDSPPosX2 + PORT_DATA_LEN;
        }
        s_nDSPPosX2 = 6;
        
        for(s_nSlave = 0; s_nSlave < ROBOT_AI_SLAVE_COUNT; s_nSlave++)
        {
            for(s_nPort = 0; s_nPort < SLAVE_AI_PORT_COUNT; s_nPort++)
            {
                VGA_printf(s_nDSPPosX2, nNinthLine, "%.2f", g_AinPortVal[s_nSlave][s_nPort]);
                s_nDSPPosX2 = s_nDSPPosX2 + (3 * PORT_DATA_LEN);
            }
        }

        s_nDSPPosX2 = 18;
        VGA_printf(s_nDSPPosX2, nNinthLine,  "A-Out:");

        s_nDSPPosX2 = 24;
        for(s_nSlave = 0; s_nSlave < ROBOT_AO_SLAVE_COUNT; s_nSlave++)
        {
            for(s_nPort = 0; s_nPort < SLAVE_AO_PORT_COUNT; s_nPort++)
            {
                VGA_printf(s_nDSPPosX2, nNinthLine, "%.2f", g_AoutPortVal[s_nSlave][s_nPort]);
                s_nDSPPosX2 = s_nDSPPosX2 + (3 * PORT_DATA_LEN);
            }
        }

        if(g_pShmem_sc != NULL)
        {
            s_nDSPPosX2 = 37;
            VGA_printf(s_nDSPPosX2, nNinthLine, "W-In: %-4.2lf  %-3.0lf",
                       g_pShmem_sc->inputstate.dbWeldVoltInVal,
                       g_pShmem_sc->inputstate.dbWeldCurrInVal);

            s_nDSPPosX2 = 56;
            VGA_printf(s_nDSPPosX2, nNinthLine, "W-Out: %-4.2lf  %-3.0lf",
                       g_pShmem_sc->outputstate.dbWeldVoltOutVal,
                       g_pShmem_sc->outputstate.dbWeldCurrOutVal);
        }
#if 0
        VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_YELLOW, VGA_COLOR_RED));
        if(g_pShmem_sc != NULL)
        {
            s_nDSPPosX2 = 21;
            VGA_printf(s_nDSPPosX2, nSeventhLine, "SVON: %d", g_pShmem_sc->outputstate.fServoOnOutState);
            s_nDSPPosX2 = 30;
            VGA_printf(s_nDSPPosX2, nSeventhLine, "EST: %d", g_pShmem_sc->sysstate.fEStopState);
            s_nDSPPosX2 = 38;
            VGA_printf(s_nDSPPosX2, nSeventhLine, "ERR: %d", g_pShmem_sc->sysstate.fErrorState);
        }
        
        s_nDSPPosX2 = 46;
        if(g_pShmem_sc != NULL)
        {
            VGA_printf(s_nDSPPosX2, nSeventhLine, "Code:%6x", g_pShmem_sc->sysstate.nErrorCode);
        }
        
        s_nDSPPosX2 = 58;
        VGA_printf(s_nDSPPosX2, nSeventhLine, "Tr:%7dns", g_nTimerNanoRes);
#endif
        /* Display EtherCAT Statistics */
        if(g_Arg.bEcatStatDisplay == TRUE)
        {
            VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_LIGHTGREEN, VGA_COLOR_BROWN));
            s_nDSPPosX3 = 0;
            VGA_printf(s_nDSPPosX3, nTenthLine,
                       "                                                                            ");
            VGA_printf(s_nDSPPosX3, nEleventhLine,
                       "                                                                            ");
            VGA_printf(s_nDSPPosX3, nTwelvethLine,
                       "                                                                            ");

            VGA_printf(s_nDSPPosX3, nTenthLine, "EcatErMsg:");
            s_nDSPPosX3 = 10;
            VGA_printf(s_nDSPPosX3, nTenthLine, g_szEcatErrorDescription);

            s_nDSPPosX3 = 0;
            VGA_printf(s_nDSPPosX3, nEleventhLine, "Snd/Rec/Par Err:%4d/%4d/%4d  WC:%8d",
                       (int) g_EcatStatistics.RTStat.dwSendErrors,
                       (int) g_EcatStatistics.RTStat.dwReceiveErrors,
                       (int) g_EcatStatistics.RTStat.dwParseErrors,
                       (int) g_EcatStatistics.RTStat.dwWrongWC);
            s_nDSPPosX3 = 45;
            VGA_printf(s_nDSPPosX3, nEleventhLine, "Code:%4x", g_nErrCodeEcat);
            s_nDSPPosX3 = 56;
            VGA_printf(s_nDSPPosX3, nEleventhLine, "E-Msg:%4x/%3x(Ax:%d)",
                       (int) g_wEmergencyErrStateCode[g_nErrAxis],
                       g_nEmergencyCodeServo[g_nErrAxis], g_nErrAxis);

            s_nDSPPosX3 = 0;
            VGA_printf(s_nDSPPosX3, nTwelvethLine, "Cy/CyJ: %-8d/%-5d",
                       (int) g_EcatStatistics.RTStat.dwAvgCycleTime, (int) g_EcatStatistics.RTStat.dwAvgCycleJitter);
            s_nDSPPosX3 = 20;
            VGA_printf(s_nDSPPosX3, nTwelvethLine, "SubCy/SubCyJ: %-9d/%-5d",
                       (int) g_EcatStatistics.RTStat.dwAvgSubCycleTime, (int) g_EcatStatistics.RTStat.dwAvgSubCycleJitter);
            s_nDSPPosX3 = 47;
            VGA_printf(s_nDSPPosX3, nTwelvethLine, "SlaveResp: %4d",
                       (int) g_EcatStatistics.SysStat.dwRespondTimeUs);
        }
        
        THREAD_Sleep(100);
        //DANDY_SLEEP(120);
        //THREAD_Sleep(120);
    }

    g_fDisplayThreadExitState = TRUE;

    return 0;
}
