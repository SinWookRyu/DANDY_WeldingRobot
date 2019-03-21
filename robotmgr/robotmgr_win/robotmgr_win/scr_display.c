/////////////////////////////////////////////////////////////////////////////
//
//  scr_display.c: Screen Display
//                                            2013.06.18  Ryu SinWook

#include <stdlib.h>
#include <string.h>         // for strcpy(), strlen(), memset
#include <stdio.h>

///////////////////////////////////////
#include "dandy_echo.h"     // for VERBOSE
#include "dandy_thread.h"   // for service request thread
#include "dandy_msgpass.h"  // for message passing (service request)
#include "dandy_ansi.h"     // for key input value

#include "dandy_platform.h"
#include "dandy_debug.h"

#include "ipc_robotmgr.h"
#include "ipc_taskexec.h"
#include "ipc_servocon.h"

#include "robotmgr_main.h"
#include "sys_conf.h"
#include "ascii_def.h"
#include "error_def.h"
#include "CRT.h"

///////////////////////////////////////
#define VERBOSE_NAME    "RM_VERBOSE_"
#define ___DEBUG

///////////////////////////////////////
//Global_variables

int g_coidTE;
int g_coidSC;
int g_fSysModeStatusThRun = RUN;
int g_fSysAliveThRun = RUN;
int g_fVGA_DSPThRun = RUN;
int g_fConsoleExitAct = FALSE;
int g_fInitRet[INIT_STEP_LEN] = {-1, -1, -1, -1, -1};
char g_szSystemMode[SYSMODE_NAME_LEN];
char g_szExecStat[EXECSTATE_NAME_LEN];
char g_szWorkType[WORKTYPE_NAME_LEN];
char g_szRMState[SIBLINGSTATE_NAME_LEN];
char g_szTEState[SIBLINGSTATE_NAME_LEN];
char g_szSCState[SIBLINGSTATE_NAME_LEN];

int g_retOpenShmemTE;
int g_retOpenShmemSC;

int g_retInitRM = FALSE;
int g_nSystemMode = MODE_STAT_ENTRY;

RMGR_PACKET                     RM_packet;
RMGR_REPLY_PACKET               RM_reply_packet;
MSG_INFO                        info_msg;
TE_MSG                          TE_msg;
SC_MSG                          SC_msg;
TE_REPLY                        TE_reply;
SC_REPLY                        SC_reply;

ARGUMENT_OPTION                 g_Arg;

SHM_TE_TEST*                    g_pShmemTE;
SHM_RM_SYSSTATUS*               g_pShm_SysStatus;
SHM_SC_SYSTEM*                  g_pShmemSC;

ERROR_CODE_STACK                g_ErrCodeStack;

///////////////////////////////////////
// Functions

int MSG_ConnectChannelServer(int nCoid);
int MSG_CloseConnection(int nCoid, int nCoidAlive);
int MSG_CreateRMChannel(void);
int SVC_InitRMService(void);
int SHM_OpenSharedMemory(void);
int PARAM_LoadSystemParameter(void);
int SHM_CreateSharedMemory(void);
int MAIN_Initialize(void);
int SVC_InitTEService(void);
int SVC_InitSCService(void);
int SVC_ExitTEService(void);
int SVC_ExitSCService(void);
int SVC_ExecService(int fKeyIn);

static int _loc_MAIN_Finalize(void);


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_DSP_VGA_SystemInform(int x, int y,
//                                      char targ, int fcolor, int bcolor)
//      - Display System Information

static void _loc_DSP_VGA_SystemInform(int s_nPosX, int s_nPosY,
                                      char *s_szPrintTarget,
                                      int s_nForeColor, int s_nBackColor)
{
    // config vga display color
    VGA_SetAttr(VGA_MAKE_ATTR(s_nForeColor, s_nBackColor));

    VGA_printf(s_nPosX, s_nPosY, s_szPrintTarget); 
}


/////////////////////////////////////////////////////////////////////////////
//
//  Thread Routine: _loc_DSP_VGADisplyThread()
//

static THREAD_ENTRY_TYPE _loc_DSP_VGADisplyThread(void* pParam)
{
    static int s_nDSPPosX;
    static int s_nCnt;

    #define FirstLine     0
    #define SecondLine    1

    while(g_fVGA_DSPThRun == RUN)
	{
        // first line display define: 1. system mode 2. execution state
        //                            3. work type   4. error state
        s_nDSPPosX = 0;
        _loc_DSP_VGA_SystemInform(s_nDSPPosX, FirstLine, g_szSystemMode,
                                  VGA_COLOR_LIGHTCYAN, VGA_COLOR_MAGENTA);

        s_nDSPPosX = s_nDSPPosX + SYSMODE_NAME_LEN;
        _loc_DSP_VGA_SystemInform(s_nDSPPosX, FirstLine, g_szExecStat,
                                  VGA_COLOR_WHITE, VGA_COLOR_DARKGRAY);

        s_nDSPPosX = s_nDSPPosX + EXECSTATE_NAME_LEN;
        _loc_DSP_VGA_SystemInform(s_nDSPPosX, FirstLine, g_szWorkType,
                                  VGA_COLOR_WHITE, VGA_COLOR_DARKGRAY);
        
        s_nDSPPosX = s_nDSPPosX + WORKTYPE_NAME_LEN;  
        if(g_hShm_SysStatus != 0 && g_hShm_SysStatus != -1)
        {
            if(g_pShm_SysStatus->fErrorState == TRUE)
            {
                SVC_ParceErrCodeToErrContent(g_pShm_SysStatus->nErrCode);
                                
                // process for flickering
                if(s_nCnt <= 15)
                {
                    _loc_DSP_VGA_SystemInform(s_nDSPPosX, FirstLine,
                                              g_szErrContent,
                                              VGA_COLOR_LIGHTRED,
                                              VGA_COLOR_RED);
                    s_nCnt++;
                }
                else if(s_nCnt >= 15 && s_nCnt <= 25)
                {
                    _loc_DSP_VGA_SystemInform(s_nDSPPosX, FirstLine,
                                              "                         ",
                                              VGA_COLOR_LIGHTRED,
                                              VGA_COLOR_RED);
                    s_nCnt++;
                }
                else if(s_nCnt >= 25)
                {
                    s_nCnt = 0;
                }
            }
            else if(g_pShm_SysStatus->fErrorState == FALSE)
            {
                s_nCnt = 0;
                _loc_DSP_VGA_SystemInform(s_nDSPPosX, FirstLine,
                                          " Not Error State         ",
                                          VGA_COLOR_LIGHTRED,
                                          VGA_COLOR_RED);
            }
        }

        // second line display define: 1. service name 2. TE state 3. SC state
        s_nDSPPosX = 0;
        _loc_DSP_VGA_SystemInform(s_nDSPPosX, SecondLine, g_szServContent,
                                  VGA_COLOR_BLUE, VGA_COLOR_LIGHTMAGENTA);

        s_nDSPPosX = s_nDSPPosX + SERV_NAME_LEN;
        _loc_DSP_VGA_SystemInform(s_nDSPPosX, SecondLine, g_szRMState,
                                  VGA_COLOR_GREEN, VGA_COLOR_LIGHTCYAN);

        s_nDSPPosX = s_nDSPPosX + SIBLINGSTATE_NAME_LEN;
        _loc_DSP_VGA_SystemInform(s_nDSPPosX, SecondLine, g_szTEState,
                                  VGA_COLOR_GREEN, VGA_COLOR_LIGHTCYAN);

        s_nDSPPosX = s_nDSPPosX + SIBLINGSTATE_NAME_LEN;
        _loc_DSP_VGA_SystemInform(s_nDSPPosX, SecondLine, g_szSCState,
                                  VGA_COLOR_GREEN, VGA_COLOR_LIGHTCYAN);

        DANDY_SLEEP(10);
    }

    return 0;
}