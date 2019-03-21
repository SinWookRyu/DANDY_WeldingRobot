#ifndef __SERVICE_H__
#define __SERVICE_H__

#include "servocon_main.h"

/* Motor Control Service */
int SERV_ABSEncoderReset(int nAxis);
int SERV_ABSEncoderResetOnly(int nAxis);
int SERV_SetZeroPosition(int nAxis);
int SERV_SetHomeOffsetZeroValue(int nAxis);
int SERV_EStop(int nValue);
int SERV_ServoOffStop(void);
int SERV_JobStopProcess(void);
int SERV_GetPosition(void);
int SERV_GetServoAlarmCode(void);

int SERV_BrakeReleaseCmd(int nAxis, int nValue);
int SERV_ServoOnCmd(int nValue);
int SERV_SetPosition(int nValue);
int SERV_ClearServoAlarm(void);


/* I/O Control Service */
int SERV_EcatDigitalOut(int nSlave, int nIndex, BOOL bValue);
int SERV_Scan_Welder_IO(int nOpt);
int SERV_Scan_Servo_IO(int nOpt);

int SERV_WeldVolt_out(double dbValue);
int SERV_WeldCurr_out(double dbValue);
int SERV_ArcOn_out(int nValue);
int SERV_GasOn_out(int nValue);
int SERV_InchingPos_out(int nValue);
int SERV_InchingNeg_out(int nValue);
int SERV_TouchStart_out(int nValue);
int SERV_TouchReady_out(int nValue);

int SERV_LampControllerReady_out(int nValue);
int SERV_LampUnderOperating_out(int nValue);
int SERV_LampServoOn_out(int nValue);
int SERV_LampEtherCATRun_out(int nValue);
int SERV_LampError_out(int nValue);


/* Network Service */
int SERV_GetNetworkState(void);
int SERV_GetServoState(void);
int SERV_RestartMaster(void);


/* System Service */
int  SERV_SetVersion(SC_MSG* pMsg);
int  SERV_Scan_System_State(int nOpt);
void DSP_SetVerbose(void);
int  SERV_ArcSensorDataFileSave(int nOpt);
int  SERV_ArcSensorRDataFileLoad(void);

int SYS_CurrPositionDataFileOpen(int nOpt);
int SYS_CurrPositionDataFileLoad(void);
int SYS_CurrPositionDataFileSave(void);
int SYS_CurrPositionDataFileClose(void);


#endif  // end of __SERVICE_H__