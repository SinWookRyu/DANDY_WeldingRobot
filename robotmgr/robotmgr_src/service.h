#ifndef __SERVICE_H__
#define __SERVICE_H__

#include "robotmgr_main.h"

/* Control Service */
int SVC_Error_Reset(void);
int SVC_TimerTest(void);
int SVC_RCON_Home_Move(int nHomeIndex);
int SVC_RCON_WireCut_JobCall(void);
int SVC_RCON_SetVoltageRealTimeOffset(int nOpt);
int SVC_RCON_SetCurrentRealTimeOffset(int nOpt);

void SVC_ShutdownSystem(int nOpt);

/* System Information Service */
int SVC_ShowSystemStateDisplay(void);
int SVC_ShowErrorHistoryDisplay(void);
int SVC_SetSystemInfoPacketDefine(int nValue);
int SVC_Clear_ErrorHistory(void);
int SVC_ShowSystemInfoDisplay(int nValue);
int SVC_SetSystemStatePacketDefine(void);
int SVC_SetErrorHistoryPacketDefine(void);

int SVC_SendVersionInform(void);
int SVC_SystemMonDataSendToTP(void);
int SVC_JobProgLineDisplay(int nDisplayLineIndex);

/* Parameter Service */
int SVC_SetHomePositionValue(int nHomeIndex);
int SVC_SetUserCoordinateValue(int nUserCoordIndex);
int SVC_SetToolCenterdPointValue(void);
int SVC_SetCartOffsetValue(void);
int SVC_SetWordCoordinateOffsetValue(void);
int SVC_SetWeldTuneInputValue(int nIdx);
int SVC_SetWeldTuneOutputValue(int nIdx); 
int SVC_SendWeldTuneInputParameterValue(int nIdx);
int SVC_SendWeldTuneOutputParameterValue(int nIdx);
int SVC_ApplyWeldTuneInputParameterValue(int nIdx);
int SVC_ApplyWeldTuneOutputParameterValue(int nIdx);
int SVC_SetRestartParameterValue(void);
int SVC_SetArcSensorParamValue(void);
int SVC_SendHomePositionValue(int nHomeIndex);
int SVC_SendUserCoordinateValue(int nUserCoordIndex);
int SVC_SendUserParameterValue(int nOption);
int SVC_SetGapCondition(void);
int SVC_SendConstJobVarInfo(int nVarType);
int SVC_SendWeldJobVarInfo(int nVarType);
int SVC_EditConstJobVar(int nVarType);
int SVC_SetSkipCondition(int nOption);
int SVC_ShowSkipCondition(void);
int SVC_SetGapSkipCondition(void);
int SVC_ShowGapSkipCondition(void);
int SVC_ConstVarFileHandle(int nOpt);
int SVC_ModifyBvarForGapSkipCond(int nGapSkipBvarIdx, int nGapSkipCond);
int FUNC_GetBVarValue(int nVarIndex);
int SVC_InitConstJobVar(int nVarType);
int SVC_SetWeldMeasureParamValue(int nOpt);

int SVC_SaveUserParamConfigToFile(int nRobot);
int SVC_ApplyParamToSibling(void);
int FUNC_CheckTuneOutputValueValidation(int nOpt, double dbA, double dbB, double dbC);
int	FUNC_GetWelderCoeff(double* dbWelderCoeff, BYTE nWelderID, int nOpt);
int SVC_CalculateAndSaveWeldCoefficient(int nIdx);

/* Job Service */
int SVC_LoadJobData(const char* pszCompileTargetFileName, int nOpt);
int SVC_DumpJobShmData(int nOpt);
int SVC_ExecuteJobProgAuto(int nLineIdx);
int SVC_ExecuteJobProgDry(int nLineIdx);
int SVC_ExecuteJobProgStep(int nLineIdx);
int SVC_StopJobProg(void);
int SVC_JobCmdJump(void);
int SVC_JobCmdCall(void);
int SVC_JobCmdReturn(void);
int SVC_JobCmdContinue(void);
int SVC_JobFileZipUncompress(void);

int SVC_JobBinSendHeader(int nOpt);
int SVC_JobBinSendData(int nIndex);
int SVC_JobBinReceiveHeader(int nOpt);
int SVC_JobBinReceiveData(int nIndex);

int SVC_JobCompBinSendHeader(int nOpt);
int SVC_JobCompBinSendData(int nIndex);
int SVC_JobCompBinReceiveHeader(int nOpt);
int SVC_JobCompBinReceiveData(int nIndex);

int SVC_JobRestart(void);

/* Statistics Service */
int SVC_GetStatisticsData(int nOpt);
int Fn_WriteStatisticsData(int nVerboseOpt);
int Fn_ResetStatisticsData(void);
int SVC_SaveStatisticsDataToFile(int nRobot, int nVerboseOpt);

int SVC_VGADisplayStatisticsData(int nDisYPos);

#endif  // end of __SERVICE_H__