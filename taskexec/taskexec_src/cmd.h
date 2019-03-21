#ifndef CMD_H_20130913_
#define CMD_H_20130913_

#include "arg.h"

#define CWEAVE_MODE_NONE     0      // 
#define CWEAVE_MODE_VERT     1      // Vertical Weaving (start)
#define CWEAVE_MODE_DWELL_V  2      // Dwell After Vert. Weaving. Just waits btw Vert Weav & Round Weav
#define CWEAVE_MODE_ROUND    3      // Rount Weaving
#define CWEAVE_MODE_DWELL_H  4      // Dwell Before Horz. Weaving.Just waits btw Rount Weav & Horz Weav
#define CWEAVE_MODE_HORZ     5      // Horizontal Weaving (final)

#if defined(__cplusplus)
extern "C" {
#endif
   

int Cmd_Init(int index, double jnt_start[6], UNT f_dry); 
int Cmd_Update(int index);
void Cmd_Stop(int index, int f_estop);

// To Use Extra Cmd Handler Sets External Cmd Handlers. 
// Init Call must be done seperately. 
void Cmd_ExtHandlerSet(int (*CmdUpd)(void), VOD (*CmdStp)(int f_estop), char* str_cmd); 

// returns if 'cmd_code' is the linear motion cmd. 
int Cmd_IsLinMot(int i_prog);
int Cmd_IsCirMot(int i_prog);
UNT Cmd_IsWeaveCmd(int i_prog);
UNT Cmd_IsCweaveCmd(int i_prog); 

// Cmd for Move
int CmdSet_MOVJ(int index, double jnt_start[6]);
int CmdSet_MOVL(int index, double jnt_start[6]);
int CmdSet_MOVO(int index, double jnt_start[6]);
int CmdSet_MOVI(int index, double jnt_start[6]);
int CmdSet_MOVC(int index, double jnt_start[6]);
int CmdSet_HOME(int index, double jnt_start[6]);
int CmdSet_WEAV(int index, double jnt_start[6]);
int CmdSet_CWEAV(int index, double jnt_start[6]); 

int CmdUpd_Mov(void); 
int CmdUpd_WEAV(void);
int CmdUpd_CWEAV(void); 

VOD CmdStp_Mov(int f_estop); 
VOD CmdStp_WEAV(int f_fast); 
VOD CmdStp_CWEAV(int f_fast); 

// Cmd for Operation
int CmdSet_SET(int index, double jnt_start[6]);
int CmdSet_ADD(int index, double jnt_start[6]); 
int CmdSet_SUB(int index, double jnt_start[6]); 
int CmdSet_MUL(int index, double jnt_start[6]); 
int CmdSet_DIV(int index, double jnt_start[6]);
int CmdSet_MOD(int index, double jnt_start[6]); 
int CmdSet_AND(int index, double jnt_start[6]); 
int CmdSet_OR(int index, double jnt_start[6]); 
int CmdSet_XOR(int index, double jnt_start[6]); 
int CmdSet_NOT(int index, double jnt_start[6]);
int CmdSet_COMP(int index, double jnt_start[6]); 

// Cmd for Branch
int CmdSet_JUMP(int index, double jnt_start[6]); 
int CmdSet_CALL(int index, double jnt_start[6]); 
int CmdSet_RET(int index, double jnt_start[6]);
int CmdUpd_CALL_RET(void); 
#if 0
int CmdUpd_CALL(void); 
int CmdUpd_RET(void); 
#endif

// 
int Cmd_Set_GETP(int index, double jnt_start[6]); 

// Welding Cmds
int CmdSet_TOUCH(int index, double jnt_start[6]); 
int CmdUpd_TOUCH(void); 
VOD CmdStp_TOUCH(int f_fast); 

int CmdSet_ARCON(int index, DBL* jnt);
int CmdUpd_ARCON(void);
int CmdSet_ARCOFF(int index, DBL* jnt);
int CmdUpd_ARCOFF(void);
int CmdSet_ARCSET(int index, DBL* jnt);
int CmdSet_WIRE(int index, DBL* jnt); 
int CmdUpd_WIRE(void); 
VOD CmdStp_WELD(int f_fast); 

// port & wait cmd
double TimerCountDown(void);    // returns Expiration Time remained. 

int CmdSet_PORT(int index, DBL* jnt); 
int CmdSet_TIMER(int index, DBL* jnt); 
int CmdUpd_TIMER(void); 
VOD CmdStp_TIMER(int f_fast); 
// Set : CmdSet_TIMER(..)
// Exits if the condition is satisfied. 
int CmdUpd_WAIT(void); 
// Set : CmdSet_TIMER(..)
// Waits until the condition is satisfied
int CmdUpd_UNTIL(void); 

// Waits infinite until Stop requested. 
int CmdSet_PAUSE(int index, DBL* jnt); 
int CmdUpd_PAUSE(void); 

#if defined(_cplusplus)
}
#endif

#endif
