#ifndef TE_SERV_H_
#define TE_SERV_H_

// Jog Service Structure
#pragma pack(push, 1)
typedef struct t_msgval_jog
{
    BYTE        nRobNum; 
    char        coord;      // Coord Index(COORDINDEX_xxx). ex)COORDINDEX_BASE
    BYTE        axis   : 4; // Axis Index(AXIS_xxx). ex)AXIS_LIN_X    
    BYTE        t_keep : 4; // Jog Keep Time = 2^t_keep [ms]
    char        nSpeed;     // [%] -100~100:-100~100%, <-100:(nSpeed+100)*0.1, >100:(nSpeed-100)*0.1
} VALUE_JOG; 
#pragma pack(pop)


#if defined (__cplusplus)
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
// Implemetations 

// Checks mapping of Te Service Registry & TE_SERV_CODE. 
// Suc(-1), Fail(TE_SERV_CODE)
int TeServ_CheckMapping(void);

// Returns Te Service Name. 
const char* TeServ_GetName(TESERV_CODE code); 

// Service Handler of TE 
// 1) Sets data in reply packet 
// 2) Handles Service
// 3) Returns the result of handling. Suc(0), Fail(Else)
// 
// TE Service Hanlder 
// Basic Form : TeServ_xxx(void* buff, int* size, const TE_MSG* p_msg); 
// - buff : Output body data to reply. 
// - size : buff(body) size. Reply Size = size + Header Size. 
// - p_msg: Message data requested. 
// 
// Returns Reply Result(Normally 0) or Unsupported(-1)
int TeServ_Handler(TESERV_CODE code, void* buff, int* p_size, const TE_MSG* p_msg); 

////////////////////////////////////////////////////////////////////////////////
// Handlers 

// Service Handler of TE ///////////////////////////////////////////////////////
// 1) Sets data in reply packet 
// 2) Handles Service
// 3) Returns the result of handling. Suc(0), Fail(Else)

// TE Service Hanlder 
// Basic Form : TeServ_xxx(void* buff, int* size, const TE_MSG* p_msg); 
// - buff : Output body data to reply. 
// - size : buff(body) size. Reply Size = size + Header Size. 
// - p_msg: Message data requested. 

int TeServ_Vers(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_Exit(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_Init(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_Stop(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_TimeTest(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_Jog(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_ProgThru(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_ProgStep(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_ProgDry(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServgRestart(void* buff, int* p_size, const TE_MSG* p_msg); 
#if 0
int TeServ_Vga(void* buff, int* p_size, const TE_MSG* p_msg);
#endif 
int TeServ_DispToggle(void* buff, int* p_size, const TE_MSG* p_msg);
int TeServ_SampTime(void* buff, int* p_size, const TE_MSG* p_msg);
int TeServ_File(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_LifeCheck(void* buff, int* p_size, const TE_MSG* p_msg);
#if 0
int TeServ_EcatPdoCtrl(void* buff, int* p_size, const TE_MSG* p_msg);
#endif
int TeServ_ResetErr(void* buff, int* p_size, const TE_MSG* p_msg); 
#if 0
int TeServ_TrajInfo(void* buff, int* p_size, const TE_MSG* p_msg); 
#endif 
int TeServ_InfoPrint(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_JogMode(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_Jnt2Cart(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_DispCoord(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_TimerSkip(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_PrgVarClr(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_WeldIoMap(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_ProgCtrlVar(void* buff, int* p_size, const TE_MSG* p_msg); 
int TeServ_RuntimePause(void* buff, int* p_size, const TE_MSG* p_msg); 
#if 0
int TeServ_RestartPrint(void* buff, int* p_size, const TE_MSG* p_msg); 
#endif 

#if defined (__cplusplus)
}
#endif

#endif
