#ifndef STR_CONV_H_ 
#define STR_CONV_H_

// #include "trajectory.h"
#include "runtime.h"
#include "run_serv.h"

#if defined (__cplusplus)
extern "C" {
#endif

#define Str_Dir(dir_) ( ((dir_) < 0)? "-" : "+" )

// Returns Coord Name. Not includes the index of USER(Not USR_11, But USR only)
const char* Str_Coord(char i_coord); 

// Returns Axis Name String 
const char* Str_Axis(char i_coord, BYTE i_axis); 

// Returns Runtime Thread Mode in String Form
const char* Str_RunMode(int mode_run); 

// Returns Program Mode
const char* Str_ProgMode(int mode_prog); 

// Returns "ON", "OFF" as 'on' value 1 or 0.
const char* Str_OnOff(unsigned on);

// Not only error message but also includes Normal Event. 
#if 0
const char* Str_Error(int inst, int err); 
#else
const char* Str_Error(unsigned short code); 
#endif

// Returns Section Name of Each Sect. 
const char* Str_Sect(unsigned short sect); 

char* Str_WeldIoMap(void); 

void Str_PrintArg(void); 

char* Str_WeaveSeg(unsigned n_seg); 

char* Str_ProgCtrlVar(void); 

char* Str_CweaveMode(int mode); 
#if 0
// Returns Cmd Name
const char* Str_Cmd(int cmd); 
#endif
#if defined (__cplusplus)
}
#endif

#endif 
