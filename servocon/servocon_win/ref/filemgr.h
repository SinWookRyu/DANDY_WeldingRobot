// FILEMGR.C is the File Write Program. 
// 2013.7.26 mrch0
// 
// Usage : FILE_Open -> Check(File Pointer NULL) -> FILE_Write ... -> FILE_Close
// When opens file, If opened already, no new open. 
// User can modify some Definitions defined in Modify Definition Section 
// E.X) MAX_ROBOT_COUNT..

#ifndef _FILEMGR_H_
#define _FILEMGR_H_

#include "stdio.h"

#if defined(__cplusplus)
extern "C"
{
#endif
 
extern FILE *fpTrg[4]; 
extern FILE *fpAct[4];
extern FILE *fpCart[4]; 
extern FILE *fpProf[4];

int File_Open(int nRob);

void File_Close(int nRob);

void File_Write(const double* profile, const double trg[6], 
                const double cart[6],  const double act[6], int nRob); 

#if defined(_cplusplus)
}
#endif

#endif

