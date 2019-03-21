#ifndef COORD_H_
#define COORD_H_

#include "taskexec_def.h"
#include "arg.h"

#if defined(__cplusplus)
extern "C" {
#endif

#if 1
// Returns COORD wrt BASE. 
// CAUTION!! COORDINDEX_TCP uses bTe with Actual bTe. 
TRANS TargetCoordWrtBase(int coord);

// Returns COORD wrt BASE. 
// CAUTION!! COORDINDEX_TCP uses bTe with Actual Joint. 
TRANS ActualCoordWrtBase(int coord); 
#else
// Returns COORD wrt BASE. 
TRANS CoordWrtBase(int coord, TRANS bTe); 
#endif

// cTb_target
TRANS BaseWrtTargetCoord(int coord); 

// cTt_target
void TcpWrtTargetCoord(TRANS* T, int coord); 

// cTb_actual
TRANS BaseWrtActualCoord(int coord); 

// cTt_actual
void TcpWrtActualCoord(TRANS* T, int coord); 

// JOINT -> ROBPOS wrt JOINT
// ROBPOS is described by deg
void JOINT_2_ROBPOS(ROBOT_POS* rpos, const double* joint); 

#if 0
// bT wrt BASE -> ROBPOS wrt Coord 
// ROBPOS is described by mm and deg
void TRANS_2_ROBPOS(ROBOT_POS* rpos, int coord, const TRANS* bT); 
#endif
// T-> ROBPOS. ROBPOS is described by mm and deg. T by mm and rad. 
void TRANS_2_ROBPOS(ROBOT_POS* rpos, const TRANS* T); 

#if defined(__cplusplus)
}
#endif

#endif