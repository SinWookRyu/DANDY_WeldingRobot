#ifndef KINEMATICS_H_20130820_
#define KINEMATICS_H_20130820_

// Inverse Error 
#define ERR_INVALID_PARAM       1
#define ERR_SINGULAR_NEAR_Z0    2 
#define ERR_SINGULAR_TH5        3
#define ERR_UNREACHABLE         4
#define ERR_LARGE_VEL           5
    
#if defined(__cplusplus)
extern "C" {
#endif

void For_Dandy2(TRANS *bTe,             //[out]trans of base->6
                const double *th,       //[in] physical joint.
                const DH *dh);          //[in] dh-param

// Returns Dandy2 Configuration
//   1st bit : ARM    : FRONT(1) | BACK(0)
//   2nd bit : ELBOW  : ABOVE(1) | BELOW(0)
//   3rd bit : FLIP   : FLIP(1)  | NON(0)
// Conf of Normal Below Looking Pose is 0x07. (ARM-Front, Elbow-Above, Flip)
unsigned Conf_Dandy2(const double* th, const DH *dh);

// Gets theta for trans. from base to 6th link, no regards to TCP.
// return suc(0), z0-singular(-1), th5-singular(-2), unreachable(-3)
// Robot Pose Configuration
// - ARM(1st bit)   : FRONT(1), BACK(0)
// - ELBOW(2nd bit) : ABOVE(1), BELOW(0)
// - WRIST(3rd bit) : FLIP(1),  NO-FLIP(0)
// - 0b111 : FRONT-ABOVE-FLIP - Normal Pose
int Inv_Dandy2( double*  pTh,           // [out]physical joint angles.
                const TRANS* bTe,       // [in] trans from BASE to LINK6                                      
                unsigned conf,          // [in] configuration of robot pose
                const DH *pDh,          // [in] dh-param
                const double th_prev[6],// [in] previouss Joint Value
                unsigned f_running);    // [in] running flag

#if defined(__cplusplus)
}
#endif

#endif
