// Independent Utilities to TAKSEXEC. 
// Dependent Utilities at 'TASKEXEC_DEF.H'

#ifndef UTILITY_H_
#define UTILITY_H_

#include "float.h"

#ifndef PI
#define PI (3.141592653589793238462643383279)
#endif

#define ON  1
#define OFF 0

// returns if 'value_' is 0. 
// In the void return function the form is possible, ASSERT_RETURN(value, );. 
#define ASSERT_RETURN(value_, ret_)     \
{                                       \
    if(!(value_))                       \
    {                                   \
        return ret_;                    \
    }                                   \
}      

// returns if 'value_' is not 0
// In the void return function the form is possible, EXIST_RETURN(value, );. 
#define EXIST_RETURN(value_, ret_)      \
{                                       \
    if((value_))                        \
    {                                   \
        return  ret_ ;                  \
    }                                   \
}      

#define IS_ODD_NUM(n_)     ((n_) % 2)
#define IS_EVEN_NUM(n_)    (!IS_ODD_NUM(n_))

// For using of Large Number such as Jog Distance 
// Designed to allow 'sqrt' operation (LARGE_NUM^2 < DBL_MAX)
// DBL_MAX : 1.7976931348623158e+308 
#define LARGE_NUM           (1.7976931348623158e+150) 
#define EPSILON             DBL_EPSILON        
#define EPS_T               0.001   // EPSILON TIME 0.001
#if 0
#define EPS_P               0.001   // EPSILON POS  0.001 [mm]
#define EPS_O               0.000001// EPSILON ORI  0.001 [mrad]
#else   // correcting comparison to vel of epsilon. 
#define EPS_P               0.000001// EPSILON POS & POS VEL 0.001 [mm, mm/ms]
#define EPS_O               0.000001// EPSILON ORI & ORI VEL 0.001 [mrad, mrad/ms]
#endif 


#define SET_ERROR(ptr_, code_)    *(ptr_) = (code_);

typedef void        VOD; 
typedef unsigned    UNT; 
typedef double      DBL; 
typedef char        CHR; 
typedef char*       STR; 
typedef unsigned char UCH;
// typedef unsigned int uint;
typedef unsigned int UINT;


#endif
