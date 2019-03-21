#ifndef __DANDY_JOB_H__
#define __DANDY_JOB_H__

///////////////////////////////////////

#include "dandy_platform.h"
#include "sys_conf.h"

///////////////////////////////////////

#define DANDY_JOB_MODULE_NAME_SIZE      32

/////////////////////////////////////////////////////////////////////////////
//
//  dandy job commad structures and definitions
//

///////////////////////////////////////
//
//  dandy job commad mnemonics
//

// system command
#define DANDY_JOB_CODE_NOP          0       // just consume one clock, no operation, no argument
#define DANDY_JOB_CODE_SYNC         1       // ARG_SYNC, argSync, broadcast sync number
#define DANDY_JOB_CODE_PAUSE        2       // execution pause until TP/UP continue
#define DANDY_JOB_CODE_COMMENT      3       // ARG_COMMENT, comment or empty line 

// motion command
#define DANDY_JOB_CODE_MOVJ         100     // ARG_MOVE, argMove, Move Joint
#define DANDY_JOB_CODE_MOVL         101     // ARG_MOVE, argMove, Move Linear
#define DANDY_JOB_CODE_MOVC         102     // ARG_MOVE, argMove, Move Circular
#define DANDY_JOB_CODE_MOVO         103     // ARG_MOVE, argMove, Move Orientation
#define DANDY_JOB_CODE_IMOVL        104     // ARG_IMOVE, argIMove, Incrimental Move
#define DANDY_JOB_CODE_HOME         105     // ARG_HOME, argHome, move to home
#define DANDY_JOB_CODE_TOUCH        106     // ARG_TOUCH, argTouch, touch
#define DANDY_JOB_CODE_UWEAVL       107     // ARG_WEAVE, argWeave, Weaving On
#define DANDY_JOB_CODE_CWEAV        108     // ARG_WEAVE, argWeave, Weaving On
#define DANDY_JOB_CODE_HOVER        109     // ARG_HOVER, argHover, hovering

//#define DANDY_JOB_CODE_BLEND        110     // still unsupported
//#define DANDY_JOB_CODE_SCANSEAM     111     // ARG_SCANSEAM, just find out seam
//#define DANDY_JOB_CODE_FINDSEAM     112     // ARG_FINDSEAM, track move to seam
//#define DANDY_JOB_CODE_FINDASP      113     // ARG_FINDASP, track move to arc start point

//#define DANDY_JOB_CODE_ACCEL        120     // ARG_ACCEL, still unsupported, setting accel/decel parameter
//#define DANDY_JOB_CODE_LOG          121     // ARG_LOG, still unsupported, logging

// I/O command
#define DANDY_JOB_CODE_INP          200     // ARG_IO, input digital/analog/plc
#define DANDY_JOB_CODE_OUTP         201     // ARG_IO, output digital/analog/plc
#define DANDY_JOB_CODE_PULSE        202     // ARG_IO, pulse output digital/analog/plc for specified interval
#define DANDY_JOB_CODE_WAIT         203     // ARG_IO, wait for the port value 
#define DANDY_JOB_CODE_UNTIL        204     // ARG_IO, until for the port value 
#define DANDY_JOB_CODE_TIMER        205     // ARG_IO, wait while specified time

// welding commands
#define DANDY_JOB_CODE_ARCON        300     // ARG_ARCON, argArcOn, welding arc-on (ARC2 of Dandy-1996)
#define DANDY_JOB_CODE_ARCOFF       301     // ARG_ARCOFF, argArcOff, welding arc-off
#define DANDY_JOB_CODE_ARCSET       302     // ARG_ARCON, argArcOn, change welding conditon (WSET of Dandy-1996)
#define DANDY_JOB_CODE_WIREF        303     // ARG_WIRE, argWire, forward wire feeding & touch check
#define DANDY_JOB_CODE_WIREB        304     // ARG_WIRE, argWire, backward wire feeding (retract the wire)

// branch commands
#define DANDY_JOB_CODE_CALL         400     // ARG_BRANCH
#define DANDY_JOB_CODE_JUMP         401     // ARG_BRANCH
#define DANDY_JOB_CODE_LABEL        402     // ARG_BRANCH, obsolete in Dandy-2015 (only for Dandy-1996)
#define DANDY_JOB_CODE_RET          403     // no argument
#define DANDY_JOB_CODE_END          404     // no argument
#define DANDY_JOB_CODE_COMP         405     // ARG_ARITH, compare

#define DANDY_JOB_CODE_GETCRD       500     // ARG_GETCOORD, user coordinate reqest
#define DANDY_JOB_CODE_SETCRD       501     // ARG_SETCOORD, user coordinate setting
#define DANDY_JOB_CODE_DEFCRD       502     // ARG_DEFCOORD, default coordinate setting
#define DANDY_JOB_CODE_GETPOS       503     // ARG_GETPOS, get current position
#define DANDY_JOB_CODE_SETPOS       504     // set current position

#define DANDY_JOB_CODE_SET          600     // ARG_ARITH, Assignment
#define DANDY_JOB_CODE_ADD          601     // ARG_ARITH, Addition
#define DANDY_JOB_CODE_SUB          602     // ARG_ARITH, Subtraction
#define DANDY_JOB_CODE_MUL          603     // ARG_ARITH, Multiplication
#define DANDY_JOB_CODE_DIV          604     // ARG_ARITH, Division
#define DANDY_JOB_CODE_MOD          605     // ARG_ARITH, Remainder
#define DANDY_JOB_CODE_AND          606     // ARG_ARITH, And operation
#define DANDY_JOB_CODE_OR           607     // ARG_ARITH, Or operation
#define DANDY_JOB_CODE_XOR          608     // ARG_ARITH, Xor operation
#define DANDY_JOB_CODE_NOT          609     // ARG_ARITH, Not operation

//#define DANDY_JOB_CODE_HOME_ALL     704
//#define DANDY_JOB_CODE_HOME_ROBOT   705
//#define DANDY_JOB_CODE_HOME_GNT     706
//#define DANDY_JOB_CODE_REGPOS       707
//#define DANDY_JOB_CODE_REGUSR       708

///////////////////////////////////////
//
//  DANDY_JOB_VAL
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_VAL
{
    int nValueType;

    union
    {
        BYTE    ByteConst;
        INT     IntConst;
        DOUBLE  RealConst;
        int     nVarIndex;
    } value;

    // available only I-variable indexed type
    // ignore for constant & fixed type
    int         nVarOffset;
} DANDY_JOB_VAL;
#pragma pack(pop)

/*
///////////////////////////////////////
//
//  ARG_SYNC
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_SYNC
{
    // message
    DANDY_JOB_VAL   valSyncNum; // only integer (MIN_SYNC_BROADCAST_NUM ~ MAX_SYNC_BROADCAST_COUNT-1) (app specific)
    DANDY_JOB_VAL   valTimeout;
    DANDY_JOB_VAL   valOption;  // only integer 0 ~ 255 (app specific)
    DANDY_JOB_VAL   valMask;    // only integer 0 ~ 255 (app specific)

    // these param will be copied to SHM_PROC_STATUS::rgdbSyncParam1 & 2 each
    DANDY_JOB_VAL   valParam1;  // app specific number
    DANDY_JOB_VAL   valParam2;  // app specific number
} DANDY_JOB_ARG_SYNC;
#pragma pack(pop)
*/

///////////////////////////////////////
//
//  ARG_COMMENT
//
#define DANDY_JOB_COMMENT_SIZE  256

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_COMMENT
{
    char    szComment[DANDY_JOB_COMMENT_SIZE];
} DANDY_JOB_ARG_COMMENT;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_MOVE
//
//  movj, movl, movc

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_MOVE
{
    DANDY_JOB_VAL   valTargetPos;
    DANDY_JOB_VAL   valViawayPos;   // valid on MOVC

    DANDY_JOB_VAL   valAngle;   // target angle (radian) for multi-turn MOVC. DANDY_JOB_VAL_TYPE_NONE is normal MOVC 
    DANDY_JOB_VAL   valSpeed;   // SPEED_PERCENT/LINEAR/...
    INT             nOriType;   // ORI_TYPE_INCLINE, ORI_TYPE_PATH, ...

    DANDY_JOB_VAL   valCoord;   // COORD_BASE, COORD_WORLD, COORD_TOOL, COORD_USER(index)
    DANDY_JOB_VAL   valPosLevel;
} DANDY_JOB_ARG_MOVE;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_IMOVE
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_IMOVE
{
    DANDY_JOB_VAL   valIncX;
    DANDY_JOB_VAL   valIncY;
    DANDY_JOB_VAL   valIncZ;

    DANDY_JOB_VAL   valSpeed;   // must be SPEED_LINEAR

    DANDY_JOB_VAL   valCoord;   // COORD_BASE, COORD_WORLD, COORD_TOOL, COORD_USER(index)
    DANDY_JOB_VAL   valPosLevel;
} DANDY_JOB_ARG_IMOVE;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_HOME
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_HOME
{
    DANDY_JOB_VAL   valHomeFile;
    DANDY_JOB_VAL   valSpeed;   // must be SPEED_PERCENT
} DANDY_JOB_ARG_HOME;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_ARITH
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_ARITH
{
    // target
    DANDY_JOB_VAL   valTarget;
    int             nTargetElement; // VAL_ELE_NONE, VAL_ELE_J0~31, VAL_ELE_X~RZ, VAL_ELE_B0~31
        
    // source
    DANDY_JOB_VAL   valSource;
    int             nSourceElement; // VAL_ELE_NONE, VAL_ELE_J0~31, VAL_ELE_X~RZ, VAL_ELE_B0~31
} DANDY_JOB_ARG_ARITH;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_ARCON
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_ARCON
{
    DANDY_JOB_VAL   valWeldStart;   // start welding condition file
    DANDY_JOB_VAL   valWeldMain;    // main welding condition file
    DANDY_JOB_VAL   valWeldWeave;   // weav welding condition file (weaving distance range only)
    DANDY_JOB_VAL   valSensorUsed;  // 
} DANDY_JOB_ARG_ARCON;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_ARCOFF
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_ARCOFF
{
    DANDY_JOB_VAL   valWeldEnd;     // end welding condition file
} DANDY_JOB_ARG_ARCOFF;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_TOUCH
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_TOUCH
{
    DANDY_JOB_VAL   valIncX;
    DANDY_JOB_VAL   valIncY;
    DANDY_JOB_VAL   valIncZ;

    DANDY_JOB_VAL   valSpeed;

    INT             nOnFail;    // SEARCH_FAIL_BREAK/CONTINUE

    DANDY_JOB_VAL   valCoord;   // COORD_BASE, COORD_WORLD, COORD_TOOL, COORD_USER(index)
    DANDY_JOB_VAL   valSavePos;
    DANDY_JOB_VAL   valLimit;
} DANDY_JOB_ARG_TOUCH;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_WIRE
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_WIRE
{
    DANDY_JOB_VAL   valSpeed;       // wire moving speed (SPEED_LINEAR)
    DANDY_JOB_VAL   valTimeout;     // wire moving timeout
    DANDY_JOB_VAL   valTouchUsed;   // touch sensor used?
} DANDY_JOB_ARG_WIRE;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_UWEAVE
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_WEAVE
{
    DANDY_JOB_VAL   valTargetPos;   // target postion
    DANDY_JOB_VAL   valViawayPos;   // Viaway postion (only CWEAV)

    union
    {
        // UWEAVL
        struct
        {
            DANDY_JOB_VAL   valDistWeave;   // weaving distance
            DANDY_JOB_VAL   valDistAngle;   // angle distance
            DANDY_JOB_VAL   valDistEnd;     // end distance
        };
        // CWEAV
        struct
        {
            DANDY_JOB_VAL   valDistVert;    // vertical distance
            DANDY_JOB_VAL   valDistHorz;    // horizontal distance
            DANDY_JOB_VAL   valDepth;       // weaving depth
        };
    };
    
    DANDY_JOB_VAL   valWvf;         // weaving condition file
    DANDY_JOB_VAL   valCoord;       // COORD_BASE, COORD_WORLD, COORD_TOOL, COORD_USER(index)
    //
    DANDY_JOB_VAL   valStartPos;    // assumed previous latest position (internal use only)
} DANDY_JOB_ARG_WEAVE;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_HOVER
//

#define TRACK_SENSOR_TYPE_NONE      0
#define TRACK_SENSOR_TYPE_3DLVS     1
#define TRACK_SENSOR_TYPE_6DLVS     2

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_HOVER
{
    int                 nHoverCond;     // OPCOND_NONE, OPCOND_EQ, ...
    DANDY_JOB_ARG_ARITH argArith;       // hovering condition compare

    DANDY_JOB_VAL   valTimeout;     // hovering time

    // tool stare virtual directions
    DANDY_JOB_VAL   valIncX;
    DANDY_JOB_VAL   valIncY;
    DANDY_JOB_VAL   valIncZ;

    int         nSensorType;    // TRACK_SENSOR_TYPE_NONE, TRACK_SENSOR_TYPE_...
    DANDY_JOB_VAL   valFeedRatePercent;

    DANDY_JOB_VAL   valSpeed;       // virtual speed for the direction (valIncX, valIncY, ...)
    DANDY_JOB_VAL   valObjNum;      // tracking object number

    DANDY_JOB_VAL   valCoord;       // COORD_BASE, COORD_WORLD, COORD_TOOL, COORD_USER(index)

    // offset
    DANDY_JOB_VAL   valOfsX;
    DANDY_JOB_VAL   valOfsY;
    DANDY_JOB_VAL   valOfsZ;
    DANDY_JOB_VAL   valOfsCoord;
} DANDY_JOB_ARG_HOVER;
#pragma pack(pop)

/*
///////////////////////////////////////
//
//  ARG_ACCEL
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_ACCEL
{
    INT             nAccelType;     // ACCEL_TRAPEZOIDAL, ACCEL_SCURVE, ACCEL_SETP
    DANDY_JOB_VAL   valAccelTime;   // 0 = max acceleration
} DANDY_JOB_ARG_ACCEL;
#pragma pack(pop)
*/


/*
///////////////////////////////////////
//
//  ARG_LOG
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_CLOG
{
    INT             nReq;           // 0 = off, 1 = on
    DANDY_JOB_VAL   valTimeout;     // unit [msec]
} DANDY_JOB_ARG_CLOG;
#pragma pack(pop)
*/

/*
///////////////////////////////////////
//
//  ARG_BLEND
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_BLEND
{
    INT         nReq;           // 0 = off, 1 = on
} DANDY_JOB_ARG_BLEND;
#pragma pack(pop)
*/

///////////////////////////////////////

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_SCANSEAM
{
    DANDY_JOB_VAL   valIncX;
    DANDY_JOB_VAL   valIncY;
    DANDY_JOB_VAL   valIncZ;

    DANDY_JOB_VAL   valSpeed;   // must be SPEED_LINEAR

    INT         nOnFail;    // SEARCH_ON_FAIL_BREAK/CONTINUE

    DANDY_JOB_VAL   valCoord;   // COORD_BASE, COORD_WORLD, COORD_TOOL, COORD_USER(index)
    DANDY_JOB_VAL   valPosLevel;

    DANDY_JOB_VAL   valSavePos;     // joint position (P var)
    DANDY_JOB_VAL   valSaveGap;     // joint gap of the position
    DANDY_JOB_VAL   valSaveArea;    // joint area of the position
    DANDY_JOB_VAL   valSaveMismatch;// joint mismatch of the position
    DANDY_JOB_VAL   valLimit;

    DANDY_JOB_VAL   valObjNum;

} DANDY_JOB_ARG_SCANSEAM;
#pragma pack(pop)

///////////////////////////////////////

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_FINDSEAM
{
    DANDY_JOB_VAL   valIncX;
    DANDY_JOB_VAL   valIncY;
    DANDY_JOB_VAL   valIncZ;

    DANDY_JOB_VAL   valSpeed1;  // must be SPEED_LINEAR

    INT         nOnFail;    // SEARCH_ON_FAIL_BREAK/CONTINUE

    DANDY_JOB_VAL   valCoord;   // COORD_BASE, COORD_WORLD, COORD_TOOL, COORD_USER(index)
    DANDY_JOB_VAL   valPosLevel;

    DANDY_JOB_VAL   valSavePos;     // joint position (P var)
    DANDY_JOB_VAL   valSaveGap;     // joint gap of the position
    DANDY_JOB_VAL   valSaveArea;    // joint area of the position
    DANDY_JOB_VAL   valSaveMismatch;// joint mismatch of the position
    DANDY_JOB_VAL   valLimit;

    DANDY_JOB_VAL   valSpeed2;
    DANDY_JOB_VAL   valObjNum;

} DANDY_JOB_ARG_FINDSEAM;
#pragma pack(pop)

///////////////////////////////////////
//
//  DANDY_JOB_ARG_FINDASP
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_FINDASP
{
    DANDY_JOB_VAL   valIncX;
    DANDY_JOB_VAL   valIncY;
    DANDY_JOB_VAL   valIncZ;

    DANDY_JOB_VAL   valSpeed;   // must be SPEED_LINEAR

    INT         nOnFail;    // SEARCH_ON_FAIL_BREAK/CONTINUE

    DANDY_JOB_VAL   valCoord;   // COORD_BASE, COORD_WORLD, COORD_TOOL, COORD_USER(index)
    DANDY_JOB_VAL   valObjNum;

    // offset
    DANDY_JOB_VAL   valOfsX;
    DANDY_JOB_VAL   valOfsY;
    DANDY_JOB_VAL   valOfsZ;
    DANDY_JOB_VAL   valOfsCoord;
} DANDY_JOB_ARG_FINDASP;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_GETPOS
//

#define DANDY_JOB_OFFSET_NONE   0   // ignore the offsets
#define DANDY_JOB_OFFSET_JOINT  1   // offset elements are for joint
#define DANDY_JOB_OFFSET_CART   2   // offset element are for cartesian

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_GETPOS
{
    DANDY_JOB_VAL   valPos;         // save target position (P/PI variable)
    DANDY_JOB_VAL   valRefCoord;    // COORD_JOINT, COORD_BASE, COORD_WORLD, COORD_TOOL, COORD_USER, ...

    // offset adding value for each axis
    union
    {
        struct      // valRefCoord == cartesian (COORD_BASE, COORD_WORLD, ...)
        {
            DANDY_JOB_VAL   valX;
            DANDY_JOB_VAL   valY;
            DANDY_JOB_VAL   valZ;
            DANDY_JOB_VAL   valRX;
            DANDY_JOB_VAL   valRY;
            DANDY_JOB_VAL   valRZ;
        } cart;

        DANDY_JOB_VAL   rgJoint[MAX_ROBOT_AXIS_COUNT];  // valRefCoord == COORD_JOINT
        DANDY_JOB_VAL   rgVal[MAX_ROBOT_AXIS_COUNT];
    } offset;

    // offset coord type
    int     nOffsetType;    // DANDY_JOB_OFFSET_NONE/JOINT/CART
} DANDY_JOB_ARG_GETPOS;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_SETCOORD
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_SETCOORD
{
    DANDY_JOB_VAL   valTargetCoord;

    DANDY_JOB_VAL   valOffsetX;
    DANDY_JOB_VAL   valOffsetY;
    DANDY_JOB_VAL   valOffsetZ;
    DANDY_JOB_VAL   valOffsetRX;
    DANDY_JOB_VAL   valOffsetRY;
    DANDY_JOB_VAL   valOffsetRZ;

    DANDY_JOB_VAL   valRefCoord;
} DANDY_JOB_ARG_SETCOORD;
#pragma pack(pop)

///////////////////////////////////////
//
//  ARG_DIO
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_IO
{
    int             nWaitCond;  // wait/until, OPCOND_NONE, OPCOND_EQ, ...
    DANDY_JOB_VAL   valTarget;  // target (variable for inport, port number for outport)
    DANDY_JOB_VAL   valSource;  // source (port number for inport, value for outport)
    DANDY_JOB_VAL   valTimeout;
} DANDY_JOB_ARG_IO;
#pragma pack(pop)

///////////////////////////////////////
//
//  DANDY_JOB_ARG_BRANCH
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_ARG_BRANCH
{
    int             nBranchCond;    // OPCOND_NONE, OPCOND_EQ, ...

    DANDY_JOB_VAL   valAddr;
    char            szName[DANDY_JOB_MODULE_NAME_SIZE];

    // 'if' argument
    DANDY_JOB_VAL   valIfL;
    DANDY_JOB_VAL   valIfR;
} DANDY_JOB_ARG_BRANCH;
#pragma pack(pop)
// ignored 'nBranchCond' and 'valAddr' on 'LABEL' command

///////////////////////////////////////
//
//  DANDY_JOB_CMD
//

#pragma pack(push, 1)
typedef struct _DANDY_JOB_CMD
{
    int     nLineIndex; // zero based line number of the source
    int     nLineId;    // specifed number at first token (default:DANDY_JOB_NO_LINEID)
    int     nCode;      // DANDY_JOB_CODE_NOP/MOVJ/MOVL/...
    int     nReloc;     // relocation info, interanl use only (JOBASM_SYM_ATTR_RELOC_CMD, ...)

    union
    {
//        ARG_SYNC        argSync;        // sync
        DANDY_JOB_ARG_MOVE      argMove;        // movj, movl, movc
        DANDY_JOB_ARG_IMOVE     argIMove;       // imov
        DANDY_JOB_ARG_HOME      argHome;        // home
        DANDY_JOB_ARG_ARCON     argArcOn;       // arcon
        DANDY_JOB_ARG_ARCOFF    argArcOff;      // arcoff
        DANDY_JOB_ARG_TOUCH     argTouch;       // touch
        DANDY_JOB_ARG_WIRE      argWire;        // wiref, wireb

        DANDY_JOB_ARG_ARITH     argArith;       // set, add, sub, div, mul, and, or, xor, comp

        DANDY_JOB_ARG_WEAVE     argWeave;       // uweavl, cweav
        DANDY_JOB_ARG_HOVER     argHover;       // hover

//      DANDY_JOB_ARG_ACCEL     argAccel;       // accel
//      DANDY_JOB_ARG_LOG       argLog;         // log
//      DANDY_JOB_ARG_BLEND     argBlend;       // blend
//      DANDY_JOB_ARG_SCANSEAM  argScanSeam;    // scanseam
//      DANDY_JOB_ARG_FINDSEAM  argFindSeam;    // findseam
//      DANDY_JOB_ARG_FINDASP   argFindAsp;     // findasp

        DANDY_JOB_ARG_IO        argIo;          // din, dout, pulse, wait, timer

        DANDY_JOB_ARG_BRANCH    argBranch;  // call, jump, label
//      DANDY_JOB_ARG_SETCOORD  argSetCoord;    // setcrd
        DANDY_JOB_ARG_GETPOS    argGetPos;  // getpos

        DANDY_JOB_ARG_COMMENT   argComment;

        unsigned char   argByteArray[1]; 
        int             argIntArray[1]; 
    } arg;
} DANDY_JOB_CMD;
#pragma pack(pop)

#define DANDY_JOB_NO_LINEID     -1      // no line identifier

///////////////////////////////////////
//
//  DANDY_POS
//
//  robot position
//  joint : all axis unit = degree
//  carte : 0~2=millimeter, 3~=degree
//
#pragma pack(push, 1)
typedef struct _DANDY_JOB_POS
{
	DOUBLE  pos[MAX_ROBOT_AXIS_COUNT];  // Joint or World Cartesian
	int     nConfig;                    // DANDY_JOB_POS_JOINT/CART
} DANDY_JOB_POS;
#pragma pack(pop)

// DANDY_JOB_POS::nConfig
#define DANDY_JOB_POS_JOINT 0x00000000  // joint
#define DANDY_JOB_POS_RAD   0x00000001  // original pos text is radian-based (just for reference)
#define DANDY_JOB_POS_CART  0x80000000  // cartesian

///////////////////////////////////////
//
//  DANDY_JOB_WVF
//

enum _DANDY_JOB_WVF_ELEMENT
{
    DANDY_JOB_WVF_WIDTH = 0,    // 0
    DANDY_JOB_WVF_PITCH,        // 1
    DANDY_JOB_WVF_DEPTH,        // 2 (unused)
    DANDY_JOB_WVF_SPEED,        // 3
    DANDY_JOB_WVF_ANGLE,        // 4
    DANDY_JOB_WVF_SYMM,         // 5 (unused)
    DANDY_JOB_WVF_DWELL1,       // 6
    DANDY_JOB_WVF_DWELL2,       // 7
    DANDY_JOB_WVF_AUTO_ANGLE,   // 8 (unused)
    DANDY_JOB_WVF_START_WIDTH,  // 9
    DANDY_JOB_WVF_START_PITCH,  // 10
    DANDY_JOB_WVF_START_SPEED,  // 11
    DANDY_JOB_WVF_DWELL3,       // 12
    DANDY_JOB_WVF_SPARE,        // 13 (spare)
    DANDY_JOB_WVF_ELEMENT_COUNT
};

#define DANDY_JOB_WEAV_WIDTH    DANDY_JOB_WVF_WIDTH
#define DANDY_JOB_WEAV_PITCH    DANDY_JOB_WVF_PITCH
#define DANDY_JOB_WEAV_DEPTH    DANDY_JOB_WVF_DEPTH
#define DANDY_JOB_WEAV_SPEED    DANDY_JOB_WVF_SPEED
#define DANDY_JOB_WEAV_ANGLE    DANDY_JOB_WVF_ANGLE
#define DANDY_JOB_WEAV_SYMM     DANDY_JOB_WVF_SYMM
#define DANDY_JOB_WEAV_DWELL1   DANDY_JOB_WVF_DWELL1
#define DANDY_JOB_WEAV_DWELL2   DANDY_JOB_WVF_DWELL2
#define DANDY_JOB_WEAV_AUTO_ANGLE       DANDY_JOB_WVF_AUTO_ANGLE
#define DANDY_JOB_WEAV_START_WIDTH      DANDY_JOB_WVF_START_WIDTH
#define DANDY_JOB_WEAV_START_PITCH      DANDY_JOB_WVF_START_PITCH
#define DANDY_JOB_WEAV_START_SPEED      DANDY_JOB_WVF_START_SPEED
#define DANDY_JOB_WEAV_ELEMENT_COUNT    DANDY_JOB_WVF_ELEMENT_COUNT

#pragma pack(push, 1)
typedef union _DANDY_JOB_WEAV
{
    double  rgdbWeavCond[DANDY_JOB_WVF_ELEMENT_COUNT];

    struct
    {
        double  dbWidth;        //  0, weaving width, [mm]
        double  dbPitch;        //  1, weaving pitch, [mm]
        double  dbDepth;        //  2, weaving depth, [mm], unused 
        double  dbSpeed;        //  3, weaving speed, [mm/s]
        double  dbAngle;        //  4, weaving angle, [deg]
        double  dbSymm;         //  5, start angle, [deg], unused
        double  dbDwell1;       //  6, dwell time 1, [sec]
        double  dbDwell2;       //  7, dwell time 2, [sec]
        double  dbAutoAngle;    //  8, auto angle, 0 or 1, unused
        double  dbStartWidth;   //  9, start width, [mm]
        double  dbStartPitch;   // 10, start pitch, [mm]
        double  dbStartSpeed;   // 11, start speed, [mm/s]
        double  dbDwell3;       // 12, dwell time 3, [sec]
        double  __dbSpare;      // 13, spare
    };
} DANDY_JOB_WEAV;
#pragma pack(pop)

typedef DANDY_JOB_WEAV  DANDY_JOB_WEAVE;
typedef DANDY_JOB_WEAV  DANDY_JOB_WVF;

///////////////////////////////////////
//
//  DANDY_JOB_WELD
//
//  DANDY_JOB_SWF
//  DANDY_JOB_MWF
//  DANDY_JOB_EWF
//

// DANDY_JOB_WELD, welding condition 
#define MAX_WELD_ELEMENT_COUNT  8
#pragma pack(push, 1)
typedef struct _DANDY_JOB_WELD
{
    double  rgdbWeldCond[MAX_WELD_ELEMENT_COUNT];
} DANDY_JOB_WELD;
#pragma pack(pop)

///////////////////////////////////////
//
// DANDY_JOB_SWF
//

enum _DANDY_JOB_SWF_ELEMENT
{
    DANDY_JOB_SWF_VOLTAGE = 0,  // 0, start welding voltage [V]
    DANDY_JOB_SWF_CURRENT,      // 1, start welding current [A]
    DANDY_JOB_SWF_ARC_TIME,     // 2, start welding arc check time (stay time) [sec]
    DANDY_JOB_SWF_PREFLOW,      // 3, shielding gas preflow time [sec]
    DANDY_JOB_SWF_DISTANCE,     // 4, start welding arc augmentation distance [mm]
    DANDY_JOB_SWF_ELEMENT_COUNT
};

#pragma pack(push, 1)
typedef union _DANDY_JOB_SWF
{
    DANDY_JOB_WELD      swf;

    struct
    {
        double  dbVoltage;      // start welding voltage, unit [V]
        double  dbCurrent;      // start welding current, unit [A]
        double  dbArcTime;      // arc check/endurance time [sec]
        double  dbPreflowTime;  // shielding gas pre-flow time, unit [sec]
        double  dbArcDist;      // arc augmentation moving distance, unit [mm]
    };
} DANDY_JOB_SWF;
#pragma pack(pop)
// starting welding voltage = DANDY_JOB_SWF.dbVoltage + DANDY_JOB_MWF.dbVoltage
// starting welding current = DANDY_JOB_SWF.dbCurrent + DANDY_JOB_MWF.dbCurrent

///////////////////////////////////////
//
// DANDY_JOB_MWF
//

enum _DANDY_JOB_MWF_ELEMENT
{
    DANDY_JOB_MWF_VOLTAGE = 0,  // 0, main welding voltage [V]
    DANDY_JOB_MWF_CURRENT,      // 1, main welding current [A]
    DANDY_JOB_MWF_SPEED,        // 2, main welding speed [mm/s]
    DANDY_JOB_MWF_ELEMENT_COUNT
};

#pragma pack(push, 1)
typedef union _DANDY_JOB_MWF
{
    DANDY_JOB_WELD      mwf;

    struct
    {
        double  dbVoltage;      // main welding voltage, unit [V]
        double  dbCurrent;      // main welding current, unit [A]
        double  dbSpeed;        // torch(gun) moving speed, unit [mm/s]
    };
} DANDY_JOB_MWF;
#pragma pack(pop)

///////////////////////////////////////
//
// DANDY_JOB_EWF
//

enum _DANDY_JOB_EWF_ELEMENT
{
    DANDY_JOB_EWF_VOLTAGE = 0,  // 0, end welding crater voltage [V]
    DANDY_JOB_EWF_CURRENT,      // 1, end welding crater current [A]
    DANDY_JOB_EWF_CRATER_TIME,  // 2, end welding crater stay time [sec]
    DANDY_JOB_EWF_POSTFLOW,     // 3, end welding post flow time [sec]
    DANDY_JOB_EWF_ELEMENT_COUNT
};

#pragma pack(push, 1)
typedef union _DANDY_JOB_EWF
{
    DANDY_JOB_WELD      ewf;

    struct
    {
        double  dbVoltage;      // creater processing coltage, unit [A]
        double  dbCurrent;      // creater processing current, unit [A]
        double  dbCraterTime;   // creater processing time, unit [msec]
        double  dbPostflowTime; // post flow time time, unit [msec]
    };
} DANDY_JOB_EWF;
#pragma pack(pop)
// ending welding voltage = DANDY_JOB_EWF.dbVoltage + DANDY_JOB_MWF.dbVoltage
// ending welding current = DANDY_JOB_EWF.dbCurrent + DANDY_JOB_MWF.dbCurrent

///////////////////////////////////////
//
//  DANDY_JOB_RWF
//
//  obsolete (do not use)
//

enum _DANDY_JOB_RWF_ELEMENT
{
    DANDY_JOB_RWF_VOLTAGE = 0,  // 0, restart welding voltage [V]
    DANDY_JOB_RWF_CURRENT,      // 1, restart welding current [A]
    DANDY_JOB_RWF_ARCTIME,      // 2, restart welding arc check time (stay time) [sec]
    DANDY_JOB_RWF_PREFLOW,      // 3, restart shielding gas preflow time [sec]
    DANDY_JOB_RWF_DISTANCE,     // 4, restart welding arc augmentation distance [mm]
    DANDY_JOB_RWF_OVDIST,       // 5, restart overlap distance
    DANDY_JOB_RWF_RETBACKVEL,   // 6, restart return back velocity
    DANDY_JOB_RWF_ELEMENT_COUNT
};


///////////////////////////////////////
//
//  DANDY_JOB_COORD_xxxx
//
//  re-definition for coordinate from "sys_conf.h"
//

#define DANDY_JOB_COORD_USER    COORDINDEX_USER(0)
#define DANDY_JOB_COORD_JOINT   COORDINDEX_JOINT    // not formal coord (used only in jog)
#define DANDY_JOB_COORD_WORLD   COORDINDEX_WORLD
#define DANDY_JOB_COORD_BASE    COORDINDEX_BASE
#define DANDY_JOB_COORD_END     COORDINDEX_END      // end link (not wrist)
#define DANDY_JOB_COORD_TOOL    COORDINDEX_TOOL
#define DANDY_JOB_COORD_SENSOR  COORDINDEX_SENSOR
// for internal use
#define DANDY_JOB_COORD_UNKNOWN -128                        // how we should treat this coordinate...
#define DANDY_JOB_COORD_IGNORE  (DANDY_JOB_COORD_UNKNOWN-1) // ignore (== COORD_UNKNOWN)

///////////////////////////////////////
//
//  DANDY_JOB_COORD
//
//  job coordinate
//
/*
#pragma pack(push, 1)
typedef union _DANDY_JOB_COORD
{
    double  rgdb[MAX_JOINT_COUNT];  // x-y-z, r-p-y
                                    // or joint    
    struct
    {
        double  x;
        double  y;
        double  z;
        double  rx;
        double  ry;
        double  rz;
    };
} DANDY_JOB_COORD;	
#pragma pack(pop)
*/

///////////////////////////////////////
//
//  DANDY_JOB_SPEED_xxxx
//
//  speed unit/type
//

#define DANDY_JOB_SPEED_LINEAR      0   // cartesian base speed = mm/ms, rad/ms
#define DANDY_JOB_SPEED_PERCENT     1   // joint moving only (MOVJ, HOME)
#define DANDY_JOB_SPEED_ORIENT      2   // rotation speed
#define DANDY_JOB_SPEED_EXTERN      3   // 

///////////////////////////////////////
//
//  DANDY_JOB_ORI_TYPE_xxxx
//
//  orientation type at target position
//  use at MOVL, MOVC
//

#define DANDY_JOB_ORI_TYPE_INCLINE  0   // (default) orientation reached at target orientation finally
#define DANDY_JOB_ORI_TYPE_TARGET   1   // fixed orientatin with target before travel
#define DANDY_JOB_ORI_TYPE_START    2   // fixed orientatin with start orientation
#define DANDY_JOB_ORI_TYPE_PATH     3   // orientation decided by path automatically

#define DANDY_JOB_ORI_TYPE_INC      DANDY_JOB_ORI_TYPE_INCLINE

///////////////////////////////////////
//
//  DANDY_JOB_SEARCH_ON_FAIL_xxxx
//
//  TOUCH / SCANSEAM / FINDSEAM / FINDASP on fail post-action
//

#define DANDY_JOB_SEARCH_FAIL_BREAK     0   // job exec break on fail
#define DANDY_JOB_SEARCH_FAIL_CONTINUE  1   // job exec continue even failure

///////////////////////////////////////
//
//  DANDY_JOB_VAL_TYPE_xxxx
//

#define DANDY_JOB_VAL_TYPE_NONE         -1  // internal use only, do not use

// constant
#define DANDY_JOB_VAL_TYPE_B_CONST      0   // obsolete (only for dandy1 compatible)
#define DANDY_JOB_VAL_TYPE_I_CONST      1   // 10, 20, 30, ...
#define DANDY_JOB_VAL_TYPE_R_CONST      2   // 1.1, 1.2, 1.3, ...
#define DANDY_JOB_VAL_TYPE_STRING       3   // "ABCDEFG"

// sigle variable
#define DANDY_JOB_VAL_TYPE_T_VAR        10  // T001, T[001], T(001), directly unused
#define DANDY_JOB_VAL_TYPE_P_VAR        11  // P001, P[001], P(001)
#define DANDY_JOB_VAL_TYPE_B_VAR        12  // obsolete (only for dandy1 compatible)
#define DANDY_JOB_VAL_TYPE_I_VAR        13  // I001, I[001], I(001)
#define DANDY_JOB_VAL_TYPE_R_VAR        14  // R001, R[001], R(001)
#define DANDY_JOB_VAL_TYPE_WVF_VAR      15  // weaving variable
#define DANDY_JOB_VAL_TYPE_SWF_VAR      16  // start welding condition file var
#define DANDY_JOB_VAL_TYPE_MWF_VAR      17  // main welding condition file var
#define DANDY_JOB_VAL_TYPE_EWF_VAR      18  // end welding condition file var
#define DANDY_JOB_VAL_TYPE_DIP_VAR      19  // digital inport variable
#define DANDY_JOB_VAL_TYPE_DOP_VAR      20  // digital outport variable
#define DANDY_JOB_VAL_TYPE_AIP_VAR      21  // analog inport variable
#define DANDY_JOB_VAL_TYPE_AOP_VAR      22  // analog outport variable
#define DANDY_JOB_VAL_TYPE_PLC_VAR      23  // PLC
#define DANDY_JOB_VAL_TYPE_U_VAR        24  // user coord

// integer variable indexed variable
#define DANDY_JOB_VAL_TYPE_TI_VAR       30  // T[I[001]], T(I(001)), directly unused
#define DANDY_JOB_VAL_TYPE_PI_VAR       31  // P[I[001]], P(I(001)), int var indexed
#define DANDY_JOB_VAL_TYPE_BI_VAR       32  // unused
#define DANDY_JOB_VAL_TYPE_II_VAR       33  // I[I[001]], I(I(001)), int var indexed
#define DANDY_JOB_VAL_TYPE_RI_VAR       34  // R[I[001]], R(I(001)), int var indexed
#define DANDY_JOB_VAL_TYPE_WVFI_VAR     35  // 
#define DANDY_JOB_VAL_TYPE_SWFI_VAR     36  // int indexed start welding condition file var
#define DANDY_JOB_VAL_TYPE_MWFI_VAR     37  // int indexed main welding condition file var
#define DANDY_JOB_VAL_TYPE_EWFI_VAR     38  // int indexed end welding condition file var
#define DANDY_JOB_VAL_TYPE_DIPI_VAR     39  // int indexed digital inport variable
#define DANDY_JOB_VAL_TYPE_DOPI_VAR     40  // int indexed digital outport variable
#define DANDY_JOB_VAL_TYPE_AIPI_VAR     41  // int indexed analog inport variable
#define DANDY_JOB_VAL_TYPE_AOPI_VAR     42  // int indexed analog outport variable
#define DANDY_JOB_VAL_TYPE_PLCI_VAR     43
#define DANDY_JOB_VAL_TYPE_UI_VAR       44

// internal use only types (for compiler)
#define DANDY_JOB_VAL_TYPE_RWF_VAR      100 // restart welding condition file var

//
#define DANDY_JOB_IS_CONST_VAL(__type)      ((__type) >= 0 && (__type) <= 3)
#define DANDY_JOB_IS_FIXED_VAR(__type)      ((__type) >= 10 && (__type) <= 24)
#define DANDY_JOB_IS_INDEXED_VAR(__type)    ((__type) >= 30 && (__type) <= 44)

#define DANDY_JOB_CONV_TO_FIXED_VAR(__indexed_type) \
        ((__indexed_type) - (DANDY_JOB_VAL_TYPE_TI_VAR - DANDY_JOB_VAL_TYPE_T_VAR))
#define DANDY_JOB_CONV_TO_INDEXED_VAR(__fixed_type) \
        ((__fixed_type) + (DANDY_JOB_VAL_TYPE_TI_VAR - DANDY_JOB_VAL_TYPE_T_VAR))

///////////////////////////////////////
//
//  DANDY_JOB_VAL_ELE_xxxx
//
//  variable element
//

#define DANDY_JOB_VAL_ELE_NONE  -1
        ///////////
        // member/joint elements : M0~, J0~
#define DANDY_JOB_VAL_ELE_M0    0
#define DANDY_JOB_VAL_ELE_M1    1
#define DANDY_JOB_VAL_ELE_M2    2
#define DANDY_JOB_VAL_ELE_M3    3
#define DANDY_JOB_VAL_ELE_M4    4
#define DANDY_JOB_VAL_ELE_M5    5
#define DANDY_JOB_VAL_ELE_M6    6
        //..

        ///////////
        // joint elements : j0, j1, j2, ... j0+MAX_ROBOT_AXIS_COUNT
#define DANDY_JOB_VAL_ELE_J0    0
#define DANDY_JOB_VAL_ELE_J1    1
#define DANDY_JOB_VAL_ELE_J2    2
#define DANDY_JOB_VAL_ELE_J3    3
#define DANDY_JOB_VAL_ELE_J4    4
#define DANDY_JOB_VAL_ELE_J5    5
#define DANDY_JOB_VAL_ELE_J6    6
        // ~ DANDY_JOB_VAL_ELE_J0 + MAX_ROBOT_AXIS_COUNT
#define DANDY_JOB_VAL_ELE_MAX   (DANDY_JOB_VAL_ELE_J0 + MAX_ROBOT_AXIS_COUNT)

        ///////////
        // cartesian elements : x, y, z, rx, ...
#define DANDY_JOB_VAL_ELE_X     100
#define DANDY_JOB_VAL_ELE_Y     101
#define DANDY_JOB_VAL_ELE_Z     102
#define DANDY_JOB_VAL_ELE_RX    103
#define DANDY_JOB_VAL_ELE_RY    104
#define DANDY_JOB_VAL_ELE_RZ    105

#define DANDY_JOB_VAL_ELE_CART0 DANDY_JOB_VAL_ELE_X
#define DANDY_JOB_VAL_ELE_CART6 (DANDY_JOB_VAL_ELE_X+6)
        // extra cartesian compo will be added from DANDY_JOB_VAL_ELE_CART6 (106)

        ///////////
        // bit elements : b0 ~ b31
#define DANDY_JOB_VAL_ELE_B0    200
#define DANDY_JOB_VAL_ELE_B1    201
#define DANDY_JOB_VAL_ELE_B2    202
#define DANDY_JOB_VAL_ELE_B3    203
#define DANDY_JOB_VAL_ELE_B4    204
#define DANDY_JOB_VAL_ELE_B5    205
#define DANDY_JOB_VAL_ELE_B6    206
        //...
#define DANDY_JOB_VAL_ELE_B31   (DANDY_JOB_VAL_ELE_B0 + 31) // 231

///////////////////////////////////////
//
// compare operator
// relational operator (used in if statement)
//

#define DANDY_JOB_OPCOND_NONE   -1      // always false
#define DANDY_JOB_OPCOND_EQ     0x00    // ==   equal
#define DANDY_JOB_OPCOND_NE     0x01    // !=   equal
#define DANDY_JOB_OPCOND_LT     0x10    // <    less than           (signed result)
#define DANDY_JOB_OPCOND_LE     0x11    // <=   less or equal       (signed result)
#define DANDY_JOB_OPCOND_GT     0x12    // >    greater than        (signed result)
#define DANDY_JOB_OPCOND_GE     0x13    // >=   greater or equal    (signed result)
//#define DANDY_JOB_OPCOND_BT     0x20    // <    blow than           (usigned result)
//#define DANDY_JOB_OPCOND_BE     0x21    // <    blow or equal       (usigned result)
//#define DANDY_JOB_OPCOND_AT     0x22    // <    above than          (usigned result)
//#define DANDY_JOB_OPCOND_AE     0x23    // <    above or equal      (usigned result)

///////////////////////////////////////
//
//  default values
//

#define DANDY_JOB_DEF_PL        0                           // PL=0
#define DANDY_JOB_DEF_COORD     DANDY_JOB_COORD_WORLD       // CRD=WORLD
#define DANDY_JOB_DEF_HOME      0                           // HF=0
#define DANDY_JOB_DEF_ORITYPE   DANDY_JOB_ORI_TYPE_INCLINE
#define DANDY_JOB_DEF_ONFAIL    DANDY_JOB_SEARCH_FAIL_BREAK
#define DANDY_JOB_DEF_SENSOR    DANDY_FALSE                 // SEN=OFF

///////////////////////////////////////

#endif
