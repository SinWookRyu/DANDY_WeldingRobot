/////////////////////////////////////////////////////////////////////////////
//
//  err_parsing.c: Error Code Parsing
//                                            2014.02.25  Ryu SinWook

///////////////////////////////////////

#include "robotmgr_main.h"

///////////////////////////////////////

///////////////////////////////////////
//
//  Function: SYSMON_ParceErrCodeToErrContent()
//      - Service Name: RMGR_SERV_SYSSTATE

int SYSMON_ParceErrCodeToErrContent(int s_nErrCode)
{
    static int s_nErrMode;
    static int s_nErrOwner;

    s_nErrOwner = s_nErrCode & 0xf000000;
    s_nErrMode  = s_nErrCode & 0x0ff0000;
    s_nErrCode  = s_nErrCode & 0x000ffff;

    if(s_nErrOwner == ERR_OWNER_FROM_TE)
    {
        //CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TE INTERNAL ERROR");

        if(s_nErrCode == ERR_NONE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_NONE");
        }
        else if(s_nErrCode == ERR_NULL_PTR)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_NULL_PTR");
        }
        else if(s_nErrCode == ERR_NO_JOBSHM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NO_JOBSHM");
        }
        else if(s_nErrCode == ERR_CMDINIT_FAIL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMDINIT_FAIL");
        }
        else if(s_nErrCode == ERR_UNSUPPORT_CMD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "UNSUPPORT_CMD");
        }
        else if(s_nErrCode == ERR_INVALID_ARGVAL_TYPE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_ARGVAL_TYPE");
        }
        else if(s_nErrCode == ERR_UNSUPPORT_VAL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "UNSUPPORT_VAL");
        }
        else if(s_nErrCode == ERR_NO_JNT_ACCESS)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NO_JNT_ACCESS");
        }
        else if(s_nErrCode == ERR_NO_CART_ACCESS)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NO_CART_ACCESS");
        }
        else if(s_nErrCode == ERR_UNSUPPORT_DBL_OPER)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "UNSUPPORT_DBL_OPER");
        }
        else if(s_nErrCode == ERR_UNSUPPORT_OPER_CMD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "UNSUPPORT_OPER_CMD");
        }
        else if(s_nErrCode == ERR_UNSUPPORT_BRANCH_CMD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "UNSUPPORT_BRANCH_CMD");
        }
        else if(s_nErrCode == ERR_UNDEF_STEP)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_UNDEF_STEP");
        }
        else if(s_nErrCode == ERR_STOP_REQ)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "STOP_REQ");
        }
        else if(s_nErrCode == ERR_QUICK_STOP_REQ)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "QUICK_STOP_REQ");
        }
        else if(s_nErrCode == ERR_ESTOP_ON)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ESTOP_ON");
        }
        else if(s_nErrCode == ERR_MOTOR_ERR)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "MOTOR_ERR");
        }
        else if(s_nErrCode == ERR_SERVO_OFF)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SERVO_OFF");
        }
        else if(s_nErrCode == ERR_NO_RESTART_DATA)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NO_RESTART_DATA");
        }
        else if(s_nErrCode == ERR_NO_LMOT_DATA_SAVED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NO_LMOT_DATA_SAVED");
        }
        else if(s_nErrCode == ERR_NO_WEAVE_DATA_SAVED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NO_WEAVE_DATA_SAVED");
        }
        else if(s_nErrCode == ERR_TRAJ_ERR_NO_RESTART)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_ERR_NO_RESTART");
        }
        else if(s_nErrCode == ERR_INVALID_AXIS)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_AXIS");
        }
        else if(s_nErrCode == ERR_INVALID_COORD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_COORD");
        }
        else if(s_nErrCode == ERR_INVALID_DIR)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_DIR");
        }
        else if(s_nErrCode == ERR_WATCHDOG_OVER)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WATCHDOG_OVER");
        }
        else if(s_nErrCode == ERR_TIME_OVER)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TIME_OVER");
        }
        else if(s_nErrCode == ERR_STACK_NOMORE_PUSH)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "STACK_NOMORE_PUSH");
        }
        else if(s_nErrCode == ERR_STACK_NOMORE_POP)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "STACK_NOMORE_POP");
        }
        else if(s_nErrCode == ERR_JOBLOAD_FAIL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBLOAD_FAIL");
        }
        else if(s_nErrCode == ERR_TCH_NOT_READY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TCH_NOT_READY");
        }
        else if(s_nErrCode == ERR_TOUCH_SENSOR_RESET_FAIL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TOUCH_RESET_FAIL");
        }
        else if(s_nErrCode == ERR_TOUCH_FAIL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TOUCH_FAIL");
        }
        else if(s_nErrCode == ERR_TOUCH_ALREADY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TOUCH_ALREADY");
        }
        else if(s_nErrCode == ERR_WELDER_POWER_FAIL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WELDER_POWER_FAIL");
        }
        else if(s_nErrCode == ERR_ARCON_SIG_FAIL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ARCON_SIG_FAIL");
        }
        else if(s_nErrCode == ERR_GASON_SIG_FAIL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "GASON_SIG_FAIL");
        }
        else if(s_nErrCode == ERR_TRAJ_START)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_START");
        }
        else if(s_nErrCode == 0x0301)   //TRAJ_ERR_WRONG_PARAM
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_WRONG_PARAM");
        }
        else if(s_nErrCode == 0x0302)   //TRAJ_ERR_PROFILE_FAIL
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_PROFILE_FAIL");
        }
        else if(s_nErrCode == 0x0303)   //TRAJ_ERR_MOTION_FAIL
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_MOTION_FAIL");
        }
        else if(s_nErrCode == 0x0304)   //TRAJ_ERR_ZERO_VEL
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_ERR_ZERO_VEL");
        }
        else if(s_nErrCode == 0x0305)   //TRAJ_ERR_LIMIT_POS
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_POS");
        }
        else if(s_nErrCode == 0x0306)   //TRAJ_ERR_LIMIT_VEL
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_VEL");
        }
        else if(s_nErrCode == 0x0307)   //TRAJ_ERR_CONF_MISMATCH
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_CONF_MISMATCH");
        }
        else if(s_nErrCode == 0x0308)   //TRAJ_ERR_UNDEF_CIRCLE
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_UNDEF_CIRCLE");
        }
        else if(s_nErrCode == 0x0309)   //TRAJ_ERR_NEAR_POS
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_NEAR_POS");
        }
        else if(s_nErrCode == 0x0310)   //TRAJ_ERR_UNDEF_INVERSE
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_UNDEF_INVERSE");
        }
        else if(s_nErrCode == 0x0311)   //TRAJ_ERR_INVERSE_Z0
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_INVERSE_Z0");
        }
        else if(s_nErrCode == 0x0312)   //TRAJ_ERR_INVERSE_TH5
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_INVERSE_TH5");
        }
        else if(s_nErrCode == 0x0313)   //TRAJ_ERR_INVERSE_UNREACH
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_INVERS_UNREACH");
        }
        else if(s_nErrCode == 0x0314)   //TRAJ_ERR_INVERSE_LARGE_V
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_INVERS_LARGE_V");
        }
        else if(s_nErrCode == 0x0320)   //TRAJ_ERR_LIMIT_POS_0
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_POS_0");
        }
        else if(s_nErrCode == 0x0321)   //TRAJ_ERR_LIMIT_POS_1
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_POS_1");
        }
        else if(s_nErrCode == 0x0322)   //TRAJ_ERR_LIMIT_POS_2
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_POS_2");
        }
        else if(s_nErrCode == 0x0323)   //TRAJ_ERR_LIMIT_POS_3
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_POS_3");
        }
        else if(s_nErrCode == 0x0324)   //TRAJ_ERR_LIMIT_POS_4
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_POS_4");
        }
        else if(s_nErrCode == 0x0325)   //TRAJ_ERR_LIMIT_POS_5
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_POS_5");
        }
        else if(s_nErrCode == 0x0330)   //TRAJ_ERR_LIMIT_VEL_0
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_VEL_0");
        }
        else if(s_nErrCode == 0x0331)   //TRAJ_ERR_LIMIT_VEL_1
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_VEL_1");
        }
        else if(s_nErrCode == 0x0332)   //TRAJ_ERR_LIMIT_VEL_2
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_VEL_2");
        }
        else if(s_nErrCode == 0x0333)   //TRAJ_ERR_LIMIT_VEL_3
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_VEL_3");
        }
        else if(s_nErrCode == 0x0334)   //TRAJ_ERR_LIMIT_VEL_4
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_VEL_4");
        }
        else if(s_nErrCode == 0x0335)   //TRAJ_ERR_LIMIT_VEL_5
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TRAJ_LIMIT_VEL_5");
        }
        else if(s_nErrCode == ERR_ARG_ELEM_MISMATCH)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ARG_ELEM_MISMATCH");
        }
        else if(s_nErrCode == 0x0402)   //ERR_DIV_BY_0
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_DIV_BY_0");
        }
        else if(s_nErrCode == 0x0500)   //WV_ERR_NONE
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_ERR_NONE");
        }
        else if(s_nErrCode == 0x0501)   //WV_ERR_NULL_PTR
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_NULL_PTR");
        }
        else if(s_nErrCode == 0x0502)   //WV_ERR_INV_PARAM
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_INV_PARAM ");
        }
        else if(s_nErrCode == 0x0503)   //WV_ERR_NO_WX_DIR
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_NO_WX_DIR");
        }
        else if(s_nErrCode == 0x0504)   //WV_ERR_TRAJ
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_TRAJ");
        }
        else if(s_nErrCode == 0x0505)   //WV_ERR_SHORT_DIST
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_SHORT_DIST");
        }
        else if(s_nErrCode == 0x0506)   //WV_ERR_UNDEF_MODE
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_UNDEF_MODE");
        }
        else if(s_nErrCode == 0x0507)   //WV_INV_BASE_PLANE_OR_WV_DIR
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_INV_BPLANE_DIR ");
        }
        else if(s_nErrCode == 0x0508)   //WV_ERR_LONG_START_END_DIST
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_DIST_SHORT");
        }
        else if(s_nErrCode == 0x0509)   //WV_ERR_LONG_ANGLE_DIST
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WV_LONG_ANGLE_DIST");
        }
        else if(s_nErrCode == ERR_INVERSE_START)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVERSE_START");
        }
        else if(s_nErrCode == 0x0601)   //ERR_INVALID_PARAM
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_PARAM");
        }
        else if(s_nErrCode == 0x0602)   //ERR_SINGULAR_NEAR_Z0
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SINGULAR_NEAR_Z0");
        }
        else if(s_nErrCode == 0x0603)   //ERR_SINGULAR_TH5
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SINGULAR_TH5");
        }
        else if(s_nErrCode == 0x0604)   //ERR_UNREACHABLE
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "UNREACHABLE");
        }
        else if(s_nErrCode == 0x0605)   //ERR_LARGE_VEL
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "LARGE_VEL");
        }

        /* Not Defined Error */
        else
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NOT DEFINED ERROR");
        }


        /* Error Mode */
        if(s_nErrMode == 0x000000)  //SECT_NONE
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "       ");
        }
        else if(s_nErrMode == 0x010000)  //SECT_RUN
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "RUN");
        }
        else if(s_nErrMode == 0x020000)  //SECT_JOG
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "JOG");
        }
        else if(s_nErrMode == 0x030000)  //SECT_PROG
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "PROG");
        }
        else if(s_nErrMode == 0x040000)  //SECT_CMD_MOV
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "CMDMOV");
        }
        else if(s_nErrMode == 0x050000)  //SECT_CMD_TOUCH
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "CMDTCH");
        }
        else if(s_nErrMode == 0x060000)  //SECT_CMD_WELD
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "CMDWLD");
        }
        else if(s_nErrMode == 0x070000)  //SECT_CMD_PORT
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "CMDPRT");
        }
        else if(s_nErrMode == 0x080000)  //SECT_CMD_BRANCH
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "CMDBRCH");
        }
        else if(s_nErrMode == 0x090000)  //SECT_CMD_CWEAV
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "CMDCWV");
        }
        else if(s_nErrMode == 0x0A0000)  //SECT_RESTART
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "REST");
        }
    }
    else
    {
        /* Error OK */
        if(s_nErrCode == SYS_ERR_OK && g_fErrorReset == OFF)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NO ERROR            ");
        }
        else if(s_nErrCode == SYS_ERR_OK && g_fErrorReset == ON)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "RESET ERROR         ");
        }

        /* System Error */
        else if(s_nErrCode == SYS_ERR_INIT_RM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_INIT_RM");
        }
        else if(s_nErrCode == SYS_ERR_INIT_TE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_INIT_TE");
        }
        else if(s_nErrCode == SYS_ERR_INIT_SC)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_INIT_SC");
        }
#if 1
        else if(s_nErrCode == SYS_ERR_INIT_CONF_LOAD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_INIT_CONF_LOAD");
        }
#endif
        else if(s_nErrCode == SYS_ERR_SYNTAX_GLOBAL_PARAM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SYNTAX_GLOBAL_PARAM");
        }
        else if(s_nErrCode == SYS_ERR_SYNTAX_ROBOT_PARAM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SYNTAX_ROBOT_PARAM");
        }
        else if(s_nErrCode == SYS_ERR_SYNTAX_AXIS_PARAM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SYNTAX_AXIS_PARAM");
        }
        else if(s_nErrCode == SYS_ERR_SYNTAX_MOTOR_PARAM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SYNTAX_MOTOR_PARAM");
        }
        else if(s_nErrCode == SYS_ERR_SYNTAX_WELDER_PARAM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SYNTAX_WELDER_PARAM");
        }
        else if(s_nErrCode == SYS_ERR_SYNTAX_USER_PARAM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SYNTAX_USER_PARAM");
        }
        else if(s_nErrCode == SYS_ERR_OPEN_RESTART_PARAM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "OPEN_RESTART_PARAM");
        }
        else if(s_nErrCode == SYS_ERR_FINALIZE_RM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_FINALIZE_RM");
        }
        else if(s_nErrCode == SYS_ERR_FINALIZE_TE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_FINALIZE_TE");
        }
        else if(s_nErrCode == SYS_ERR_FINALIZE_SC)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_FINALIZE_SC");
        }
        else if(s_nErrCode == SYS_ERR_PROC_ALIVE_TE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_PROC_ALIVE_TE");
        }
        else if(s_nErrCode == SYS_ERR_PROC_ALIVE_SC)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_PROC_ALIVE_SC");
        }
        else if(s_nErrCode == SYS_ERR_SYNTAX_STATISTICS_PARAM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SYNTAX_STATISTICS");
        }
        else if(s_nErrCode == SYS_ERR_OPEN_STATISTICS_PARAM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "OPEN_STATISTICS_PARA");
        }
        else if(s_nErrCode == SYS_ERR_OPEN_CONST_VAR)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_OPEN_CONST_VAR");
        }
        else if(s_nErrCode == SVC_ERR_ERRHIST_STACK_FULL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERRHIST_STACK_FULL");
        }
        else if(s_nErrCode == SVC_ERR_CART_CYLINDER_STATE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CART_CYLINDER_STATE");
        }
        else if(s_nErrCode == SVC_ERR_ZIPFILE_UNCOMPRESS)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ZIPFILE_UNCOMPRESS");
        }
        else if(s_nErrCode == ERR_WOUT_VOLT_CALIB_PARAM_INVALID)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "VOLT_PARAM_INVALID");
        }
        else if(s_nErrCode == ERR_VOLT_PARAM_MAX_DISCRIMINANT_IMAGINARY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "[V]MAX_IMAGINARY");
        }
        else if(s_nErrCode == ERR_VOLT_PARAM_MIN_DISCRIMINANT_IMAGINARY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "[V]MIN_IMAGINARY");
        }
        else if(s_nErrCode == ERR_VOLT_PARAM_C_VALUE_LOW)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "[V]C_VALUE_LOW");
        }
        else if(s_nErrCode == ERR_VOLT_PARAM_C_VALUE_HIGH)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "[V]C_VALUE_HIGH");
        }
        else if(s_nErrCode == ERR_WOUT_CURR_CALIB_PARAM_INVALID)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CURR_PARAM_INVALID");
        }
        else if(s_nErrCode == ERR_CURR_PARAM_MAX_DISCRIMINANT_IMAGINARY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "[C]MAX_IMAGINARY");
        }
        else if(s_nErrCode == ERR_CURR_PARAM_MIN_DISCRIMINANT_IMAGINARY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "[C]MIN_IMAGINARY");
        }
        else if(s_nErrCode == ERR_CURR_PARAM_C_VALUE_LOW)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "[C]C_VALUE_LOW");
        }
        else if(s_nErrCode == ERR_CURR_PARAM_C_VALUE_HIGH)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "[C]C_VALUE_HIGH");
        }
        else if(s_nErrCode == ERR_NOT_VALID_MATRIX_DIMENSION)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVAL_MATRIX_DIM");
        }
        else if(s_nErrCode == ERR_SOVLE_INVERSE_MATRIX)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SOLVE_INVMATRIX");
        }
        else if(s_nErrCode == ERR_VOLT_RATIO_MEASURED_AND_CALCULATED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "VOLT_CAL_RATIO");
        }
        else if(s_nErrCode == ERR_CURR_RATIO_MEASURED_AND_CALCULATED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CURR_CAL_RATIO");
        }
        
        /* Ecat Error */
        else if(s_nErrCode == ECAT_ERR_INIT_ECAT_FAIL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_INIT_ECAT_FAIL");
        }
        else if(s_nErrCode == ECAT_ERR_CHANGE_OP_MODE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_CHANGE_OPMODE");
        }
        else if(s_nErrCode == ECAT_ERR_LOAD_XML_CONFIG)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_LOAD_XML");
        }


        /* Service Error */
        else if(s_nErrCode == SVC_ERR_USER_PARAM_SAVING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "USER_PARAM_SAVE");
        }
        else if(s_nErrCode == SVC_ERR_RESTART_PARAM_SAVING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "RESTART_PARAM_SAVE");
        }
        else if(s_nErrCode == SVC_ERR_SENSDATA_SAVING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SENSDATA_SAVE");
        }
        else if(s_nErrCode == SVC_ERR_ABS_ENC_RESET)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_ABS_ENC_RESET");
        }
        else if(s_nErrCode == SVC_ERR_SET_ZERO_POSITION)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SET_ZERO_POS");
        }
        else if(s_nErrCode == SVC_ERR_POS_DATA_SAVING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_POS_DATA_SAVE");
        }
        else if(s_nErrCode == SVC_ERR_STATISTICS_DATA_SAVING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "STATISTICS_SAVE");
        }
        else if(s_nErrCode == SVC_ERR_GET_STATISTICS_DATA)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "GET_STATISTICS");
        }
        else if(s_nErrCode == SVC_ERR_CONST_VAR_SAVING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CONST_VAR_SAVING");
        }
        else if(s_nErrCode == SVC_ERR_BLANK_JOB_OPEN)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "BLANK_JOB_OPEN");
        }
#if 0
        else if(s_nErrCode == SVC_ERR_JOB_COMPILE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_COMPILE");
        }
#endif
        else if(s_nErrCode == SVC_ERR_JOBCMD_PROC)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOBCMD_PROC");
        }
        else if(s_nErrCode == SVC_ERR_SERVO_OFF)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SERVO_OFF");
        }
        else if(s_nErrCode == SVC_ERR_JOBDATA_COMM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOBDATA_COMM");
        }
        else if(s_nErrCode == SVC_ERR_JOB_MON)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_MON");
        }
        else if(s_nErrCode == SVC_ERR_JOB_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_JOB_SAVE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_SAVE");
        }
        else if(s_nErrCode == SVC_ERR_JOB_ALREADY_EXEC)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOB_ALREADY_EXEC");
        }
        else if(s_nErrCode == SVC_ERR_JOBEXE_TE_REACT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_TE_JOBEXE");
        }
        else if(s_nErrCode == SVC_ERR_NOT_READY_RESTART)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NOT_READY_RESTART");
        }
#if 0
        else if(s_nErrCode == SVC_ERR_CONF_RELOAD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_CONF_RELOAD");
        }
#endif
        else if(s_nErrCode == SVC_ERR_JOBEXECAUTO)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOBEXEC_AUTO");
        }
        else if(s_nErrCode == SVC_ERR_JOBEXECDRY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOBEXEC_DRY");
        }
        else if(s_nErrCode == SVC_ERR_JOBSTOP)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_STOP");
        }
        else if(s_nErrCode == SVC_ERR_JOBEXECSTEP)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOBEXEC_STEP");
        }
        else if(s_nErrCode == SVC_ERR_HOMEPOS_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_HOMEPOS_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_USRCRD_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_USRCRD_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_TCP_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_TCP_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_WORLD_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_WORLD_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_WELDIN_TUNE_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WELDIN_TUNE_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_WELDOUT_TUNE_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WELDOUT_TUNE_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_RESTART_PARAM_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "RESTART_PARAM_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_ARCSENS_PARAM_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ARCSENS_PARAM_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_GAP_COND_SET)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_GAP_COND_SET");
        }
        else if(s_nErrCode == SVC_ERR_CONST_VAR_EDIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_CONST_VAR_EDIT");
        }
        else if(s_nErrCode == SVC_ERR_JOB_SNDHEADER)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_SNDHEADER");
        }
        else if(s_nErrCode == SVC_ERR_JOB_SNDBODY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_SNDBODY");
        }
        else if(s_nErrCode == SVC_ERR_JOB_RCVHEADER)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_RCVHEADER");
        }
        else if(s_nErrCode == SVC_ERR_JOB_RCVBODY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_RCVBODY");
        }
        else if(s_nErrCode == SVC_ERR_JOB_EXECRESTART)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_JOB_EXEC_RESTART");
        }
        else if(s_nErrCode == SVC_ERR_HOME_MOVE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_HOME_MOVE");
        }
        else if(s_nErrCode == SVC_ERR_WIRECUT_JOB_CALL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "WIRECUT_JOB_CALL");
        }

        /* I/O Error */
        else if(s_nErrCode == IO_ERR_HW_LIMIT_DETECT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_HW_LIMIT_DETECT");
        }

        /* Job Compile Error */
        else if(s_nErrCode == JOBASM_ERR_INTERNAL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_INTERNAL");
        }
        else if(s_nErrCode == JOBASM_ERR_ASSEMBLER)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ASSEM_HANDLE");
        }
        else if(s_nErrCode == JOBASM_ERR_SYNTAX)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_SYNTAX");
        }
        else if(s_nErrCode == JOBASM_ERR_UNSUPPORTED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_UNSUPPORT");
        }
        else if(s_nErrCode == JOBASM_ERR_TYPE_CONSTANT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_TYPE_CONSTANT");
        }
        else if(s_nErrCode == JOBASM_ERR_TYPE_MISMATCH)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_TYPE_MISMATCH");
        }
        else if(s_nErrCode == JOBASM_ERR_SYM_RESERVED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SYM_RESERVED");
        }
        else if(s_nErrCode == JOBASM_ERR_SYM_NAME)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_SYM_NAME");
        }
        else if(s_nErrCode == JOBASM_ERR_SYM_DUPLICATE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SYM_DUPLICATE");
        }
        else if(s_nErrCode == JOBASM_ERR_SYM_MISSING_VAL)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SYM_MISSING_VAL");
        }
        else if(s_nErrCode == JOBASM_ERR_SYM_INVALID)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_SYM_INVALID");
        }
        else if(s_nErrCode == JOBASM_ERR_SYM_UNDEF)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_SYM_UNDEF");
        }
        else if(s_nErrCode == JOBASM_ERR_SYM_UNLINK)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_SYM_UNLINK");
        }
        else if(s_nErrCode == JOBASM_ERR_SYM_CROSS)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_SYM_CROSS");
        }
        else if(s_nErrCode == JOBASM_ERR_FILE_BROKEN)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_FILE_BROKEN");
        }
        else if(s_nErrCode == JOBASM_ERR_FILE_SEEK)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_FILE_SEEK");
        }
        else if(s_nErrCode == JOBASM_ERR_NO_SOURCE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_NO_SOURCE");
        }
        else if(s_nErrCode == JOBASM_ERR_TOO_MANY_SOURCE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "TOO_MANY_SOURCE");
        }
        else if(s_nErrCode == JOBASM_ERR_UNKNOWN_CMD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_UNKNOWN_CMD");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_CMD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_INVALID_CMD");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_POS)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_INVALID_POS");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_TARGET)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_TARGET");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_VIAWAY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_VIAWAY");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_VELOCITY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_VELOCITY");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_TIME)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_TIME");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_WEAV)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_WEAV");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_WEAV_EXPR)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_WEAV_EXPR");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_WELD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_WELD");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_SWF)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_SWF");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_MWF)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_MWF");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_EWF)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_EWF");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_WELD_EXPR)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_WELD_EXPR");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_VAR)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_VAR");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_ARG)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_ARG");
        }
        else if(s_nErrCode == JOBASM_ERR_INVALID_COORD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "INVALID_COORD");
        }
        else if(s_nErrCode == JOBASM_ERR_MISSING_OBJECT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "MISSING_OBJECT");
        }
        else if(s_nErrCode == JOBASM_ERR_BRANCH_RANGE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "BRANCH_RANGE");
        }
#if 0
        else if(s_nErrCode == JOBASM_ERR_EOF)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_EOF");
        }
#endif
        else if(s_nErrCode == JOBASM_ERR_MEMORY_EXHAUST)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "MEMORY_EXHAUST");
        }
        else if(s_nErrCode == JOBASM_ERR_PATH_NAME_TOO_LONG)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "PATH_NAME_TOO_LONG");
        }
        else if(s_nErrCode == JOBASM_ERR_FILE_NAME_TOO_LONG)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "FILE_NAME_TOO_LONG");
        }
        else if(s_nErrCode == JOBASM_ERR_FILE_OPEN)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_FILE_OPEN");
        }
#if 0
        else if(s_nErrCode == JOBASM_ERR_FILE_INCLUDE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "JOBASM_ERR_FILE_INCLUDE");
        }
#endif

        /* ServoPack Error */
        else if(s_nErrCode == SERVO_ERR_PARAMETER_CHECKSUM_1)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_PARAM_CHECKSUM");
        }
        else if(s_nErrCode == SERVO_ERR_PARAMETER_FORMAT_1)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_PARAM_FORMAT");
        }
        else if(s_nErrCode == SERVO_ERR_SYSTEM_CHECKSUM_1)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SYS_CHECKSUM");
        }
        else if(s_nErrCode == SERVO_ERR_MAIN_CIRCUIT_DECTECTOR_1)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "MAIN_CIRCUIT_DETECT");
        }
        else if(s_nErrCode == SERVO_ERR_PARAMETER_SETTING_1)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_PARAM_SET");
        }
        else if(s_nErrCode == SERVO_ERR_ENCODER_OUTPULSE_SETTING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_ENC_OUTPULSE");
        }
        else if(s_nErrCode == SERVO_ERR_PARAMETER_COMBINATION)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_PARAM_COMB");
        }
        else if(s_nErrCode == SERVO_ERR_SEMICLOSED_FULLCLOSED_CONTROL_PARAMETER_SETTING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SEMI_CONT_PARAM_SET");
        }
        else if(s_nErrCode == SERVO_ERR_COMBINATION)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_COMB");
        }
        else if(s_nErrCode == SERVO_ERR_UNSUPPORTED_DEVICE_ALARM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "UNSUPPORTED_DEV");
        }
        else if(s_nErrCode == SERVO_ERR_CANCELLED_SERVOON_CMD_ALARM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CANCELLED_SRV_CMD");
        }
        else if(s_nErrCode == SERVO_ERR_OVERCURRENT_HEATSINK_OVERHEATED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "HEATSINK_OVERHEATED");
        }
        else if(s_nErrCode == SERVO_ERR_REGENERATION)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_REGENERATION");
        }
        else if(s_nErrCode == SERVO_ERR_REGENERATIVE_OVERLOAD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "REGENERT_OVERLOAD");
        }
        else if(s_nErrCode == SERVO_ERR_MAIN_CIRCUIT_PWR_SUPPLY_WIRING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "MAIN_PWR_WIRING");
        }
        else if(s_nErrCode == SERVO_ERR_OVERVOLTAGE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "OVERVOLTAGE");
        }
        else if(s_nErrCode == SERVO_ERR_UNDERVOLTAGE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "UNDERVOLTAGE");
        }
        else if(s_nErrCode == SERVO_ERR_MAIN_CIRCUIT_CAPACITOR_OVERVOLTAGE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CAPACITOR_OVERVOLT");
        }
        else if(s_nErrCode == SERVO_ERR_OVERSPEED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_OVERSPEED");
        }
        else if(s_nErrCode == SERVO_ERR_OVERSPEED_ENCODER_OUTPUT_PULSE_RATE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "OVERSPEED_ENC_OUT");
        }
        else if(s_nErrCode == SERVO_ERR_VIBRATION_ALARM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "VIBRATION");
        }
        else if(s_nErrCode == SERVO_ERR_AUTOTUNING_ALARM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "AUTOTUNING");
        }
        else if(s_nErrCode == SERVO_ERR_OVERLOAD_HIGH_LOAD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "OVERLOAD_HIGHLOAD");
        }
        else if(s_nErrCode == SERVO_ERR_OVERLOAD_LOW_LOAD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "OVERLOAD_LOWLOAD");
        }
        else if(s_nErrCode == SERVO_ERR_DYNAMIC_BRAKE_OVERLOAD1)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "DYN_BRK_OVERLOAD1");
        }
        else if(s_nErrCode == SERVO_ERR_DYNAMIC_BRAKE_OVERLOAD2)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "DYN_BRK_OVERLOAD2");
        }
        else if(s_nErrCode == SERVO_ERR_OVERLOAD_SURGE_CURRENT_LIMIT_RESISTOR)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SURGE_CUR_LIMIT");
        }
        else if(s_nErrCode == SERVO_ERR_HEATSINK_OVERHEATED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "HEATSINK_OVERHEAT");
        }
        else if(s_nErrCode == SERVO_ERR_BUILT_IN_FAN_STOPPED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "FAN_STOPPED");
        }
        else if(s_nErrCode == SERVO_ERR_ENCODER_BACKUP)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_BACKUP");
        }
         else if(s_nErrCode == SERVO_ERR_ENCODER_CHECKSUM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_CHECKSUM");
        }
        else if(s_nErrCode == SERVO_ERR_ABS_ENCODER_BATTERY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ABS_ENC_BATTERY");
        }
        else if(s_nErrCode == SERVO_ERR_ENCODER_DATA)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_DATA");
        }
        else if(s_nErrCode == SERVO_ERR_ENCODER_OVER_SPEED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_OVER_SPD");
        }
         else if(s_nErrCode == SERVO_ERR_ENCODER_OVER_HEATED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_OVERHEATED");
        }
        else if(s_nErrCode == SERVO_ERR_CURRENT_DETECTION_1_PHASE_U)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CUR_DETECT_U");
        }
        else if(s_nErrCode == SERVO_ERR_CURRENT_DETECTION_2_PHASE_V)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CUR_DETECT_V");
        }
        else if(s_nErrCode == SERVO_ERR_CURRENT_DETECTION_3_CURRENT_DETECTOR)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CUR_DETECT_DETECTOR");
        }
        else if(s_nErrCode == SERVO_ERR_SYSTEM_ALARM_0)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SYS_ALARM0");
        }
        else if(s_nErrCode == SERVO_ERR_SYSTEM_ALARM_1)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SYS_ALARM1");
        }
        else if(s_nErrCode == SERVO_ERR_SYSTEM_ALARM_2)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SYS_ALARM2");
        }
        else if(s_nErrCode == SERVO_ERR_SYSTEM_ALARM_3)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SYS_ALARM3");
        }
        else if(s_nErrCode == SERVO_ERR_SYSTEM_ALARM_4)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ERR_SYS_ALARM4");
        }
        else if(s_nErrCode == SERVO_ERR_OVERRUN_DETECTED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "OVERRUN_DETECTED");
        }
        else if(s_nErrCode == SERVO_ERR_ABS_ENCODER_CLEAR_AND_MULTITURN_LIMIT_SET)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ABS_ENC_CLR&MULT_LMT");
        }
        else if(s_nErrCode == SERVO_ERR_ENCODER_COMMUNICATION)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_COMMUNICATION");
        }
        else if(s_nErrCode == SERVO_ERR_ENCODER_COMMUNICATION_POSITION_DATA)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_COMM_POSDATA");
        }
        else if(s_nErrCode == SERVO_ERR_ENCODER_COMMUNICATION_TIMER)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_COMM_TIMER");
        }
        else if(s_nErrCode == SERVO_ERR_ENCODER_PARAMETER)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_PARAM");
        }
        else if(s_nErrCode == SERVO_ERR_ENCODER_ECHOBACK)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ENC_ECHOBACK");
        }
         else if(s_nErrCode == SERVO_ERR_MULTI_TURN_LIMIT_DISAGREEMENT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "MULTTURN_LMT_DISAGR");
        }
        else if(s_nErrCode == SERVO_ERR_POSITION_ERR_PULSE_OVERFLOW)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "POS_ERR_OVERFLOW");
        }
        else if(s_nErrCode == SERVO_ERR_POSITION_ERR_PULSE_OVERFLOW_ALARM_AT_SERVOON)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "POS_ERR_OVERFLOW");
        }
        else if(s_nErrCode == SERVO_ERR_POSITION_ERR_PULSE_OVERFLOW_ALARM_BY_SPEEDLIMIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "POS_ERR_OVERFLOW");
        }
         else if(s_nErrCode == SERVO_ERR_POSITION_DATA_OVERFLOW)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "POSDATA_OVERFLOW");
        }
        else if(s_nErrCode == SERVO_ERR_CMD_OPTION_MODULE_IF_INIT_TIMEOUT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_MOD_INT_TIMEOUT");
        }
        else if(s_nErrCode == SERVO_ERR_CMD_OPTION_MODULE_IF_SYNC)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_MOD_SYNC");
        }
        else if(s_nErrCode == SERVO_ERR_CMD_OPTION_MODULE_IF_COMMUNICATION_DATA)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_MOD_COMM");
        }
         else if(s_nErrCode == SERVO_ERR_CMD_OPTION_MODULE_DETECTION_FAIL_ALARM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_MOD_DECT_FAIL");
        }
        else if(s_nErrCode == SERVO_ERR_SAFETY_OPTION_MODULE_DETECTION_FAIL_ALARM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SFT_MOD_DECT_FAIL");
        }
        else if(s_nErrCode == SERVO_ERR_UNSUPPORTED_CMD_OPTION_MODULE_DETECTION_FAIL_ALARM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_MOD_DECT_FAIL");
        }
        else if(s_nErrCode == SERVO_ERR_UNSUPPORTED_SAFETY_OPTION_MODULE_DETECTION_FAIL_ALARM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SFT_MOD_DECT_FAIL");
        }
        else if(s_nErrCode == SERVO_ERR_CMD_OPTION_MODULE_DETECTION_DISAGREE_ALARM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_MOD_DISAGR_ALARM");
        }
        else if(s_nErrCode == SERVO_ERR_SAFETY_DEVICE_SIGNAL_INPUT_TIMING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "SFT_DEV_SIG_TIMING");
        }
        else if(s_nErrCode == SERVO_ERR_MAIN_CIRCUIT_CABLE_OPEN_PHASE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "MAIN_CIR_CABLE_OPEN");
        }

        /* ServoPack Warning */
        else if(s_nErrCode == SERVO_WARN_POSITION_ERR_PULSE_OVERFLOW)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "W>POS_ERR_OVERFLOW");
        }
        else if(s_nErrCode == SERVO_WARN_POSITION_ERR_PULSE_OVERFLOW_ALARM_AT_SERVOON)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "W>POS_ERR_OVERFLOW");
        }
        else if(s_nErrCode == SERVO_WARN_OVERLOAD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "W>OVERLOAD");
        }
        else if(s_nErrCode == SERVO_WARN_VIBRATION)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "W>VIBRATION");
        }
        else if(s_nErrCode == SERVO_WARN_REGENERATIVE_OVERLOAD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "W>REGENERT_OVERLOAD");
        }
        else if(s_nErrCode == SERVO_WARN_DYNAMIC_BRAKE_OVERLOAD)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "W>DYN_BRK_OVERLOAD");
        }
        else if(s_nErrCode == SERVO_WARN_ABS_ENCODER_BATTERY)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "W>ABS_ENC_BATTERY");
        }
        else if(s_nErrCode == SERVO_WARN_UNDERVOLTAGE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "W>UNDER_VOLTAGE");
        }

        /* EtherCAT Error */
        else if(s_nErrCode == ECAT_ERR_CMD_OPTION_IF_SERVO_UNIT_INIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_OPT_INIT");
        }
        else if(s_nErrCode == ECAT_ERR_CMD_OPTION_IF_MEMORY_CHECK)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_OPT_MEMCHECK");
        }
        else if(s_nErrCode == ECAT_ERR_CMD_OPTION_IF_SERVO_SYNC)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_OPT_SYNC");
        }
        else if(s_nErrCode == ECAT_ERR_CMD_OPTION_IF_SERVO_DATA)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "CMD_OPT_DATA");
        }
        else if(s_nErrCode == ECAT_ERR_ECAT_DC_SYNC)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ECAT_DC_SYNC");
        }
        else if(s_nErrCode == ECAT_ERR_ECAT_STATE)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ECAT_STATE");
        }
        else if(s_nErrCode == ECAT_ERR_ECAT_OUTPUT_DATA_SYNC)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ECAT_OUTDATA_SYNC");
        }
        else if(s_nErrCode == ECAT_ERR_PARAMETER_SETTING)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ECAT_PARAM_SET");
        }
        else if(s_nErrCode == ECAT_ERR_SYSTEM_INIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ECAT_SYSINIT");
        }
        else if(s_nErrCode == ECAT_ERR_COMMUNICATION_DEVICE_INIT)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ECAT_COMM_DEVICE");
        }
        else if(s_nErrCode == ECAT_ERR_LOADING_SERVO_INFORM)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ECAT_LOAD_SRVINF");
        }
        else if(s_nErrCode == ECAT_ERR_EEPROM_PARAMETER_DATA)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ECAT_PARAM_DATA");
        }
        else if(s_nErrCode == ECAT_ERR_SLAVE_CONNECTION_CLOSED)
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "ECAT_SLAVE_CONN");
        }

        /* Not Defined Error */
        else
        {
            CRT_strcpy(g_szErrContent, ERROR_NAME_LEN, "NOT DEFINED ERROR");
        }

        /* Error Mode */
        if(s_nErrMode == 0x00)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "       ");
        }
        else if(s_nErrMode == ERR_MOD_CONF_RELOAD)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "CONF");
        }
        else if(s_nErrMode == ERR_MOD_JOB_COMPILE)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "COMPILE");
        }
        else if(s_nErrMode == ERR_MOD_PARAM_VALIDCHECK)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "PRAMVAL");
        }
        else if(s_nErrMode == ERR_MOD_PARAM_EDIT)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "PRAMEDT");
        }
        else if(s_nErrMode == ERR_MOD_SYSTEM)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "SYSTEM");
        }
        else if(s_nErrMode == ERR_MOD_FILE)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "FILE");
        }
        else if(s_nErrMode == ERR_MOD_JOB_EXEC)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "JOBEXEC");
        }

        else if(s_nErrMode == ERR_MOD_FROM_AXIS1)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "AXIS1");
        }
        else if(s_nErrMode == ERR_MOD_FROM_AXIS2)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "AXIS2");
        }
        else if(s_nErrMode == ERR_MOD_FROM_AXIS3)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "AXIS3");
        }
        else if(s_nErrMode == ERR_MOD_FROM_AXIS4)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "AXIS4");
        }
        else if(s_nErrMode == ERR_MOD_FROM_AXIS5)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "AXIS5");
        }
        else if(s_nErrMode == ERR_MOD_FROM_AXIS6)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "AXIS6");
        }
        else if(s_nErrMode == ERR_MOD_FROM_NOTAXIS)
        {
            CRT_strcpy(g_szErrModeContent, ERROR_MODE_NAME_LEN, "NOTAXIS");
        }
    }

    return 0;
}


///////////////////////////////////////
//
//  Function: SYSMON_ParceEstopCode()
//      - Service Name: RMGR_SERV_SYSSTATE

int SYSMON_ParceEstopCode(int nEstopCode)
{
    if(nEstopCode == SYS_ESTOP_NONE)
    {
        CRT_strcpy(g_szEstopContent, ESTOP_NAME_LEN, "ESTOP_NONE");
    }
    else if(nEstopCode == SYS_ESTOP_TP)
    {
        CRT_strcpy(g_szEstopContent, ESTOP_NAME_LEN, "ESTOP_TP");
    }
    else if(nEstopCode == SYS_ESTOP_SERV)
    {
        CRT_strcpy(g_szEstopContent, ESTOP_NAME_LEN, "ESTOP_SERV");
    }   
    else if(nEstopCode == SYS_ESTOP_SHOCKSENSOR)
    {
        CRT_strcpy(g_szEstopContent, ESTOP_NAME_LEN, "SHOCKSEN");
    } 
    else if(nEstopCode == SYS_ESTOP_CONTROLBOX)
    {
        CRT_strcpy(g_szEstopContent, ESTOP_NAME_LEN, "CONTROLBOX");
    } 
    else if(nEstopCode == SYS_ESTOP_CART)
    {
        CRT_strcpy(g_szEstopContent, ESTOP_NAME_LEN, "ESTOP_CART");
    } 
    else if(nEstopCode == SYS_ESTOP_HWLIMIT_ACT)
    {
        CRT_strcpy(g_szEstopContent, ESTOP_NAME_LEN, "HWLIMIT ON");
    }
    else if(nEstopCode <= 0)
    {
        CRT_strcpy(g_szEstopContent, ESTOP_NAME_LEN, "NULL");
    }

    return 0;
}
