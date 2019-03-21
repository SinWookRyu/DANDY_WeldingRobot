#include "taskexec_def.h"
#include "dandy_job.h"
#include "arg.h"
#include "coord.h"

extern int PosVelFromArg(double* vel, const ARG_VALUE* arg_spd); 
extern int DistFromArg(double* dist, const ARG_VALUE* arg_dist); 

#define TCH_STEP_NONE       (0)
#define TCH_STEP_WAIT       (2)
#define TCH_STEP_MOVE       (3)
#define TCH_STEP_SENSOR_OFF (4)

static int s_step;      // running step
static int s_stop;      // stop requested
static int s_index;       // command index
static int s_touched;   // touched 
static int s_sample;    // elapse sampling
static int s_notouch_break; // no touched break flag
static DBL s_start[6];  // start joint 

static VOD ExitProc(void); 
static int MovePlan(int index, double jnt_start[6]); 
static int PosSave(int index);

////////////////////////////////////////////////////////////////////////////////

int CmdSet_TOUCH(int i_cmd, const double start[6])
{
    DANDY_JOB_ARG_TOUCH* arg; 

    WELD_SET_TCH_MC(ON); 
    WELD_SET_TCH_PROC(ON); 

    s_sample = 0; 
  
    // No Touch Break Flag    
    arg = &g_rcmd[i_cmd].arg.argTouch; 
    s_notouch_break = !(arg->nOnFail);  // onfail == 1 -> continue
    
    // Control Var's Setting 
    s_step = TCH_STEP_WAIT; 
    s_stop = 0; 
    s_index = i_cmd; 
    s_touched = FALSE;     
    memcpy(s_start, start, sizeof(s_start)); 
    return 0; 
}

VOD CmdStp_TOUCH(int f_fast)
{
    s_stop = 1; 
    Traj_Stop(&g_traj, f_fast); 
}

int CmdUpd_TOUCH(void)
{   
    int ret; 

    if(s_step == TCH_STEP_NONE)
    {
    	ExitProc();
        return -1;
    }        

    // time up    

    // out condition check 
    if(!Traj_IsRunning(&g_traj) && (s_stop))
    {
        ExitProc();         
        return -1; 
    }
     
    switch(s_step)
    {
    case TCH_STEP_WAIT:
        // time up
        s_sample++; 

        // check of wait time end 
        if(s_sample * g_t_samp < WAITTIME_IO_CHANGE)
        {
            return 0; 
        }

        if(s_stop)
        {
            ExitProc(); 
            return -1; 
        }

        // wait time end ///////////////////////////////////////////////////////

        // Check Already Touched. 
        if(WELD_GET_TCH_SIG == ON)
        {
            ERROR_SET(SECT_CMD_TOUCH, ERR_TOUCH_ALREADY); 
            ExitProc();             
            return -1; 
        }

        if(WELD_GET_TCH_PROC == OFF)
        {
            ERROR_SET(SECT_CMD_TOUCH, ERR_TCH_NOT_READY); 
            ExitProc();             
            return -1; 
        }

///Change variable name After sc is modified. //////////////////////////////////        
        ret = MovePlan(s_index, s_start); 
        if(ret)
        {
            ERROR_SET(SECT_CMD_TOUCH, ret);
        	ExitProc();
            return -1; 
        }

        s_step = TCH_STEP_MOVE; 
        return 0; 

    case TCH_STEP_MOVE:

        if(!Traj_Update(&g_traj))
        {
            ERROR_SET(SECT_CMD_TOUCH, ERRCODE_TRAJ(g_traj.error)); 

            if(WELD_GET_TCH_SIG == ON)
            {                   
                PosSave(s_index); 

                // Sets after success of touching 
                Traj_Stop_Time(&g_traj, g_dec_time_touch);                 
                WELD_SET_TCH_PROC(OFF); 
                s_touched = TRUE; 
            }
            return 0; 
        }
        
        // Motion End    

        ERROR_SET(SECT_CMD_TOUCH, ERRCODE_TRAJ(g_traj.error)); 

        // touch fail check
        if(s_touched == FALSE && s_notouch_break && s_stop == FALSE)
        {
            ERROR_SET(SECT_CMD_TOUCH, ERR_TOUCH_FAIL); 
            ExitProc();             
            return -1;
        }
        ExitProc();
        return -1; 
        
    default:
    	ExitProc();
        return -1; 
    }
}
    
static void ExitProc(void)
{
    s_step = TCH_STEP_NONE; 
    WELD_SET_TCH_PROC(OFF); 
    VERBOSE_MESSAGE("Touch Exits & Touch-Off Dout Enable\n");
}


static int MovePlan(int index, double jnt_start[6])
{
    DANDY_JOB_ARG_TOUCH* arg; 
    TRANS bTc;   // Start & Final TCP wrt BASE
    POS s_p, s_r; 
    DBL pos; 
    DBL v_p; 
    DBL lim; 
    int coord; 
    int ret; 

    // Get Cmd Arg
    arg = &g_rcmd[index].arg.argTouch; 
    
    // Get Ref Coord 
    ret = Arg_Scalar(&coord, NULL, &arg->valCoord); 
    if(ret)
    {
        return ret; 
    }

    // Pos Dir    
    s_p.x = 0; 
    s_p.y = 0; 
    s_p.z = 0;    

    ret |= Arg_Scalar(NULL, &s_p.x, &arg->valIncX); 
    ret |= Arg_Scalar(NULL, &s_p.y, &arg->valIncY); 
    ret |= Arg_Scalar(NULL, &s_p.z, &arg->valIncZ); 

    pos = POS_Norm(s_p);
    if(POS_Unit(&s_p))
    {
        return ERR_INVALID_DIR; 
    }

    // POS wrt BASE
    bTc = TargetCoordWrtBase(coord);  // base_T_coord
    s_p = ROT_Multi_POS(bTc.R, s_p); 

    // vel_pos
    ret = PosVelFromArg(&v_p, &arg->valSpeed); 
    if(ret)
    {
        return ret; 
    }

    // dist
    ret = DistFromArg(&lim, &arg->valLimit); 
    if(ret)
    {
        return ret; 
    }

    // tamporary rotation info's        
    s_r.x = 0; 
    s_r.y = 0; 
    s_r.z = 1.; 

#if 0 // PRINT
    printf("Tch Mov Plan 'g_pos_act' : %f %f %f\n", g_pos_act[0], g_pos_act[1], g_pos_act[2]); 
    printf("Tch Mov Plan 'jnt_start' : %f %f %f\n", jnt_start[0], jnt_start[1], jnt_start[2]); 
#endif 
#if 0
    ret = Traj_Set_Lmot(&g_traj, jnt_start, 
                         &s_p, lim, v_p, 
                         &s_r, 0, 0, 
                         g_traj.Config(g_pos_act, g_traj.dh)); 
#else    
    ret = Traj_Set_Lmot(&g_traj, jnt_start, 
                         &s_p, lim, v_p, 
                         &s_r, 0, 0, 
                         g_traj.Config(jnt_start, g_traj.dh)); 
#endif

    return ERRCODE_TRAJ(ret); 
}

static int PosSave(int index)
{    
    ROBOT_POS rpos, rpos_temp; 
    int coord; 
    int ret; 
 
    // Get Ref Coord 
    ret = Arg_Scalar(&coord, NULL, &g_rcmd[s_index].arg.argTouch.valCoord); 
    if(ret)
    {
        return ret; 
    }

    if(coord == COORDINDEX_JOINT)
    {
#if 0 // TOUCH
        JOINT_2_ROBPOS(&rpos, g_pos_trg); 
#else
        JOINT_2_ROBPOS(&rpos, g_pos_act); 
#endif
        Rad2Deg(rpos.pos, 6); 
        R_VERB_VRB("Touched. Jnt[deg] : %.2f %.2f %.2f %.2f %.2f %.2f\n", 
            g_pos_act[0]*180./PI, g_pos_act[1]*180./PI, g_pos_act[2]*180./PI, 
            g_pos_act[3]*180./PI, g_pos_act[4]*180./PI, g_pos_act[5]*180./PI);          
    }
    else
    {        
#if 0 // TOUCH
        TRANS bTt, cTb, cTt; 
        // Change Coord cTt = cTb * bTt
        bTt = TRANS_Multi_TRANS(g_bTe, g_eTt); 
        cTb = BaseWrtTargetCoord(coord); 
        cTt = TRANS_Multi_TRANS(cTb, bTt); 
#else 
        TRANS cTb, cTt; 
        // Change Coord cTt = cTb * bTt
        cTb = ActualCoordWrtBase(coord); 
        cTt = TRANS_Multi_TRANS(cTb, g_bTt_act); 
#endif 
        TRANS_2_ROBPOS(&rpos, &cTt);   

        R_VERB_VRB("Touched. Cart[mm] : %.2f %.2f %.2f wrt %s\n", 
            g_bTt_act.p.x,  g_bTt_act.p.y, g_bTt_act.p.z, Str_Coord(coord)); 
    }
    
    ret = Arg_Set_RobPos(&g_rcmd[s_index].arg.argTouch.valSavePos, &rpos); 
    if(!ret)
    {
        // debug print
        Arg_RobPos(&rpos_temp, &g_rcmd[s_index].arg.argTouch.valSavePos);
        R_VERB_VRB("Saved Value : %.2f %.2f %.2f %.2f %.2f %.2f, Conf:0x%x\n", 
            rpos_temp.pos[0], rpos_temp.pos[1], rpos_temp.pos[2], 
            rpos_temp.pos[3], rpos_temp.pos[4], rpos_temp.pos[5], 
            rpos_temp.nConfig); 
    }
    return ret; 

    
}
