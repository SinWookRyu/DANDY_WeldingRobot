#include "taskexec_def.h"
#include "call_stack.h"

static void VgaDisplay(void); 

THREAD_ENTRY_TYPE VgaThread(void* pParam)
{
    while(1)
    {        
        if(!g_f_runtime_pause)
        {
            VgaDisplay(); 
            THREAD_Sleep(VGA_SAMPLING); 
        }
        
        if(g_f_vga_exit)
        {
            goto EXIT_PROCESS; 
        }    
    }

EXIT_PROCESS:
    g_f_vga_exit = FALSE; 
    printf("VGA Thread exits\n"); 
    return 0; 
}

#if 0
extern double g_temp_d; 
extern double g_temp_ud; 
extern double g_temp_tu; 
extern double g_temp_t; 
#endif

// VGA 
static void VgaDisplay(void)
{
    char    str[8]; 
    UNT     node; 
    
#if 0
    // check vga off option
    if(g_f_vga_on == FALSE)
    {
    	return;
    }
#else
    // check vga off option
    if(g_print_off & BIT_VGA_OFF)
    {
    	return;
    }
#endif
   
    if(g_run_tick & 0x80)
    {
        CRT_strcpy(str, sizeof(str), "    "); 
    }            
    else 
    {
        CRT_strcpy(str, sizeof(str), Str_RunMode(g_mode));        
    }

    VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_YELLOW, VGA_COLOR_RED));
    
    VGA_printf(0, g_i_vga+0, "%s ERR:%04x EST:%2s SRV:%2s FIL:%2s Arc:%s TchSen:%s TchSig:%s NoPwr:%s NoGas:%s ",
        str, g_error.code, Str_OnOff(g_estop), Str_OnOff(g_servo), Str_OnOff(g_f_file), 
        Str_OnOff(WELD_GET_ARC),     Str_OnOff(WELD_GET_TCH_PROC), 
        Str_OnOff(WELD_GET_TCH_SIG), Str_OnOff(WELD_GET_PWR_FAIL), 
        Str_OnOff(WELD_GET_NO_GAS));
    
    VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_YELLOW, VGA_COLOR_DARKGRAY));

    VGA_printf(0, g_i_vga+1, "JntT:%07.2f %07.2f %07.2f %07.2f %07.2f %07.2f[mm ] Ts:%07llu(%04x) ",
            g_pos_trg[0]*180./PI, g_pos_trg[1]*180./PI, g_pos_trg[2]*180./PI,
    		g_pos_trg[3]*180./PI, g_pos_trg[4]*180./PI, g_pos_trg[5]*180./PI, 	
            g_time_diff, g_n_samp_over&0xffff);    

    VGA_printf(0, g_i_vga+2, "JntA:%07.2f %07.2f %07.2f %07.2f %07.2f %07.2f[deg] Te:%08lld(%08lld) ",
            g_pos_act[0]*180./PI, g_pos_act[1]*180./PI, g_pos_act[2]*180./PI,
    		g_pos_act[3]*180./PI, g_pos_act[4]*180./PI, g_pos_act[5]*180./PI, 	
            g_time_err, g_time_err_max);     		

    VGA_printf(0, g_i_vga+3, "CrtT:%07.2f %07.2f %07.2f %07.2f %07.2f %07.2f[%s] %07llu(%04x) %07llu ",
            g_xyzrpy.x,             g_xyzrpy.y,             g_xyzrpy.z,
            g_xyzrpy.roll*180./PI,  g_xyzrpy.pitch*180./PI, g_xyzrpy.yaw*180./PI, 
            Str_Coord(g_coord_ref), g_time_spend, g_n_spend_over, g_time_spend_max);             
        
    VGA_printf(0, g_i_vga+4, "CrtA:%07.2f %07.2f %07.2f %07.2f %07.2f %07.2f[%s]  ",
            g_xyzrpy_act.x,            g_xyzrpy_act.y,             g_xyzrpy_act.z,
            g_xyzrpy_act.roll*180./PI, g_xyzrpy_act.pitch*180./PI, g_xyzrpy_act.yaw*180./PI, 
            Str_Coord(g_coord_ref));

    node = arc_save_num; 
    VGA_SetAttr(VGA_MAKE_ATTR(VGA_COLOR_YELLOW, VGA_COLOR_GREEN));  

    VGA_printf(58, g_i_vga+4, "dist:%05.1f RES:%2s N%03d", g_dist_work, Str_OnOff(g_restart.f_restart), g_i_prog_next); 
    
    VGA_printf(0, g_i_vga+5, "PMOD:%4s CMD:%5s(%03d) COMP:%c STAK:%d WELD:%2s WV:%d,%s Skip:%c-%c Gap:%s(%02d)  ",
            Str_ProgMode(RunServProg_ModeGet()), g_CmdStr, g_i_prog_act,             
            (g_comp_result == 0.0)? '=' : (g_comp_result >  0.0)? '>' : '<', 
            CallStack_Count(), Str_OnOff(g_f_weld), 
            g_weave.i_mot, Str_WeaveSeg(g_weave.n_seg), 
            (g_pshm_rm_sys && g_pshm_rm_sys->fLeftSkip)? 'L':'X', 
            (g_pshm_rm_sys && g_pshm_rm_sys->fRightSkip)? 'R':'X', 
            (PROGCOND_GAP_LEFT)? "LFT" : (PROGCOND_GAP_RIGHT)? "RGT" : "HOZ", g_n_shift);             

#if 0
    VGA_printf(0, g_i_vga+6, "SEN[%s] Data:%04d Nod:%03d Dy:%4.2f(r:%04.2f a:%04.2f) Dz:%4.2f(r:%06.2f a:%06.2f)     ",
        Str_OnOff(weld_idx.arc_sensor), g_n_adc_data, node,
        Delta_Z[node], Delta_T[node], 
        Mean_Moving_Ampere[node], Mean_Moving_Weight[node], 
        ref_current, ref_weight);
#endif
    VGA_printf(0, g_i_vga+6, "SEN[%s] Data:%04d Nod:%03d Dy:%4.2f(r:%04.2f j:%04.2f) Dz:%4.2f(r:%06.2f j:%06.2f)     ",
        Str_OnOff(weld_idx.arc_sensor), g_n_adc_data, node,
        Delta_Z[node], ref_weight,  0.5*(Mean_Moving_Weight[node] + Mean_Moving_Weight[(0<node)? node-1 : node]),
        Delta_T[node], ref_current, 0.5*(Mean_Moving_Ampere[node] + Mean_Moving_Ampere[(0<node)? node-1 : node]));

    if(g_weave.i_mot >= g_weave.n_start)
    {
           VGA_printf(0, g_i_vga+7, "Main Node Idx:%03d Arc WV Pos:%.2f %.2f %.2f       ",
               g_weave.i_mot - g_weave.n_start, 
               shm_servo->d_xw[g_weave.i_mot - g_weave.n_start], 
               shm_servo->d_yw[g_weave.i_mot - g_weave.n_start], 
               shm_servo->d_zw[g_weave.i_mot - g_weave.n_start]); 
    }
    else
    {
        VGA_printf(0, g_i_vga+7, "Main Node Idx:%03d Arc WV Pos:%.2f %.2f %.2f       ", 0, 0., 0., 0.);
    }
}
