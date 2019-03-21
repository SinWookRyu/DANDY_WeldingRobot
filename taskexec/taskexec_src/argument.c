/*
 * argument.c
 *  This is the argument managing program for TASKEXEC.
 *  2013. 7. 15. mrch0
 */

#include "taskexec_def.h"

#pragma pack(push, 1)
typedef struct
{
	const char* str_arg;
	const char* str_help;
	void (*func)(const char* sub_arg);
} ARGUMENT;
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////

// Options
static const char s_arg_help[]          = "-h";
static const char s_arg_print[]         = "-p";
static const char s_arg_wait[]          = "-t";
static const char s_arg_no_rm[]			= "-no_rm";
static const char s_arg_no_sc[]			= "-no_sc";
static const char s_arg_loop_back[]		= "-loopback";
static const char s_arg_man_err_reset[] = "-man_err_reset"; 
static const char s_arg_vga_line[]      = "-vga_line";

static const char s_arg_samp[]			= "-samp";
static const char s_arg_peri[]			= "-peri";
static const char s_arg_sync[]			= "-sync";
static const char s_arg_dc[]			= "-dc";

// Help Strings
static const char s_sz_help_te[] =
"TASKEXEC is the task executor of Dandy2015.\n"
"Copyright(c) 2013 DSME Co.Ltd. All Rights Reserved.\n"
"\n"
"USAGE : taskexec_<var> [-h] [-t <time>] [-p <level>]\n"
"\n"
"var: Variants \n"
"   qnx : for QNX \n"
"   win : for Windows\n"
"\n";

static const char s_sz_help_help[] =
"-h : Help Message Print Option. \n"
"     This makes TASKEXEC prints the help message without running.\n"
"     This option is prior to all the options.\n";

static const char s_sz_help_wait[] =
"-t: IPC Initialization Wait Time Option (Keep Alive)\n"
"     Waits for IPC connection for <time>[s] before fail exit.\n"
"     If <time> is <= 0, TASKEXEC waits infinitely. (Default = Inf.)\n";

static const char s_sz_help_print[] =
"-p : The message option controls the message display level\n"
"     The message display levels are among VERBOSE, MESSAGE, WARNING, ERROR,\n"
"     and NO-MESSAGE.\n"
"     The VERBOSE is the lowest and NO-MESSAGE is the highest.\n"
"     If one level is defined, Messages under defined level do not appear.\n"
"     The default message level is MESSAGE.\n"
"\n"
"     <level>\n"
"        v : VERBOSE\n"
"        m : MESSAGE (default)\n"
"        w : WARNING\n"
"        e : ERROR \n"
"        n : NO-MESSAGE\n";

static const char s_sz_help_no_rm[] =
"-no_rm : Runs w/o the system configure SHM of ROBOT_MGR & Uses Def. Values.\n";

static const char s_sz_help_no_sc[] =
"-no_sc : Runs w/o the motor & I/O imforation SHM & Connection of SERVO_CON.\n"
"         And Loop-backs the target value to the actual data directly.\n";

static const char s_sz_help_loopback[] =
"-loopback : Loopbacks Actual Motor Pos from Target Motor Pos Directly.\n"; 

static const char s_sz_help_man_err_reset[] = 
"-man_err_reset : Resets error info's manually when new mode starts."; 

static const char s_sz_help_vga_line[] = 
"-vga_line <i> : VGA start from <i>-th line."; 

// Handlers for each Argument
static void ArgHelp(const char* sub_arg);
static void ArgPrint(const char* sub_arg);
static void ArgWait(const char* sub_arg);
static void ArgNoRm(const char* sub_arg);
static void ArgNoSc(const char* sub_arg);
static void ArgLoopBack(const char* sub_arg);
static void ArgManErrReset(const char* sub_arg); 
static void ArgVgaLine(const char* sub_arg); 
// Utils 
static unsigned ArgCountGet(void);

////////////////////////////////////////////////////////////////////////////////

static ARGUMENT s_argument[] =
{
	{s_arg_help, 	s_sz_help_help, 	ArgHelp},
	{s_arg_print, 	s_sz_help_print, 	ArgPrint},
	{s_arg_wait, 	s_sz_help_wait, 	ArgWait}, 
    {s_arg_no_rm,   s_sz_help_no_rm,    ArgNoRm}, 
    {s_arg_no_sc,   s_sz_help_no_sc,    ArgNoSc}, 
    {s_arg_loop_back,       s_sz_help_loopback,         ArgLoopBack}, 
    {s_arg_man_err_reset,   s_sz_help_man_err_reset,    ArgManErrReset}, 
    {s_arg_vga_line,        s_sz_help_vga_line,         ArgVgaLine},
};

////////////////////////////////////////////////////////////////////////////////

// No help argument parsing : return 0
int Arg_Init(int argc, char* argv[])
{
	int i, j, n;
	char* sub_arg;

	n = ArgCountGet();
    
    g_argc = argc; 
    g_argv = argv; 

    Str_PrintArg(); 

    // Argument Option Value
    for ( i=0 ; i<argc ; i++)	// Argument input
    {
    	sub_arg = (i+1 < argc)? argv[i+1] : NULL;

    	for(j=0 ; j<n ; j++)	// Argument prepared
    	{
    		if(strcmp(argv[i], s_arg_help) == 0 )
    		{
    			ArgHelp(sub_arg);
    			return -1;
    		}
    		else if(strcmp(argv[i], s_argument[j].str_arg) == 0)
			{
				s_argument[j].func(sub_arg);
				continue;
			}
    	}
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

static unsigned ArgCountGet(void)
{
	div_t temp;
	temp = div(sizeof(s_argument), sizeof(ARGUMENT));
	return temp.quot;
}

// Handler of each argument ////////////////////////////////////////////////////

static void ArgHelp(const char* sub_arg)
{
	int i, n;

	n = ArgCountGet();

	// Header Print
	printf("\nTASKEXEC_QNX [Ver:%s Build:%s]\n", TE_VERSION, TE_BUILD);
	printf("%s", s_sz_help_te);

	// Help of each option Print
	for(i=0 ; i<n ; i++)
	{
		printf("%s\n", s_argument[i].str_help);
	}
}

static void ArgPrint(const char* sub_arg)
{
	if(sub_arg != NULL)
	{
		g_n_print_opt = *sub_arg;
	}
}

static void ArgWait(const char* sub_arg)
{
	if(sub_arg != NULL)
	{
		g_ipc_wait = 1000*atoi(sub_arg);
	}
}

static void ArgNoRm(const char* sub_arg)
{
	g_f_no_rm = TRUE;
}

static void ArgNoSc(const char* sub_arg)
{
	g_f_no_sc = TRUE;
}

static void ArgLoopBack(const char* sub_arg)
{
	g_f_loop_back = TRUE;
}

static void ArgManErrReset(const char* sub_arg)
{
    g_f_auto_err_reset = FALSE; 
}

static void ArgVgaLine(const char* sub_arg)
{
    if(sub_arg != NULL)
	{
		g_i_vga = atoi(sub_arg);
	} 
}
