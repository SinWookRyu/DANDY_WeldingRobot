// FILEMGR.C is the File Write Program. 
// 2013.7.26 mrch0
// 
// Usage : FILE_Open -> Check(File Pointer NULL) -> FILE_Write ... -> FILE_Close
// User can modify some Definitions defined in Modify Definition Section 
// E.X) MAX_ROBOT_COUNT..

#include <stdio.h>
#include "CRT.h"
#include "filemgr.h"

////////////////////////////////////////////////////////////////////////////////
// Modify Definition as Project 

#define MAX_ROBOT_COUNT                     4
#define FILE_MAX_WRITE_COUNT                100000

#define NUM_SIZE                            2   // Robot Number Char Size

#if 0
char s_str_axis[] = "axis_00";                  // _00 : robot index
char s_str_cart[] = "cart_00"; 
char s_str_prof[] = "prof_00"; 
char s_str_trac[] = "trac_00"; 

////////////////////////////////////////////////////////////////////////////////
// Static Variables 
static int  s_nWriteCount[MAX_ROBOT_COUNT];   // What times wrote

FILE *fpProf[MAX_ROBOT_COUNT] = {0,};
FILE *fpAxis[MAX_ROBOT_COUNT] = {0,};
FILE *fpCart[MAX_ROBOT_COUNT] = {0,};
FILE *fpTrack[MAX_ROBOT_COUNT]= {0,};

////////////////////////////////////////////////////////////////////////////////

// File Opens. If opened already, no new open. 
int File_Open(int nRob)
{
    int nRet = 0;   
    int i_num; 
    int err;

    s_nWriteCount[nRob] = 0;

    ///// Profile File /////
    if(!fpProf[nRob])
    {
        // File Name with Rob Index
        i_num = strlen(s_str_prof) - NUM_SIZE; 
        CRT_sprintf(&s_str_prof[i_num], NUM_SIZE+1, "%02d", nRob); 

        // Open      
        err = CRT_fopen(&fpProf[nRob], s_str_prof, "wt");

        if(fpProf[nRob] == NULL)
        {
            nRet = -1; 
            goto POST_WORK; 
        }
    }
    
    ///// Axis File /////
    if(!fpAxis[nRob])
    {
        // File Name with Rob Index
        i_num = strlen(s_str_axis) - NUM_SIZE; 
        CRT_sprintf(&s_str_axis[i_num], NUM_SIZE+1, "%02d", nRob); 

        // Open    
        err = CRT_fopen(&fpAxis[nRob], s_str_axis, "wt");
        if(fpAxis[nRob] == NULL)
        {
            nRet = -1; 
            goto POST_WORK; 
        }

    }
    
    ///// Cartesian File /////    
    if(!fpCart[nRob])
    {
        // File Name with Rob Index
        i_num = strlen(s_str_cart) - NUM_SIZE; 
        CRT_sprintf(&s_str_cart[i_num], NUM_SIZE+1, "%02d", nRob); 

        // Open    
        err = CRT_fopen(&fpCart[nRob], s_str_cart, "wt");
        if(fpCart[nRob] == NULL)
        {
            nRet = -1; 
            goto POST_WORK; 
        }
    }

    ///// Tracking File /////
    if(!fpTrack[nRob])
    {
        // File Name with Rob Index
        i_num = strlen(s_str_trac) - NUM_SIZE; 
        CRT_sprintf(&s_str_trac[i_num], NUM_SIZE+1, "%02d", nRob); 

        // Open    
        err = CRT_fopen(&fpTrack[nRob], s_str_trac, "wt");
        if(fpTrack[nRob] == NULL)
        {
            nRet = -1; 
            goto POST_WORK; 
        }
    }
    
POST_WORK:
    if(nRet)
    {
        File_Close(nRob);
        return -1;
    }
    return 0; 
}

void File_Close(int nRob)
{      
    if(fpAxis[nRob] != NULL)
    {
        fclose(fpAxis[nRob]);
        fpAxis[nRob] = NULL;  
    }

    if(fpCart[nRob] != NULL)
    {
        fclose(fpCart[nRob]);
        fpCart[nRob] = NULL;            
    }

    if(fpProf[nRob] != NULL)
    {
        fclose(fpProf[nRob]);
        fpProf[nRob] = NULL;     
    }

    if(fpTrack[nRob] != NULL)
    {
        fclose(fpTrack[nRob]);
        fpTrack[nRob] = NULL;            
    }
}

// Writes Data on File. 
void File_Write(const double* profile, const double axis[6], 
                const double cart[6],  const double track[6], int nRob)       
{     

    // Count Check   
    if(s_nWriteCount[nRob] > FILE_MAX_WRITE_COUNT) 
    {
        File_Close(nRob);
    }
    s_nWriteCount[nRob]++;
#if 0
    // Profile
    if(fpProf[nRob] != NULL && profile != NULL)
    {
        fprintf(fpProf[nRob], "%.20f %.20f %.20f %.20f %.20f %.20f\n",                 
                profile[0], profile[1], profile[2], profile[3], profile[4], profile[5]); 
    }
#else
    // Profile
    if(fpProf[nRob] != NULL && profile != NULL)
    {
        fprintf(fpProf[nRob], "%.20f\n", *profile); 
    }
#endif

    // Axis Write
    if(fpAxis[nRob] != NULL && axis != NULL)
    {
        fprintf(fpAxis[nRob], "%.20f %.20f %.20f %.20f %.20f %.20f\n",                 
                axis[0], axis[1], axis[2], axis[3], axis[4], axis[5]); 
    }

    // Cartesian Write
    if(fpCart[nRob] != NULL && cart != NULL)
    {
        fprintf(fpAxis[nRob], "%.20f %.20f %.20f %.20f %.20f %.20f\n", 
                cart[0], cart[1], cart[2], cart[3], cart[4], cart[5]); 
    }

    // Tracking Data Write
    if(fpTrack[nRob] != NULL && track != NULL)
    {
        fprintf(fpTrack[nRob], "%.20f %.20f %.20f %.20f %.20f %.20f\n", 
                track[0], track[1], track[2], track[3], track[4], track[5]); 
    }
}

#else

////////////////////////////////////////////////////////////////////////////////
// Static Variables 

char s_str_trg[] = "trg_00";                  // _00 : robot index
char s_str_act[] = "act_00"; 
char s_str_cart[] = "cart_00"; 
char s_str_prof[] = "prof_00"; 

static int  s_nWriteCount[MAX_ROBOT_COUNT];   // What times wrote

FILE *fpTrg[4] = {0,};
FILE *fpAct[4] = {0,};
FILE *fpCart[4] = {0,};
FILE *fpProf[4] = {0,};

////////////////////////////////////////////////////////////////////////////////

// File Opens. If opened already, no new open. 
int File_Open(int nRob)
{
    int nRet = 0;   
    int i_num; 
    int err;

    s_nWriteCount[nRob] = 0;

    ///// Profile File /////
    if(!fpProf[nRob])
    {
        // File Name with Rob Index
        i_num = strlen(s_str_prof) - NUM_SIZE; 
        CRT_sprintf(&s_str_prof[i_num], NUM_SIZE+1, "%02d", nRob); 

        // Open      
        err = CRT_fopen(&fpProf[nRob], s_str_prof, "wt");

        if(fpProf[nRob] == NULL)
        {
            nRet = -1; 
            goto POST_WORK; 
        }
    }
    
    ///// Axis File /////
    if(!fpTrg[nRob])
    {
        // File Name with Rob Index
        i_num = strlen(s_str_trg) - NUM_SIZE; 
        CRT_sprintf(&s_str_trg[i_num], NUM_SIZE+1, "%02d", nRob); 

        // Open    
        err = CRT_fopen(&fpTrg[nRob], s_str_trg, "wt");
        if(fpTrg[nRob] == NULL)
        {
            nRet = -1; 
            goto POST_WORK; 
        }

    }
    
    ///// Cartesian File /////    
    if(!fpCart[nRob])
    {
        // File Name with Rob Index
        i_num = strlen(s_str_cart) - NUM_SIZE; 
        CRT_sprintf(&s_str_cart[i_num], NUM_SIZE+1, "%02d", nRob); 

        // Open    
        err = CRT_fopen(&fpCart[nRob], s_str_cart, "wt");
        if(fpCart[nRob] == NULL)
        {
            nRet = -1; 
            goto POST_WORK; 
        }
    }

    ///// Actual File /////
    if(!fpAct[nRob])
    {
        // File Name with Rob Index
        i_num = strlen(s_str_act) - NUM_SIZE; 
        CRT_sprintf(&s_str_act[i_num], NUM_SIZE+1, "%02d", nRob); 

        // Open    
        err = CRT_fopen(&fpAct[nRob], s_str_act, "wt");
        if(fpAct[nRob] == NULL)
        {
            nRet = -1; 
            goto POST_WORK; 
        }
    }
    
POST_WORK:
    if(nRet)
    {
        File_Close(nRob);
        return -1;
    }
    return 0; 
}

void File_Close(int nRob)
{      
    if(fpTrg[nRob] != NULL)
    {
        fclose(fpTrg[nRob]);
        fpTrg[nRob] = NULL;  
    }

    if(fpCart[nRob] != NULL)
    {
        fclose(fpCart[nRob]);
        fpCart[nRob] = NULL;            
    }

    if(fpProf[nRob] != NULL)
    {
        fclose(fpProf[nRob]);
        fpProf[nRob] = NULL;     
    }

    if(fpAct[nRob] != NULL)
    {
        fclose(fpAct[nRob]);
        fpAct[nRob] = NULL;            
    }
}

// Writes Data on File. 
void File_Write(const double* profile, const double trg[6], 
                const double cart[6],  const double act[6], int nRob)       
{     

    // Count Check   
    if(s_nWriteCount[nRob] > FILE_MAX_WRITE_COUNT) 
    {
        File_Close(nRob);
    }
    s_nWriteCount[nRob]++;
#if 0
    // Profile
    if(fpProf[nRob] != NULL && profile != NULL)
    {
        fprintf(fpProf[nRob], "%.20f %.20f %.20f %.20f %.20f %.20f\n",                 
                profile[0], profile[1], profile[2], profile[3], profile[4], profile[5]); 
    }
#else
    // Profile
    if(fpProf[nRob] != NULL && profile != NULL)
    {
        fprintf(fpProf[nRob], "%.20f\n", *profile); 
    }
#endif

    // trg Write
    if(fpTrg[nRob] != NULL && trg != NULL)
    {
        fprintf(fpTrg[nRob], "%.20f %.20f %.20f %.20f %.20f %.20f\n",                 
                trg[0], trg[1], trg[2], trg[3], trg[4], trg[5]); 
    }

    // Cartesian Write
    if(fpCart[nRob] != NULL && cart != NULL)
    {
        fprintf(fpCart[nRob], "%.20f %.20f %.20f %.20f %.20f %.20f\n", 
                cart[0], cart[1], cart[2], cart[3], cart[4], cart[5]); 
    }

    // Tracking Data Write
    if(fpAct[nRob] != NULL && act != NULL)
    {
        fprintf(fpAct[nRob], "%.20f %.20f %.20f %.20f %.20f %.20f\n", 
                act[0], act[1], act[2], act[3], act[4], act[5]); 
    }
}
#endif