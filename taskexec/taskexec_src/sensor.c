// Modified Part // 
#include "memory.h"
#include "sensor.h"

#define ON              1
#define OFF             0

////////////////////////////////////////////////////////////////////////////////

#if 0
shm_mmi_task_t      shm_mmi_temp;
shm_task_servo_t    shm_shm_servo;

// shared memory definition TEMPORARY
shm_mmi_task_t      *shm_mmi = &shm_mmi_temp;
shm_task_servo_t    *shm_servo = &shm_shm_servo;
#endif

volatile shm_mmi_task_t*         shm_mmi = NULL; 
volatile shm_task_servo_t*       shm_servo = NULL; 


// global 
weld_idx_t  weld_idx;

////////////////////////////////////////////////////////////////////////////////
uint arc_save_num;
double ref_current; 
double ref_weight; 
double Delta_T[NUM_OF_NODE];    // dir of yw
double Delta_Z[NUM_OF_NODE];    // dir of zw(upper from workpiece is '+')
double Mean_Moving_Ampere[NUM_OF_NODE];
double Mean_Moving_Weight[NUM_OF_NODE];

// shared memory

static word Rdata[DEF_NODE_ADD][RDATA_SIZE];
static uint data_num;
static word ADC_Data[ARC_ARRAY_SIZE];
static double Ampere[NUM_OF_NODE];
static double Moving_Ampere[NUM_OF_NODE];

#define  MAX_OFFSET_ANGLE	(6.5)
#define  SAMPLE_SIZE		(4)
#define  TRACK_GROUP		(2)
#define  REFER_GROUP		(8)
#define  SIGMA			(0.7)
#define  SCALE			(70.0928)
#define  OFFSET			(-1.8027)

#define  NO_SKIP_WEAVE	(12)
#define  CK0			(270.9069)
#define  CK1			(-8.6778)
#define  CK2			(-0.0776)
#define  CK3			(-2.8625)
#define  CK4			(0.0464)
#define  CK5			(-2.9583)
#define  WK0			(-15.0390)
#define  WK1			(0.4129)
#define  WK2			(2.0249)
#define  WK3			(0.0003)
#define  WK4			(-0.0058)
#define  WK5			(-0.6879)

// Used to detect arc end
#define End_Ref_Start		(9)
#define End_Ref_Group		(6)

#if 0
static int  get_moving_ampere(weave_data_t *wv, main_cond_t *main, uint node);
#else
static int  get_moving_ampere(weave_data_t *wv, uint node);
#endif
static int  get_moving_weight(uint node);
static void moving_seam_track(uint node, double *dt, double *dz);

/*************** Variables for Seam Tracking ***************/

unsigned short NODE_ADD= DEF_NODE_ADD;

static int    weave_dir=-1;
static int new_no_data;

static double volt;  /* ref_current,Ta,Tb,*/
static double feed, speed; //?   /* ref_current,Ta,Tb,*/


static double Ta, Tb, Za, Zb;
static double up_limit_amp, low_limit_amp;
static double up_limit_wei, low_limit_wei;
static double stick_out,ref_w_offset;

#if 0
static double down_ref_current, up_ref_current, down_stick_out, up_stick_out,
       down_up_limit_amp, down_low_limit_amp, up_up_limit_amp, up_low_limit_amp;
static double down_ref_weight, up_ref_weight, down_ref_w_offset, up_ref_w_offset,
       down_up_limit_wei, down_low_limit_wei, up_up_limit_wei, up_low_limit_wei;
#endif

#if 0
// arc end detect functions 
uint detect_arc_end(weave_data_t *wv, weld_end_detect_cond_t *wedc)
{
	uint i= 0, wnode= 0;
	double end_ref_weight= 0., end_ref_ampere= 0.;
	double mean_weight= 0., mean_ampere1= 0., mean_ampere2= 0.;
	double diff_weight= 0., diff_ampere1= 0., diff_ampere2= 0.;
	char str[80];

// check end detect status
	if ( wedc->end_detect_status == OFF )
		return(ARC_ON);

	// variable initialization
	end_ref_weight = end_ref_ampere = 0.;
	mean_weight = mean_ampere1 = mean_ampere2 = 0.;
	diff_weight = diff_ampere1 = diff_ampere2 = 0.;

	if ( wv->w_flag )
	{
		wnode = (wv->node - wv->wd_node)%NUM_OF_NODE;
	} else
	{
		sprintf(str, "%d, in WD Region                         ", wnode);
		Print(3, 18, str);
		return(ARC_ON);
	}
	// if weaving initial region, return
	if ( wnode < NO_SKIP_WEAVE )
	{
		sprintf(str, "%d, in ARC END SKIP Region               ", wnode);
		Print(3, 18, str);
		return(ARC_ON);
	}
	// Calculation of End Detect Reference
	for ( i= wnode - End_Ref_Start; i< wnode - End_Ref_Start + End_Ref_Group; i++ )
	{
		// calculation of end detect weight
		end_ref_weight += Mean_Moving_Weight[i] / End_Ref_Group;
		// calculation of end detect ampere
		end_ref_ampere += Mean_Moving_Ampere[(i%NUM_OF_NODE)] / End_Ref_Group;
	}
// calculation of mean weight
	mean_weight = ( Mean_Moving_Weight[wnode] + Mean_Moving_Weight[wnode - 1] ) / 2.;
// calculation of mean ampere1
	mean_ampere1 = ( Mean_Moving_Ampere[wnode] + Mean_Moving_Ampere[wnode - 1] ) / 2.;
// calculation of mean ampere2
	mean_ampere2 = ( Mean_Moving_Ampere[wnode] + Mean_Moving_Ampere[wnode - 1] + Mean_Moving_Ampere[wnode - 2] ) / 3.;

// calculation of difference of weight and ampere w.r.t end detect reference
	diff_weight = fabs( end_ref_weight - mean_weight );
	diff_ampere1 = fabs( end_ref_ampere - mean_ampere1 );
	diff_ampere2 = fabs( end_ref_ampere - mean_ampere2 );

// comparision of conditions
	if ( wedc->cond_a <= diff_weight && diff_weight < wedc->cond_b )
	{
		sprintf(str, "%d, C1 Yes, ARC OFF                      ", wnode);
		Print(3, 18, str);
		return(ARC_OFF);
	} else if ( wedc->cond_c <= diff_weight && diff_weight < wedc->cond_a )
	{
		if ( wedc->cond_d <= diff_ampere1 )
		{
			sprintf(str, "%d, C1 No, C2 Yes, C3 Yes, ARC OFF       ", wnode);
			Print(3, 18, str);
			return(ARC_OFF);
		} else if ( wedc->cond_e <= diff_ampere2 )
		{
			sprintf(str, "%d, C1 No, C2 Yes, C3 No, C4 Yes, ARC OFF", wnode);
			Print(3, 18, str);
			return(ARC_OFF);
		} else
		{
			sprintf(str, "%d, C1 No, C2 Yes, C3 No, C4 No, ARC ON  ", wnode);
			Print(3, 18, str);
			return(ARC_ON);
		}
	} else
	{
		sprintf(str, "%d, C1 No, C2 No, ARC ON                 ", wnode);
		Print(3, 18, str);
		return(ARC_ON);
	}
}
#endif

// dt : correction value wrt Current value. 
// dz : correction value wrt Even 'yw' dir.  
void get_correct_value(double *dt, double *dz, weave_data_t *wv)
{
	int node = 0;

	// check arc sensor status is ON
	// if arc sensor is OFF, initialize and return
	if ( weld_idx.arc_sensor == OFF )
	{
		*dt = *dz = 0.0;
		return;
	}

	// if wd region process
	if ( wv->w_flag )
	{
		node = wv->node - wv->wd_node;
// original        
		if ( (wv->wd_node + 1) % 2) // wd_node is even number.
			weave_dir = 1;
		else                        // wd_node is odd number
			weave_dir = -1;
        // weave_dir is negative to the 'yw' dir because of CK5 & WK5 is '-'. 
        // To arrange compensation dir in 'dz = (ref_weight-judge_weight) / CK5'
	} 
    else 
    {
		*dt = *dz = 0.0;
		return;
	}

// ADC_Data number setting
	data_num = (uint)(shm_servo->ADC_gathering_index % ARC_ARRAY_SIZE);
// ADC_Data copy
	if ( data_num <= ARC_ARRAY_SIZE )
    {
		memcpy(ADC_Data, (void*)shm_servo->ADC_Data, sizeof(word)*data_num);
    }
	else
	{
		*dt= *dz = 0.;
		return;
	}

	if (1)  // original : w_xang_test == 0.0
	{ // if weaving plane angle == 0
		// if (node > 2 && get_moving_ampere(wv, node) && get_moving_weight(node))
        if (node >= 0 && get_moving_ampere(wv, node) && get_moving_weight(node))
		{
			if( node >= NO_SKIP_WEAVE )
			{ // sensor modification region
				moving_seam_track(node, dt, dz);
			} 
            else
            {
				*dt = *dz = 0.0;
            }
		} 
        else
        {
			*dt = *dz = 0.0;
        }
		arc_save_num = (uint)node;
		shm_servo->arc_save_num = (uint)node;
	}     

	Delta_Z[(node%NUM_OF_NODE)] = *dz;
	Delta_T[(node%NUM_OF_NODE)] = *dt;
	shm_servo->Delta_Z[(node%NUM_OF_NODE)] = Delta_Z[(node%NUM_OF_NODE)];
	shm_servo->Delta_T[(node%NUM_OF_NODE)] = Delta_T[(node%NUM_OF_NODE)];
}

#if 0
static int get_moving_ampere(weave_data_t *wv, main_cond_t *main, uint node)
{
	int i= 0, j= 0;
	uint min1= 0, min2= 0, max1= 0, max2= 0, mid1= 0, mid2= 0, sum= 0;
	int no_data= 0;
	double amp= 0., c_alpa= 0., c_beta= 0.;
	double sum_ampere= 0., sum_moving_ampere= 0.;
	uint rptr = 0;

	sum_ampere= sum_moving_ampere= 0.0;
	no_data = data_num / SAMPLE_SIZE;
	Mean_Moving_Ampere[(node%NUM_OF_NODE)]=0.0;
	new_no_data = 0;

	for(i=0; i<no_data; i++)
	{ // get mean ampere : averaging of estimate current
		j = i * SAMPLE_SIZE;
		min1 = (word)(( ADC_Data[j]< ADC_Data[j+1] )	? ADC_Data[j]	: ADC_Data[j+1]);
		max1 = (word)(( ADC_Data[j]< ADC_Data[j+1] )	? ADC_Data[j+1]	: ADC_Data[j]);
		min2 = (word)(( ADC_Data[j+2]< ADC_Data[j+3] )	? ADC_Data[j+2]	: ADC_Data[j+3]);
		max2 = (word)(( ADC_Data[j+2]< ADC_Data[j+3] )	? ADC_Data[j+3]	: ADC_Data[j+2]);
		mid1 = (word)(( min1<min2 ) ? min2 : min1);
		mid2 = (word)(( max1<max2 ) ? max1 : max2);

		sum = (word)((mid1+mid2)/2.);

		if (node >= shm_mmi->sdata_save_cond.start &&
			node < shm_mmi->sdata_save_cond.start + DEF_NODE_ADD )
		{
			if (rptr<RDATA_SIZE)
			{
				shm_servo->Rdata[(node-shm_mmi->sdata_save_cond.start)%DEF_NODE_ADD][(rptr)%RDATA_SIZE] = sum;
				Rdata[(node-shm_mmi->sdata_save_cond.start)%DEF_NODE_ADD][(rptr++)%RDATA_SIZE] = sum;
			}
		}

		volt = (sum*9.9954)/4096.;	// convert shunt voltage to real voltage
		amp = volt * shm_mmi->wm.c_a + shm_mmi->wm.c_b;	// convert voltage to current

		new_no_data ++;
		if(amp <= 30.) continue;
		Ampere[(new_no_data%NUM_OF_NODE)] = amp;

		sum_ampere += amp;
	}

	if (new_no_data < 2 ) return(0);

/////////////////////// Moving Average ////////////////////////
	Moving_Ampere[0] = (Ampere[0] + Ampere[1]) / 2.;
	sum_moving_ampere += Moving_Ampere[0];

	for(i=1;i<new_no_data-1;i++)
	{
		Moving_Ampere[(i%NUM_OF_NODE)] = SIGMA*Moving_Ampere[i-1] + (1.0-SIGMA) * Ampere[i+1];
		sum_moving_ampere += Moving_Ampere[i];
	}

	Mean_Moving_Ampere[(node%NUM_OF_NODE)] = sum_moving_ampere / (new_no_data-1);
	shm_servo->Mean_Moving_Ampere[(node%NUM_OF_NODE)] = Mean_Moving_Ampere[(node%NUM_OF_NODE)];

	if(node >= NO_SKIP_WEAVE - 1)
	{
		if(node == NO_SKIP_WEAVE - 1)
		{   // get reference current
			////////  new reference ///////////
			/// volt=voltage, feed=wire Feeding rate, speed=weaving speed ///
			ref_current=0.0;
			for(i=node;i>(NO_SKIP_WEAVE - 1)-REFER_GROUP ;i--)
            {
				ref_current += Mean_Moving_Ampere[i] / REFER_GROUP;
            }

			feed= main->curr;
			volt= main->volt;
			speed= wv->speed;

			c_alpa=CK0+CK1*volt+CK2*feed+CK3*speed+CK4*volt*feed;
			c_beta=CK5;
			Ta=1/c_beta;
			Tb=-(c_alpa/c_beta);
			stick_out=Ta*ref_current + Tb;
			up_limit_amp=((stick_out-3.0)-Tb) / Ta;
			low_limit_amp=((stick_out+3.0)-Tb) / Ta;
		}
	}

	return(1);
}
#else
static int get_moving_ampere(weave_data_t *wv, uint node)
{
	int i= 0, j= 0;
	uint min1= 0, min2= 0, max1= 0, max2= 0, mid1= 0, mid2= 0, sum= 0;
	int no_data= 0;
	double amp= 0., c_alpa= 0., c_beta= 0.;
	double sum_ampere= 0., sum_moving_ampere= 0.;
	uint rptr = 0;

	sum_ampere= sum_moving_ampere= 0.0;
	no_data = data_num / SAMPLE_SIZE;
	Mean_Moving_Ampere[(node%NUM_OF_NODE)]=0.0;
	new_no_data = 0;

	for(i=0; i<no_data; i++)
	{ // get mean ampere : averaging of estimate current
		j = i * SAMPLE_SIZE;
		min1 = (word)(( ADC_Data[j]< ADC_Data[j+1] )	? ADC_Data[j]	: ADC_Data[j+1]);
		max1 = (word)(( ADC_Data[j]< ADC_Data[j+1] )	? ADC_Data[j+1]	: ADC_Data[j]);
		min2 = (word)(( ADC_Data[j+2]< ADC_Data[j+3] )	? ADC_Data[j+2]	: ADC_Data[j+3]);
		max2 = (word)(( ADC_Data[j+2]< ADC_Data[j+3] )	? ADC_Data[j+3]	: ADC_Data[j+2]);
		mid1 = (word)(( min1<min2 ) ? min2 : min1);
		mid2 = (word)(( max1<max2 ) ? max1 : max2);

		sum = (word)((mid1+mid2)/2.);

#if 0 // original 
        if (shm_mmi->sdata_save_cond.status && 
            node >= shm_mmi->sdata_save_cond.start &&
			node < shm_mmi->sdata_save_cond.start + DEF_NODE_ADD )
		{
			if (rptr<RDATA_SIZE)
			{                
				shm_servo->Rdata[(node-shm_mmi->sdata_save_cond.start)%DEF_NODE_ADD][(rptr)%RDATA_SIZE] = sum;
				Rdata[(node-shm_mmi->sdata_save_cond.start)%DEF_NODE_ADD][(rptr++)%RDATA_SIZE] = sum;
			}            
		}
#else // writing rdata to shm->rdata for save with no regard of saving req.
      // writing count of rdata to shm
		
#if 1   
        if (rptr<RDATA_SIZE)
        {                
	        shm_servo->Rdata[node % DEF_NODE_ADD][(rptr)%RDATA_SIZE] = sum;
	        Rdata[node % DEF_NODE_ADD][(rptr++)%RDATA_SIZE] = sum;
            shm_servo->n_rdata[node % DEF_NODE_ADD] = rptr;             
        }            
#else
        // arc sensor debug
        // You can use this code after loading sensor saved data by 'SC'('scp 44'). 
        if (rptr<RDATA_SIZE)
        {    
            no_data = 53; 
            sum = shm_servo->Rdata[node % DEF_NODE_ADD][(i)%RDATA_SIZE]; 	        
        }
#endif 

#endif 
        

#if 0
		volt = (sum*9.9954)/4096.;	// convert shunt voltage to real voltage
#endif 
        volt = (sum*shm_mmi->wm.ain_max)/shm_mmi->wm.adc_max;   // convert shunt voltage to real voltage
		amp  = volt * shm_mmi->wm.c_a + shm_mmi->wm.c_b;	        // convert voltage to current

		new_no_data ++;
		if(amp <= 30.) continue;
		Ampere[(new_no_data%NUM_OF_NODE)] = amp;

		sum_ampere += amp;
	}

#if 0 // Test with forcing data. mrch0. 
     {
        word rdata; 
        new_no_data = 180;         
        for(i=0 ; i<new_no_data ; i++)
        {            
            rdata = shm_servo->Rdata[10][i]; 
            volt = rdata * shm_mmi->wm.ain_max/shm_mmi->wm.adc_max; 
            amp  = volt * shm_mmi->wm.c_a + shm_mmi->wm.c_b;	        // convert voltage to current
            Ampere[i] = amp; 
        }
    }
#endif

     if (new_no_data < 2 ) return(0);

/////////////////////// Moving Average ////////////////////////
	Moving_Ampere[0] = (Ampere[0] + Ampere[1]) / 2.;
	sum_moving_ampere += Moving_Ampere[0];

	for(i=1;i<new_no_data-1;i++)
	{
		Moving_Ampere[(i%NUM_OF_NODE)] = SIGMA*Moving_Ampere[i-1] + (1.0-SIGMA) * Ampere[i+1];
		sum_moving_ampere += Moving_Ampere[i];
	}

	Mean_Moving_Ampere[(node%NUM_OF_NODE)] = sum_moving_ampere / (new_no_data-1);
	shm_servo->Mean_Moving_Ampere[(node%NUM_OF_NODE)] = Mean_Moving_Ampere[(node%NUM_OF_NODE)];

	if(node >= NO_SKIP_WEAVE - 1)
	{
		if(node == NO_SKIP_WEAVE - 1)
		{   // get reference current
			////////  new reference ///////////
			/// volt=voltage, feed=wire Feeding rate, speed=weaving speed ///
			ref_current=0.0;
			for(i=node;i>(NO_SKIP_WEAVE - 1)-REFER_GROUP ;i--)
            {
				ref_current += Mean_Moving_Ampere[i] / REFER_GROUP;
            }

            feed= weld_idx.I_main;
            volt= weld_idx.V_main;
			speed= wv->speed;

			c_alpa=CK0+CK1*volt+CK2*feed+CK3*speed+CK4*volt*feed;
			c_beta=CK5;
			Ta=1/c_beta;
			Tb=-(c_alpa/c_beta);
			stick_out=Ta*ref_current + Tb;
			up_limit_amp=((stick_out-3.0)-Tb) / Ta;
			low_limit_amp=((stick_out+3.0)-Tb) / Ta;
		}
	}

	return(1);
}
#endif

static int get_moving_weight(uint node)
{
	int n, i;
	double weight_factor;
	double w_alpa,w_beta, sum_moving_weight;

	if (new_no_data < 2) return(0);
	n = (new_no_data-1) / 2;

	sum_moving_weight = 0.0;
	weight_factor = Mean_Moving_Weight[(node%NUM_OF_NODE)] = 0.0;

	for( i=0; i<(new_no_data-1); i++) 
    {
        // n == 2*new_no_data
        // odd number of new_number_data
		if((new_no_data-1) % 2) 
        {	
			if(node % 2) 
            {	// down-weaving(odd sensor node) : 1->-1
				if(i == n)  weight_factor=0.0;
				else weight_factor= 1.0 * (1.0 - (double)i/n);
			} 
            else 
            {	// up-weaving(even sensor node) : -1->1
				if(i == n)  weight_factor=0.0;
				else weight_factor = -1.0 * (1.0 - (double)i/n);
			}
		} 
        else 
        {	
            // even number of new_number_data
			if(node % 2) 
            {	// down-weaving(odd sensor node) : 1->-1
				if(i < n)
					weight_factor = 1.0 * (1.0 - (double)i/n);
				else
					weight_factor = 1.0 * (1.0 - (double)(i+1)/n);
			} 
            else 
            {	// up-weaving(even sensor node) : -1->1
				if(i < n)
					weight_factor = -1.0 * (1.0 - (double)i/n);
				else
					weight_factor = -1.0 * (1.0 - (double)(i+1)/n);
			}
		}
        // weave_dir : count of start_node is even(1), odd(-1)
		sum_moving_weight += Moving_Ampere[i] * weight_factor * weave_dir;
	}

	Mean_Moving_Weight[(node%NUM_OF_NODE)] = sum_moving_weight / (new_no_data-1);
	shm_servo->Mean_Moving_Weight[(node%NUM_OF_NODE)] = Mean_Moving_Weight[(node%NUM_OF_NODE)];

	if(node >= NO_SKIP_WEAVE - 1)
	{
		if(node == NO_SKIP_WEAVE - 1)
		{ // get reference weight
			/**** V=voltage, F=wire Feeding rate, S=weaving speed ****/
			ref_weight = 0.0;
			for(i = node; i>(NO_SKIP_WEAVE - 1)-REFER_GROUP ; i--)
				ref_weight += Mean_Moving_Weight[i] / REFER_GROUP;            
			w_alpa=WK0+WK1*stick_out+WK2*speed+WK3*volt*feed+WK4*feed*speed;
			w_beta=WK5;
			Za=1/w_beta;
			Zb=-(w_alpa/w_beta);
#if 1
            ref_w_offset = Za*ref_weight + Zb;
            up_limit_wei = ((ref_w_offset-3.0) - Zb) / Za;
			low_limit_wei = ((ref_w_offset+3.0) - Zb) / Za;
#else 
			/********* for vertical welding **********/
// absolute methode 8.17
			Zb = 0.0; //98.08.20 whsung
			ref_w_offset = Za*ref_weight + Zb;
			up_limit_wei = ((ref_w_offset-3.0) - Zb) / Za;
			low_limit_wei = ((ref_w_offset+3.0) - Zb) / Za;
// absolute methode 8.21
			ref_w_offset = 0.0; //98.08.20 whsung
#endif 
		}
	}
	return(1);
}

static void moving_seam_track(uint node, double *dt, double *dz)
{
	uint i= 0;
	double z= 0., t= 0., judge_weight= 0., judge_ampere= 0.;

	for(i = node; i > node - TRACK_GROUP; i--) 
    {
		judge_ampere += Mean_Moving_Ampere[i] / TRACK_GROUP;
		judge_weight += Mean_Moving_Weight[i] / TRACK_GROUP;
	}

	z = Za * judge_weight + Zb - ref_w_offset;
	t = Ta * judge_ampere + Tb - stick_out;
    
    if(z > CORRECTION_LIMIT) z = CORRECTION_LIMIT;
	if(z < -CORRECTION_LIMIT) z = -CORRECTION_LIMIT;
	if(t > CORRECTION_LIMIT) t = CORRECTION_LIMIT;
	if(t < -CORRECTION_LIMIT) t = -CORRECTION_LIMIT;
    
#if 0   // original dandy
	*dz = -z;	// correction value in weaving axis
	*dt = -t;	// correction value in depth axis
#else   // modified by mrch0
    *dz = z; 
    *dt = t;

#endif
}
