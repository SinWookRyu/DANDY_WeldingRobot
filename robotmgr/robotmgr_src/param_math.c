/////////////////////////////////////////////////////////////////////////////
//
//  param_math.c: Math Function for Parameter Edit Service
//                                            2014.07.07  Ryu SinWook

///////////////////////////////////////
#define _USE_MATH_DEFINES
#include "service.h"
#include <math.h>

///////////////////////////////////////


///////////////////////////////////////
//Global_variable

/////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
//
//  Function: FUNC_CheckTuneOutputValueValidation()
//      - Service Name: RSVC_SERV_WELDTUNE_OUT_EDIT

int FUNC_CheckTuneOutputValueValidation(int nOpt, double dbA, double dbB, double dbC)
{
    double dbDiscUpper, dbDiscLower, dbDiscAvr;
    double dbXcal, dbYcal;
    
    if(nOpt == TUNE_PARAM_VOLT)
    {
        double dbVolt_a = dbA, dbVolt_b = dbB, dbVolt_c = dbC;

        double dbUpperBoundY = g_dbUpperBoundY_Volt,
               dbLowerBoundY = g_dbLowerBoundY_Volt;
        double dbUpperBoundX = g_dbUpperBoundX_Volt,
               dbLowerBoundX = g_dbLowerBoundX_Volt;
        double dbUpperBoundXResultY = (dbUpperBoundX / MAX_WELDOUT_VOLTAGE) * HYOSUNG_WELD_MAX_VOLT_VAL,
               dbLowerBoundXResultY = (dbLowerBoundX / MAX_WELDOUT_VOLTAGE) * HYOSUNG_WELD_MAX_VOLT_VAL;

        double dbYActMax = dbLowerBoundXResultY, dbYActMin = dbUpperBoundXResultY;
        double dbAvrResultY = (dbUpperBoundY + dbLowerBoundY) / 2;
        double dbAvrResultX = 0.0;
#if 0
        dbVolt_a = RM_packet.Data.usrparam.PARAM.output.dbVolt_a;
        dbVolt_b = RM_packet.Data.usrparam.PARAM.output.dbVolt_b;
        dbVolt_c = RM_packet.Data.usrparam.PARAM.output.dbVolt_c;
#endif
        dbDiscUpper = dbVolt_b * dbVolt_b - 4.0 * dbVolt_a * (dbVolt_c - dbUpperBoundY);
        dbDiscLower = dbVolt_b * dbVolt_b - 4.0 * dbVolt_a * (dbVolt_c - dbLowerBoundY);
        dbDiscAvr   = dbVolt_b * dbVolt_b - 4.0 * dbVolt_a * (dbVolt_c - dbAvrResultY);

        if(dbDiscUpper < 0)
        {
            VERBOSE_ERROR("[Volt] Upper Y Value Discriminant is imaginary Value!\n");
            return ERR_VOLT_PARAM_MAX_DISCRIMINANT_IMAGINARY;
        }

        if(dbDiscLower < 0)
        {
            VERBOSE_ERROR("[Volt] Lower Y Value Discriminant is imaginary Value!\n");
            return ERR_VOLT_PARAM_MIN_DISCRIMINANT_IMAGINARY;
        }

        dbXcal = dbLowerBoundX;

        while(dbXcal <= dbUpperBoundX)
        {
            dbYcal = dbVolt_a * dbXcal * dbXcal + dbVolt_b * dbXcal + dbVolt_c;
            
            if(dbYcal < dbYActMin)
            {
                dbYActMin = dbYcal;
            }
            else if(dbYcal > dbYActMax)
            {
                dbYActMax = dbYcal;
            }
            
            if(dbXcal == dbUpperBoundX)
                break;

            dbXcal += 0.1;

            if(dbXcal > dbUpperBoundX)
            {
                dbXcal = dbUpperBoundX;
            }
        }

        dbAvrResultX = (float) (((-1 * dbVolt_b) + sqrt(fabs(dbDiscAvr))) / (2 * dbVolt_a));

        if(dbYActMax < dbUpperBoundY)
        {
            VERBOSE_ERROR("[Volt] Coefficient C Value is Low!(Max: %.1lf < Upper Bound: %.1lf)\n",
                          dbYActMax, dbUpperBoundY);
        	return ERR_VOLT_PARAM_C_VALUE_LOW;
        }
        
        if(dbYActMin > dbLowerBoundY)
        {
            VERBOSE_ERROR("[Volt] Coefficient C Value is High(Min: %.1lf > Lower Bound: %.1lf)!\n",
                          dbYActMin, dbLowerBoundY);
        	return ERR_VOLT_PARAM_C_VALUE_HIGH;
        }
#if 0
        if(dbAvrResultX < dbLowerBoundX)
        {
            SVC_DefineErrorState(ON, SVC_ERR_WOUT_CALIB_PARAM_INVALID);
            VERBOSE_ERROR("[Volt] X Result is too Low!(AvrX: %.2lf < MinX: %.2lf)\n",
                          dbAvrResultX, dbLowerBoundX);
        	return RESULT_ERROR;
        }
        
        if(dbAvrResultX > dbUpperBoundX)
        {
            SVC_DefineErrorState(ON, SVC_ERR_WOUT_CALIB_PARAM_INVALID);
            VERBOSE_ERROR("[Volt] X Result is too High!(AvrX: %.2lf > MaxX: %.2lf)\n",
                          dbAvrResultX, dbUpperBoundX);
        	return RESULT_ERROR;
        }
#endif
        VERBOSE_VERBOSE("[Volt] Validation Check OK!\n"
                        "(Min: %.1lf, Max: %.1lf, BoundY: %.1lf ~ %.1lf)\n"
                        "(BoundX: %.2lf ~ %.2lf, AvrX: %.2lf)\n",
                        dbYActMin, dbYActMax, dbLowerBoundY, dbUpperBoundY,
                        dbLowerBoundX, dbUpperBoundX, dbAvrResultX);
    }

    else if(nOpt == TUNE_PARAM_CURR)
    {
        double dbCurr_a = dbA, dbCurr_b = dbB, dbCurr_c = dbC;

        double dbUpperBoundY = g_dbUpperBoundY_Curr,
               dbLowerBoundY = g_dbLowerBoundY_Curr;
        double dbUpperBoundX = g_dbUpperBoundX_Curr,
               dbLowerBoundX = g_dbLowerBoundX_Curr;
        double dbUpperBoundXResultY = (dbUpperBoundX / MAX_WELDOUT_VOLTAGE) * HYOSUNG_WELD_MAX_CURR_VAL,
               dbLowerBoundXResultY = (dbLowerBoundX / MAX_WELDOUT_VOLTAGE) * HYOSUNG_WELD_MAX_CURR_VAL;

        double dbYActMax = dbLowerBoundXResultY, dbYActMin = dbUpperBoundXResultY;
        double dbAvrResultY = (dbUpperBoundY + dbLowerBoundY) / 2;
        double dbAvrResultX = 0.0;
#if 0
        dbCurr_a = RM_packet.Data.usrparam.PARAM.output.dbCurr_a;
        dbCurr_b = RM_packet.Data.usrparam.PARAM.output.dbCurr_b;
        dbCurr_c = RM_packet.Data.usrparam.PARAM.output.dbCurr_c;
#endif
        dbDiscUpper = dbCurr_b * dbCurr_b - 4.0 * dbCurr_a * (dbCurr_c - dbUpperBoundY);
        dbDiscLower = dbCurr_b * dbCurr_b - 4.0 * dbCurr_a * (dbCurr_c - dbLowerBoundY);
        dbDiscAvr   = dbCurr_b * dbCurr_b - 4.0 * dbCurr_a * (dbCurr_c - dbAvrResultY);

        if(dbDiscUpper < 0)
        {
            VERBOSE_ERROR("[Curr] Upper Y Value Discriminant is imaginary Value!\n");
            return ERR_CURR_PARAM_MAX_DISCRIMINANT_IMAGINARY;
        }

        if(dbDiscLower < 0)
        {
            VERBOSE_ERROR("[Curr] Lower Y Value Discriminant is imaginary Value!\n");
            return ERR_CURR_PARAM_MIN_DISCRIMINANT_IMAGINARY;
        }

        dbXcal = dbLowerBoundX;

        while(dbXcal <= dbUpperBoundX)
        {
            dbYcal = dbCurr_a * dbXcal * dbXcal + dbCurr_b * dbXcal + dbCurr_c;
            
            if(dbYcal < dbYActMin)
            {
                dbYActMin = dbYcal;
            }
            else if(dbYcal > dbYActMax)
            {
                dbYActMax = dbYcal;
            }
            
            if(dbXcal == dbUpperBoundX)
                break;

            dbXcal += 0.1;

            if(dbXcal > dbUpperBoundX)
            {
                dbXcal = dbUpperBoundX;
            }
        }

        dbAvrResultX = (float) (((-1 * dbCurr_b) + sqrt(fabs(dbDiscAvr))) / (2 * dbCurr_a));

        if(dbYActMax < dbUpperBoundY)
        {
            VERBOSE_ERROR("[Curr] Coefficient C Value is Low!(Max: %.1lf < Upper Bound: %.1lf)\n",
                          dbYActMax, dbUpperBoundY);
        	return ERR_CURR_PARAM_C_VALUE_LOW;
        }
        
        if(dbYActMin > dbLowerBoundY)
        {
            VERBOSE_ERROR("[Curr] Coefficient C Value is High!(Min: %.1lf > Lower Bound: %.1lf)\n",
                          dbYActMin, dbLowerBoundY);
        	return ERR_CURR_PARAM_C_VALUE_HIGH;
        }
#if 0
        if(dbAvrResultX < dbLowerBoundX)
        {
            SVC_DefineErrorState(ON, SVC_ERR_WOUT_CALIB_PARAM_INVALID);
            VERBOSE_ERROR("[Curr] X Result is too Low!(AvrX: %.2lf < MinX: %.2lf)\n",
                          dbAvrResultX, dbLowerBoundX);
        	return RESULT_ERROR;
        }
        
        if(dbAvrResultX > dbUpperBoundX)
        {
            SVC_DefineErrorState(ON, SVC_ERR_WOUT_CALIB_PARAM_INVALID);
            VERBOSE_ERROR("[Curr] X Result is too  High!(AvrX: %.2lf > MaxX: %.2lf)\n",
                          dbAvrResultX, dbUpperBoundX);
        	return RESULT_ERROR;
        }
#endif
        VERBOSE_VERBOSE("[Curr] Validation Check OK!\n"
                        "(Min: %.1lf, Max: %.1lf, BoundY: %.1lf ~ %.1lf)\n"
                        "(BoundX: %.2lf ~ %.2lf, AvrX: %.2lf)\n",
                        dbYActMin, dbYActMax, dbLowerBoundY, dbUpperBoundY,
                        dbLowerBoundX, dbUpperBoundX, dbAvrResultX);
    }

    return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: _loc_GetInvMat2()
//      - Service Name: RSVC_SRV_CAL_WELDCOEFF

static int  _loc_GetInvMat2(double dbInvMat[WELD_COEFF_COUNT][WELD_COEFF_COUNT],
                            double dbMat[WELD_COEFF_COUNT][WELD_COEFF_COUNT])
{
	int     i, j;
	double  dbDet;
	double  a[WELD_COEFF_COUNT][WELD_COEFF_COUNT];
	double  dbOutput[WELD_COEFF_COUNT][WELD_COEFF_COUNT];
	
	for(i = 0; i < WELD_COEFF_COUNT; i++)
    {	
		for(j = 0; j < WELD_COEFF_COUNT; j++)
        {
			a[i][j] = dbMat[i][j];
        }
    }

    dbDet= a[0][0]*(a[1][1]*a[2][2]-a[2][1]*a[1][2])
		  -a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])
		  +a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]); //adjoin
    
	if(dbDet == 0)
    {
        VERBOSE_ERROR("Inverse Determinant is Zero!\n");
        return RESULT_ERROR;
    }

    dbOutput[0][0] =  (a[1][1]*a[2][2]-a[2][1]*a[1][2]) / dbDet;
    dbOutput[0][1] = -(a[1][0]*a[2][2]-a[1][2]*a[2][0]) / dbDet;
    dbOutput[0][2] =  (a[1][0]*a[2][1]-a[2][0]*a[1][1]) / dbDet;
    dbOutput[1][0] = -(a[0][1]*a[2][2]-a[0][2]*a[2][1]) / dbDet;
    dbOutput[1][1] =  (a[0][0]*a[2][2]-a[0][2]*a[2][0]) / dbDet;
    dbOutput[1][2] = -(a[0][0]*a[2][1]-a[2][0]*a[0][1]) / dbDet;
    dbOutput[2][0] =  (a[0][1]*a[1][2]-a[0][2]*a[1][1]) / dbDet;
    dbOutput[2][1] = -(a[0][0]*a[1][2]-a[1][0]*a[0][2]) / dbDet;
    dbOutput[2][2] =  (a[0][0]*a[1][1]-a[1][0]*a[0][1]) / dbDet;

    for(i = 0; i < WELD_COEFF_COUNT; i++)
    {
		for(j = 0; j < WELD_COEFF_COUNT; j++)
        {
    		dbInvMat[i][j] = dbOutput[i][j];
        }
    }

	return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: DEVICE_GetCurveCoeff()
//      - Service Name: RSVC_SRV_CAL_WELDCOEFF

static int  _loc_GetCurveCoeff(double* dbCoeff,     // output :  3 x 1 (coefficient)
						       double* dbTerm_A,    // input  : 12 x 1 (A)	
						       double* dbTerm_B,    // input  : 12 x 1 (B)
						       int nData)           // input  : dimension	
{
	int nRet = RESULT_OK;
	int iCol, iRow, iRepeat;

	double dbA[MEASURE_WRITE_COUNT][WELD_COEFF_COUNT];    // A : analog output Command (0 ~ 10[V])
	double dbTrA[WELD_COEFF_COUNT][MEASURE_WRITE_COUNT];  // Tr(A)
	double dbB[MEASURE_WRITE_COUNT];                      // B : Welder Cur/Volt

#if defined(_WIN32)
	double dbM[WELD_COEFF_COUNT][WELD_COEFF_COUNT]      ={0,};  // M = Tr(A).A
	double dbInvM[WELD_COEFF_COUNT][WELD_COEFF_COUNT]   ={0,};  // Inverse(M)
#else
    double dbM[WELD_COEFF_COUNT][WELD_COEFF_COUNT]    ={{0,},{0,},{0,}};  // M = Tr(A).A
	double dbInvM[WELD_COEFF_COUNT][WELD_COEFF_COUNT] ={{0,},{0,},{0,}};  // Inverse(M)
#endif

	double dbN[WELD_COEFF_COUNT]		={0,};          // N = Tr(A).B
	double dbTempC[WELD_COEFF_COUNT]	={0,};

    if(nData < WELD_COEFF_COUNT)
    {
        VERBOSE_ERROR("Cannot Solve 3 by 3 Matrix Equation\n"
                      "(Input Dimenstion: %d)!\n", nData);
        return ERR_NOT_VALID_MATRIX_DIMENSION;
    }

	// 1) A.x=B					, x(welder coefficient, a/b/c )
	// 2) Tr(A).A.x = Tr(A).B	, B(Welder Cur/Volt)
	// 3) M.x = N				, M( =Tr(A).A )
	// 4) x = Inv(M).N			
	
	for (iCol = 0; iCol < nData; iCol++)
	{
		// 1. A (analog output command), B (Welder Cur/Volt)
		dbA[iCol][0] = pow(dbTerm_A[iCol],2);
		dbA[iCol][1] = pow(dbTerm_A[iCol],1);
		dbA[iCol][2] = 1.0;
		dbB[iCol]	= dbTerm_B[iCol];	

		// 2. Tr(A) : Transpose (A)
		for (iRow = 0; iRow < WELD_COEFF_COUNT; iRow++)
        {
			dbTrA[iRow][iCol] = dbA[iCol][iRow];
        }
	}

	// 3. M (3x3)
	for (iRepeat = 0; iRepeat < WELD_COEFF_COUNT; iRepeat++)
	{
		for (iRow = 0 ;iRow < WELD_COEFF_COUNT; iRow++)
		{
			for (iCol = 0; iCol < nData; iCol++)
			{
				dbM[iRepeat][iRow] += dbTrA[iRepeat][iCol] * dbA[iCol][iRow];
			}
		}
	}

	// 4. Inv(M)
	nRet = _loc_GetInvMat2(dbInvM, dbM);
	
    if (nRet != RESULT_OK)
    {
        VERBOSE_ERROR("Cannot Solve Inverse Matrix!\n");
        return ERR_SOVLE_INVERSE_MATRIX;
    }

	// 5. N (3x1)
	for (iRow = 0; iRow < WELD_COEFF_COUNT; iRow++)
	{
		for (iCol = 0; iCol < nData; iCol++)
        {
			dbN[iRow] += dbTrA[iRow][iCol] * dbB[iCol];
        }
	}

	// 6. Coefficient
	for (iRow = 0; iRow < WELD_COEFF_COUNT; iRow++)
	{
		for (iCol = 0; iCol < WELD_COEFF_COUNT; iCol++)
		{
			dbTempC[iRow] += dbInvM[iRow][iCol] * dbN[iCol];
			dbCoeff[iRow] = dbTempC[iRow];
		}
	}

	return RESULT_OK;
}


///////////////////////////////////////
//
//  Function: DEVICE_GetWldrCoeff()
//      - Service Name: RSVC_SRV_CAL_WELDCOEFF
#define		MIN_WLDR_COEFF_RATIO	5.0
#define		MIN_WLDR_COEFF_A		0.000001
#define     MIN_WLDR_REF_DATA	    0.1

int	 FUNC_GetWelderCoeff(double* dbWelderCoeff, BYTE nWelderID, int nOpt)
{
	int     idx, nDim;
	int     nRet = RESULT_OK;
	double  dbWelderOut[MEASURE_WRITE_COUNT];	// welder output
	double  dbAnalogOut[MEASURE_WRITE_COUNT];	// controller's DAC analog output
	double  dbCoeff[WELD_COEFF_COUNT];
	double  dbTemp = 0, dbRatio;
	double  dbD, dbNumerator;
	WELD_COEFF co;

	switch (nOpt)
	{
	case VOLT_OUT:
		for (idx = 0; idx < MEASURE_WRITE_COUNT; idx++)
		{
			dbAnalogOut[idx]	= g_dbControllerCmdVolt[idx];
			dbWelderOut[idx]	= g_dbWelderMeasureVolt[idx];
		}
		break;

	case CURRENT_OUT:
		for (idx = 0; idx < MEASURE_WRITE_COUNT; idx++)
		{
			dbAnalogOut[idx]	= g_dbControllerCmdCurr[idx];
			dbWelderOut[idx]	= g_dbWelderMeasureCurr[idx];
		}
		break;

	default:
		return RESULT_ERROR;
	}

	// get valid data count
	nDim = MEASURE_WRITE_COUNT;
	for (idx=0; idx < MEASURE_WRITE_COUNT; idx++)
	{
		if  ( fabs(dbAnalogOut[idx]) < MIN_WLDR_REF_DATA ||
			  fabs(dbWelderOut[idx]) < MIN_WLDR_REF_DATA)
		{
			nDim = idx;
			break;
		}
	}

	nRet = _loc_GetCurveCoeff(dbCoeff,          // Coeff.
		                      dbAnalogOut,      // A: DAC analog output
		                      dbWelderOut,      // B: Welder output
		                      nDim);            // dimension

	if (nRet != RESULT_OK)
    {
        VERBOSE_ERROR("Cannot Get Curve Coefficient!\n");
        return nRet;
    }

	if (fabs(dbCoeff[0]) < MIN_WLDR_COEFF_A)
	{
		dbCoeff[0] = 0.0;
	}

	dbWelderCoeff[0] = dbCoeff[0];
	dbWelderCoeff[1] = dbCoeff[1];
	dbWelderCoeff[2] = dbCoeff[2];

	// validate Result
	co.dbA	= dbCoeff[0];
	co.dbB	= dbCoeff[1];
	co.dbC	= dbCoeff[2];

	// ax^2 + bx + c = y 
	// x : analog output command (0 ~ 10)
	// y : welder voltage

	// ax^2 + bx + c = y
	// ax^2 + bx + c-y = 0
	// set, d = b^2- 4a(c-y)
	// then, x = (-b + sqrt(d))/2a

	idx	= nDim - WELD_COEFF_COUNT;

	dbD           = pow(co.dbB, 2) - 4.0 * co.dbA * (co.dbC - dbWelderOut[idx]);
	dbNumerator   = -co.dbB + sqrt(fabs(dbD));

	if(fabs(co.dbA) > MIN_WLDR_COEFF_A)
    {
		dbTemp = dbNumerator / (2.0 * co.dbA);
    }
	else
    {
		dbTemp = (dbWelderOut[idx] - co.dbC) / co.dbB;
    }

	// must validate the calculated coefficient
	// ratio = (measured value - calculated value)/(measured value) * 100 (%)
	dbRatio		= 100.0 * (dbAnalogOut[idx]-dbTemp)/(dbAnalogOut[idx]);
    
    if(nOpt == VOLT_OUT)
    {
	    VERBOSE_MESSAGE("[Volt Calibration] ratio=%5.2lf \n", dbRatio);

    	VERBOSE_MESSAGE("[Volt Calibration] A=%lf, B=%lf, C=%lf \n", 
	    				dbWelderCoeff[0], dbWelderCoeff[1], dbWelderCoeff[2]);

        if (fabs(dbRatio) >= MIN_WLDR_COEFF_RATIO)
	    {
            VERBOSE_ERROR("\v[Volt] Invalid Coefficient Value\n"
                          "Ratio of measured to calculated(%5.2lf%%) is so Big!\n", fabs(dbRatio));
	    	return ERR_VOLT_RATIO_MEASURED_AND_CALCULATED;
	    }
    }
    else if(nOpt == CURRENT_OUT)
    {
        VERBOSE_MESSAGE("[Curr Calibration] ratio=%5.2lf \n", dbRatio);

    	VERBOSE_MESSAGE("[Curr Calibration] A=%lf, B=%lf, C=%lf \n", 
	    				dbWelderCoeff[0], dbWelderCoeff[1], dbWelderCoeff[2]);

        if (fabs(dbRatio) >= MIN_WLDR_COEFF_RATIO)
	    {
            VERBOSE_ERROR("\v[Volt] Invalid Coefficient Value\n"
                          "Ratio of measured to calculated(%5.2lf%%) is so Big!\n", fabs(dbRatio));
	    	return ERR_CURR_RATIO_MEASURED_AND_CALCULATED;
	    }
    }

    return RESULT_OK;
}

