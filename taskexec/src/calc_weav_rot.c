// This is for the parameters for Ucell weaving. 
// Author : J.C. Hong
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

///////////////////////////////////////

#if defined(_MSC_VER)
#pragma warning(push, 4)
#endif

///////////////////////////////////////

#define ROT_TEST        0   // main() test

/////////////////////////////////////////////////////////////////////////////
//
//  _loc_check_weav_side()
//

static double _loc_check_weav_side(double dbPos)
{
    return (dbPos > 0.) ? 1. : -1.;
}

/////////////////////////////////////////////////////////////////////////////
//
//  calc_weav_rot()
//

#define WEAV_MODE_X     0   // travel by x-axis direction
#define WEAV_MODE_Y     1   // travel by y-axis direction
#define WEAV_MODE_Z     2   // travel by z-axis direction

#if 0
void calc_weav_rot(const double rgdbPosStart[3],    // input for start point
                   const double rgdbPosTarget[3],   // input for target point
                   double dbAngle,                  // input for weaving angle
                   double rgdbPlane[3], // output for yw plane vector
                   double* pdbRot)      // output for yw plane rotation
{
    double rgdbTravel[3];   // travel distance with sign
    double rgdbAbs[3];      // travel distance (absolute distance, no sign)

    int nMode;
    double dbSideSign;

    int i;

    ///////////////////////////////////
    //
    //  calc travel distance
    //

    for (i = 0; i < 3; i++)
    {
        rgdbTravel[i] = rgdbPosTarget[i] - rgdbPosStart[i];
        rgdbAbs[i] = fabs(rgdbTravel[i]);

        rgdbPlane[i] = 0;
    }

    ///////////////////////////////////
    //
    //  calc travel distance
    //

    if (rgdbAbs[0] >= rgdbAbs[1])
    {
        if (rgdbAbs[0] >= rgdbAbs[2])
        {
            // travel on the x-axis
            nMode = WEAV_MODE_X;
        }
        else
        {
            // travel to z-axis direction
            nMode = WEAV_MODE_Z;
        }
    }
    else
    {
        if (rgdbAbs[1] >= rgdbAbs[2])
        {
            // travel on the y-axis
            nMode = WEAV_MODE_Y;
        }
        else
        {
            // travel to z-axis direction
            nMode = WEAV_MODE_Z;
        }
    }

    ///////////////////////////////////
    //
    //
    //
    //

    switch (nMode)
    {
    // travel by z-axis following
    case WEAV_MODE_Z :
        // check start point Left/Right by y-axis center
        dbSideSign = _loc_check_weav_side(rgdbPosStart[1]);
        rgdbPlane[1] = dbSideSign;
        break;

    // travel by y-axis following (unused)
    case WEAV_MODE_Y :
        // check start point Left/Right by x-axis center
        dbSideSign = _loc_check_weav_side(rgdbPosStart[0]);
        rgdbPlane[0] = dbSideSign;
        break;

    // travel by x-axis following (unused)
    case WEAV_MODE_X :
        // check start point Lower/Upper by y-axis center
        dbSideSign = _loc_check_weav_side(rgdbPosStart[1]);
        rgdbPlane[2] = dbSideSign;
        break;

    // invalid
    default :
        dbSideSign = 0;
    }

    // weaving plane 'yw' rotation angle
    *pdbRot = dbAngle * dbSideSign;
}
#endif
// weaving-z
// weaving start pos(y) + -> yw(0,+1,0), +th
// weaving start pos(y) - -> yw(0,-1,0), -th

// weaving-y
// weaving dir(y) + -> yw(1,0,0), -th
// weaving dir(y) - -> yw(1,0,0), +th

// weaving-x
// weaving start pos(y) + -> yw(0,+1,0)
//       weaving dir(x) + -> +th
//       weaving dir(x) - -> -th
// weaving start pos(y) - -> yw(0,-1,0)
//       weaving dir(x) + -> -th
//       weaving dir(x) - -> +th
void calc_weav_rot(const double rgdbPosStart[3],    // input for start point
                   const double rgdbPosTarget[3],   // input for target point
                   double dbAngle,                  // input for weaving angle
                   double rgdbPlane[3], // output for yw plane vector
                   double* pdbRot)      // output for yw plane rotation
{
    double rgdbTravel[3];   // travel distance with sign
    double rgdbAbs[3];      // travel distance (absolute distance, no sign)

    int nMode;
    double dbSideSign;

    int i;

    ///////////////////////////////////
    //
    //  calc travel distance
    //

    for (i = 0; i < 3; i++)
    {
        rgdbTravel[i] = rgdbPosTarget[i] - rgdbPosStart[i];
        rgdbAbs[i] = fabs(rgdbTravel[i]);

        rgdbPlane[i] = 0;
    }

    ///////////////////////////////////
    //
    //  calc travel distance
    //

    if (rgdbAbs[0] >= rgdbAbs[1])
    {
        if (rgdbAbs[0] >= rgdbAbs[2])
        {
            // travel on the x-axis
            nMode = WEAV_MODE_X;
        }
        else
        {
            // travel to z-axis direction
            nMode = WEAV_MODE_Z;
        }
    }
    else
    {
        if (rgdbAbs[1] >= rgdbAbs[2])
        {
            // travel on the y-axis
            nMode = WEAV_MODE_Y;
        }
        else
        {
            // travel to z-axis direction
            nMode = WEAV_MODE_Z;
        }
    }

    ///////////////////////////////////
    //
    //
    //
    //

    switch (nMode)
    {
    // travel by z-axis following
    case WEAV_MODE_Z :
        // check start point Left/Right by y-axis center
        dbSideSign = _loc_check_weav_side(rgdbPosStart[1]);
        rgdbPlane[1] = dbSideSign;
        break;

    // travel by y-axis following (unused)
    case WEAV_MODE_Y :
        rgdbPlane[0] = 1.0;
        dbSideSign = (0 <= rgdbTravel[1])? -1.0 : 1.0;
        break;

    // travel by x-axis following (unused)
    case WEAV_MODE_X :
        // check start point Lower/Upper by y-axis center
        rgdbPlane[1] = _loc_check_weav_side(rgdbPosStart[1]);
        if(0 <= rgdbPlane[1])
        {
            dbSideSign = (0 <= rgdbTravel[0])? 1.0 : -1.0;
        }
        else
        {
            dbSideSign = (0 <= rgdbTravel[0])? -1.0 : 1.0;
        }
        break;

    // invalid
    default :
        dbSideSign = 0;
    }

    // weaving plane 'yw' rotation angle
    *pdbRot = dbAngle * dbSideSign;
}

/////////////////////////////////////////////////////////////////////////////

#if ROT_TEST

/////////////////////////////////////////////////////////////////////////////
//
//  _loc_disp_output
//
//

static void _loc_disp_output(const double rgdb[3], double dbRot)
{
    printf("yw = [%.1f, %.1f, %.1f], rot=%g\n",
           rgdb[0],
           rgdb[1],
           rgdb[2],
           dbRot);
}

/////////////////////////////////////////////////////////////////////////////
//
//  main()
//

int main()
{
    // travel by z-axis
#if 1
    // l : +x side travel by z-axis
    double rgdbLS[3] = {+30, 10, -100};
    double rgdbLE[3] = {+30, 10, +300};

    // r : -x side travel by z-axis
    double rgdbRS[3] = {-30, 10, -100};
    double rgdbRE[3] = {-30, 10, +300};

    // travel by y-axis
#elif 0
    // l : +x side travel by y-axis
    double rgdbLS[3] = {+30, +300, -100};
    double rgdbLE[3] = {+30, -300, -100};

    // r : -x side travel by y-axis
    double rgdbRS[3] = {-30, +300, -100};
    double rgdbRE[3] = {-30, -300, -100};

    // travel by x-axis
#else
    // u : +z side travel by x-axis
    double rgdbLS[3] = {-100, +300, +300};
    double rgdbLE[3] = {+300, +300, +300};

    // d : -z side travel by x-axis
    double rgdbRS[3] = {-100, +300, -100};
    double rgdbRE[3] = {+300, +300, -100};
#endif

    // result
    double yw[3];
    double rot;

    ///////////////////////////////////
    //
    //  left side sample
    //

    calc_weav_rot(rgdbLS, rgdbLE, 45, yw, &rot);
    _loc_disp_output(yw, rot);

    ///////////////////////////////////
    //
    //  right side sample
    //

    calc_weav_rot(rgdbRS, rgdbRE, 45, yw, &rot);
    _loc_disp_output(yw, rot);

    ///////////////////////////////////

    return EXIT_SUCCESS;
}
#endif  // ROT_TEST
