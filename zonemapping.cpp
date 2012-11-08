/****************************************************************************
*                                                                           *
*  OpenNI 1.x Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>
extern "C"{
#include <wiringPi.h>
#include <softPwm.h>
};
//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../../Config/SamplesConfig.xml"
#define SAMPLE_XML_PATH_LOCAL "SamplesConfig.xml"
#define RANGE 100

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace xn;

XnBool fileExists(const char *fn)
{
	XnBool exists;
	xnOSDoesFileExist(fn, &exists);
	return exists;
}

XnUInt32 getClosestPixel(int x_start, int y_start, int x_end, int y_end, DepthMetaData &depthMD)
{
  XnUInt32 depth, smallest;
  int i,j;
  smallest = 9000;

  for (i = x_start ; i < x_end ; ++i)
    {

      for (j = y_start ; j < y_end ; ++j)
        {
	  depth = depthMD(i,j);
          if (depth < smallest && depth)
            smallest = depth;
        }
    }
 
  return smallest;
}

XnUInt32 getGradient(int zones, int x_start, int y_start, int x_end, int y_end, DepthMetaData &depthMD)
{
  XnUInt32 z[zones],diff[zones - 1];
  int section = (y_end - y_start) / zones;
  int i;

  for (i = 1 ; i <= zones ; i++)
    {
    	z[i-1] = getClosestPixel(x_start, y_start, x_end, (y_start + (i * section)), depthMD);
  	}

  for (i = 0 ; i < zones - 1 ; i++)
    {
      if(z[i + 1] >= z[i])
		diff[i] = z[i + 1] - z[i];
      else
		diff[i] = z[i] - z[i + 1];
    }

  XnUInt32 largest = diff[0];
  
  for (i = 1 ; i < zones - 1 ; i++)
    {
      if (diff[i] > largest)
		{
	  	largest = diff[i];
		}
    }
  return largest;
}

int main()
{
	XnStatus nRetVal = XN_STATUS_OK;

	Context context;
	ScriptNode scriptNode;
	EnumerationErrors errors;

    XnUInt32 min_z1, min_z2, min_z3, maxGrad, distVal;

	const char *fn = NULL;
	if	(fileExists(SAMPLE_XML_PATH)) fn = SAMPLE_XML_PATH;
	else if (fileExists(SAMPLE_XML_PATH_LOCAL)) fn = SAMPLE_XML_PATH_LOCAL;
	else {
		printf("Could not find '%s' nor '%s'. Aborting.\n" , SAMPLE_XML_PATH, SAMPLE_XML_PATH_LOCAL);
		return XN_STATUS_ERROR;
	}
	printf("Reading config from: '%s'\n", fn);
	nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);

	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (nRetVal);
	}
	else if (nRetVal != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		return (nRetVal);
	}

	DepthGenerator depth;
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
	CHECK_RC(nRetVal, "Find depth generator");

	XnFPSData xnFPS;
	nRetVal = xnFPSInit(&xnFPS, 180);
	CHECK_RC(nRetVal, "FPS Init");

	DepthMetaData depthMD;

	//Initialize WiringPi
	if(wiringPiSetup() == -1)
		exit(1);

	//Enable SoftPWM on pin 1,2 and 3
	softPwmCreate(1, 0, RANGE);
	softPwmCreate(2, 0, RANGE);
	softPwmCreate(3, 0, RANGE);

	while (!xnOSWasKeyboardHit())
	{
		nRetVal = context.WaitOneUpdateAll(depth);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
			continue;
		}

		xnFPSMarkFrame(&xnFPS);

		depth.GetMetaData(depthMD);
		const XnDepthPixel* pDepthMap = depthMD.Data();
		int XRes = depthMD.XRes();
		int YRes = depthMD.YRes();

		//To find closest pixel value in Zone 1, Zone 2 and Zone 3
		min_z1    = getClosestPixel(  0        , 0, (XRes / 2)    , YRes, depthMD);
		min_z2    = getClosestPixel( (XRes / 4), 0, (3 * XRes / 4), YRes, depthMD);
		min_z3    = getClosestPixel( (XRes / 2), 0,  XRes         , YRes, depthMD);

		double in_low = 600;
		double in_high = 2000;
		double in_diff = in_high - in_low;
		double out_low = 51;
		double out_high = 973;
		double out_diff = out_high - out_low;

		distVal = min_z1;
		XnUInt32 pwm_val1 = ( (out_diff) / ((in_diff)*(in_diff)*(in_diff)) ) * ((in_high - distVal) * (in_high - distVal) * (in_high - distVal)) + out_low;
		distVal = min_z2;
		XnUInt32 pwm_val2 = ( (out_diff) / ((in_diff)*(in_diff)*(in_diff)) ) * ((in_high - distVal) * (in_high - distVal) * (in_high - distVal)) + out_low;
		distVal = min_z3;
		XnUInt32 pwm_val3 = ( (out_diff) / ((in_diff)*(in_diff)*(in_diff)) ) * ((in_high - distVal) * (in_high - distVal) * (in_high - distVal)) + out_low;

		// Zone 1 - Left side (pin )

		if (pwm_val1 < out_low)
			pwm_val1 = 0;  		 // if object too far, set DUTY CYCLE to 0
		if (min_z1 == 9000.0)
			pwm_val1 = out_high; //if object too close, set DUTY CYCLE to max (here, 95%)
		if (min_z1 < 600)
			pwm_val1 = out_high;

		// Zone 2 - Center (pin )

		if (pwm_val2 < out_low)
			pwm_val2 = 0;  		 // if object too far, set DUTY CYCLE to 0
		if (min_z2 == 9000.0)
			pwm_val2 = out_high; //if object too close, set DUTY CYCLE to max (here, 95%)
		if (min_z2 < 600)
			pwm_val2 = out_high;

		// Zone 3 - Right side (pin )

		if (pwm_val3 < out_low)
			pwm_val3 = 0;  		 // if object too far, set DUTY CYCLE to 0
		if (min_z3 == 9000.0)
			pwm_val3 = out_high; //if object too close, set DUTY CYCLE to max (here, 95%)
		if (min_z3 < 600)
			pwm_val3 = out_high;

		pwm_val1 = ((pwm_val1 - out_low) / (1.0 * out_diff)) * 100.0;
		pwm_val2 = ((pwm_val2 - out_low) / (1.0 * out_diff)) * 100.0;
		pwm_val3 = ((pwm_val3 - out_low) / (1.0 * out_diff)) * 100.0;

		softPwmWrite(1,(int)pwm_val1);
		softPwmWrite(2,(int)pwm_val2);
		softPwmWrite(3,(int)pwm_val3);

		if ( (depthMD.FrameID() % 30) == 0)
		{
			printf("Frame %d", depthMD.FrameID());
			printf("\n");

			printf("Zone 1 value is %u \t", pwm_val1);
			printf("Zone 2 value is %u \t", pwm_val2);
			printf("Zone 3 value is %u \n", pwm_val3);

			printf("Zone1 min_dis   %u \t", min_z1);
			printf("Zone2 min_dis   %u \t", min_z2);
			printf("Zone3 min_dis   %u \n", min_z3);


		//To find a gradient value for the floor
		//maxGrad = getGradient( 5, 0, (YRes/2) + 1, depthMD.XRes(), depthMD.YRes(), depthMD);
		//printf("Frame %d max gradient for Floor is: %u. FPS: %f\n\n", depthMD.FrameID(), maxGrad, xnFPSCalc(&xnFPS));
		}

	}

	softPwmWrite(1,0);
	softPwmWrite(2,0);
	softPwmWrite(3,0);

	depth.Release();
	scriptNode.Release();
	context.Release();



	return 0;
}
