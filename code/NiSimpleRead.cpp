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
#include <string.h>
extern "C"{
#include <wiringPi.h>
#include <softPwm.h>
};
#include <math.h>
//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../../Config/SamplesConfig.xml"
#define SAMPLE_XML_PATH_LOCAL "SamplesConfig.xml"
#define RANGE 100
#define PI 3.1458
#define OFFSET_DISTANCE 0.005
//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)\
      if(rc != XN_STATUS_OK)\
      {\
	printf("%s failed: %s\n", what, xnGetStatusString(rc));\
	return rc;\
      }

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace xn;
//---------------------------------------------------------------------------
// Structure for Waypoints
//---------------------------------------------------------------------------
struct waypoint{
  double x;
  double y;
};
//---------------------------------------------------------------------------
// Structure for Linked list to store waypoints from file
//---------------------------------------------------------------------------
struct node{
  int id;
  struct waypoint wp;
  struct node *next;
};

XnBool fileExists(const char *fn)
{
  XnBool exists;
  xnOSDoesFileExist(fn, &exists);
  return exists;
}
//---------------------------------------------------------------------------
// Function : getClosestPixel()
// Args     : (Pixel X start, Pixel Y start, Pixel X end, Pixel Y end, DepthMD) 
// Desc     : Returns the closest pixel value for the Zone
//---------------------------------------------------------------------------
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

//---------------------------------------------------------------------------
// Function : getGradient()
// Args     : (Number of Zones, Pixel X start, Pixel Y start, Pixel X end, 
//            Pixel Y end, DepthMD)
// Desc     : Used to detect if a gradient on the floor is detected.
//---------------------------------------------------------------------------
XnUInt32 getGradient(int zones, int x_start, int y_start, int x_end, int y_end, DepthMetaData &depthMD)
{
  XnUInt32 z[zones],diff[zones - 1];
  //divide it into 5 zones
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
//---------------------------------------------------------------------------
// Function : bounds()
// Args     : Latitude, Longitude, Waypoint one, Waypoint two
// Desc     : Check if current position is within bounds
//---------------------------------------------------------------------------
double bounds(double lat, double lon, waypoint one,waypoint two){
  //y = mx + c_origin;
  double c_upper, c_origin, c_lower, c_pos;
  double m = (two.y - one.y)/(two.x - one.x);
  double c_offset = OFFSET_DISTANCE * (1/cos(atan(m)));

  c_origin = one.y - (m * one.x);
  c_upper = c_origin + c_offset;
  c_lower = c_origin - c_offset;
  c_pos = lon - m*lat;
  
  //NORMALIZED VALUE
  double normalized = (c_pos - c_origin)/c_offset;
  return normalized;
}

//---------------------------------------------------------------------------
// Function : distance
// Args     : Latitude, Longitude, Waypoint
// Desc     : Get absolute scalar distance between two waypoints (waypoint one specified
//            as lat and long value)
//---------------------------------------------------------------------------
double distance(double lat, double lon, waypoint wp){
  return sqrt((wp.x - lat)*(wp.x - lat) + (wp.y - lon)*(wp.y - lon));
}

//---------------------------------------------------------------------------
// Function : find_slope
// Args     : Waypoint A, Waypoint B
// Desc     : Get the slope (angle) for a given track / path 
//---------------------------------------------------------------------------
double find_slope(waypoint A, waypoint B){
  double slope = (B.y - A.y)/(B.x - A.x);
  double theta = atan2((B.y - A.y),(B.x - A.x)) * 180.0 / PI;
  printf("slope : %f\r\n",slope);
  /*if(B.x < A.x){
    if(slope < 0)
      theta += 180;
    else
      theta -= 180;
      }*/
  return theta;
}

//---------------------------------------------------------------------------
// Function : direction_feedback()
// Args     : Waypoint Previous, Waypoint Last, Waypoint Next
// Desc     : Get feedback when waypoint is updated to indicate the direction of the
//            next waypoint, using vibration patterns
//---------------------------------------------------------------------------
void direction_feedback(waypoint prev, waypoint last, waypoint next){
  
  printf("Direction feedback is called\r\n");
  double theta_prev = find_slope(prev, last);
  printf("angle prev : %f\r\n",theta_prev);
  double theta_new = find_slope(last, next);
  printf("angle new : %f\r\n",theta_new);
  double theta_final = theta_new - theta_prev;
  printf("angle final : %f\r\n",theta_final);
  
  if(theta_final > 180.0)
    theta_final -= 360.0;
  else if(theta_final < -180.0)
    theta_final += 360.0;
  printf("angle final after if else : %f\r\n",theta_final);
  int loop = 0;
  while(loop <= 5000){
    softPwmWrite(1, 90);
    softPwmWrite(2, 90);
    softPwmWrite(3, 90);
    softPwmWrite(4, 90);
    loop++;
  }
  loop = 0;
  while(loop < 2500){
    softPwmWrite(1, 0);
    softPwmWrite(2, 0);
    softPwmWrite(3, 0);
    softPwmWrite(4, 0);
    loop++;
  }
  loop = 0;
  if(theta_final >= -180.0 && theta_final < -157.5){
    while(loop <= 5000){
      printf("Back Motor\r\n");
      softPwmWrite(4, 90);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Back Motor\r\n");
      softPwmWrite(4, 0);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Back Motor\r\n");
      softPwmWrite(4, 90);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Back Motor\r\n");
      softPwmWrite(4, 0);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Back Motor\r\n");
      softPwmWrite(4, 90);//Back Motor
      loop++;
    }
  }
  else if(theta_final >= -157.5 && theta_final < -112.5){
    while(loop <= 5000){
       printf("Back Motor and Right Motor\r\n");
      softPwmWrite(4, 90);//Back Motor
      softPwmWrite(1, 90);//Right Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Back Motor and Right Motor\r\n");
      softPwmWrite(4, 0);//Back Motor
      softPwmWrite(1, 0);//Right Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Back Motor and Right Motor\r\n");
      softPwmWrite(4, 90);//Back Motor
      softPwmWrite(1, 90);//Right Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Back Motor and Right Motor\r\n");
      softPwmWrite(4, 0);//Back Motor
      softPwmWrite(1, 0);//Right Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Back Motor and Right Motor\r\n");
      softPwmWrite(4, 90);//Back Motor
      softPwmWrite(1, 90);//Right Motor
      loop++;
    }
  }
  else if(theta_final >= -112.8 && theta_final < -67.5){
    while(loop <= 5000){
       printf("Right Motor\r\n");
      softPwmWrite(1, 90);//Right Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Right Motor\r\n");
      softPwmWrite(1, 0);//Right Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Right Motor\r\n");
      softPwmWrite(1, 90);//Right Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Right Motor\r\n");
      softPwmWrite(1, 0);//Right Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Right Motor\r\n");
      softPwmWrite(1, 90);//Right Motor
      loop++;
    }
  }
  else if(theta_final >= -67.5 && theta_final < -22.5){
    while(loop <= 5000){
      printf("Right Motor and Center Motor\r\n");
      softPwmWrite(1, 90);//Right Motor
      softPwmWrite(2, 90);//Center Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Right Motor and Center Motor\r\n");
      softPwmWrite(1, 0);//Right Motor
      softPwmWrite(2, 0);//Center Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Right Motor and Center Motor\r\n");
      softPwmWrite(1, 90);//Right Motor
      softPwmWrite(2, 90);//Center Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Right Motor and Center Motor\r\n");
      softPwmWrite(1, 0);//Right Motor
      softPwmWrite(2, 0);//Center Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Right Motor and Center Motor\r\n");
      softPwmWrite(1, 90);//Right Motor
      softPwmWrite(2, 90);//Center Motor
      loop++;
    }
  }
  else if(theta_final >= -22.5 && theta_final < 22.5){
    while(loop <= 5000){
       printf("Center Motor\r\n");
      softPwmWrite(2, 90);//Center Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Center Motor\r\n");
      softPwmWrite(2, 0);//Center Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Center Motor\r\n");
      softPwmWrite(2, 90);//Center Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Center Motor\r\n");
      softPwmWrite(2, 0);//Center Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Center Motor\r\n");
      softPwmWrite(2, 90);//Center Motor
      loop++;
    }
  }
  else if(theta_final >= 22.5 && theta_final < 67.5){
    while(loop <= 5000){
       printf("Center Motor and Left Motor\r\n");
      softPwmWrite(2, 90);//Center Motor
      softPwmWrite(3, 90);//Left Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Center Motor and Left Motor\r\n");
      softPwmWrite(2, 0);//Center Motor
      softPwmWrite(3, 0);//Left Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Center Motor and Left Motor\r\n");
      softPwmWrite(2, 90);//Center Motor
      softPwmWrite(3, 90);//Left Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Center Motor and Left Motor\r\n");
      softPwmWrite(2, 0);//Center Motor
      softPwmWrite(3, 0);//Left Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Center Motor and Left Motor\r\n");
      softPwmWrite(2, 90);//Center Motor
      softPwmWrite(3, 90);//Left Motor
      loop++;
    }
  }
  else if(theta_final >= 67.5 && theta_final < 112.5){
    while(loop <= 5000){
       printf("Left Motor\r\n");
      softPwmWrite(3, 90);//Left Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Left Motor\r\n");
      softPwmWrite(3, 0);//Left Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Left Motor\r\n");
      softPwmWrite(3, 90);//Left Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Left Motor\r\n");
      softPwmWrite(3, 0);//Left Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Left Motor\r\n");
      softPwmWrite(3, 90);//Left Motor
      loop++;
    }
  }
  else if(theta_final >= 112.5 && theta_final < 157.5){
    while(loop <= 5000){
      printf("Left Motor and Back Motor\r\n");
      softPwmWrite(3, 90);//Left Motor
      softPwmWrite(4, 90);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Left Motor and Back Motor\r\n");
      softPwmWrite(3, 0);//Left Motor
      softPwmWrite(4, 0);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Left Motor and Back Motor\r\n");
      softPwmWrite(3, 90);//Left Motor
      softPwmWrite(4, 90);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Left Motor and Back Motor\r\n");
      softPwmWrite(3, 0);//Left Motor
      softPwmWrite(4, 0);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Left Motor and Back Motor\r\n");
      softPwmWrite(3, 90);//Left Motor
      softPwmWrite(4, 90);//Back Motor
      loop++;
    }
  }
  else if(theta_final >= 157.5 && theta_final <= 180.0){
    while(loop <= 5000){
      printf("Back Motor\r\n");
      softPwmWrite(4, 90);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Back Motor\r\n");
      softPwmWrite(4, 0);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Back Motor\r\n");
      softPwmWrite(4, 90);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 2500){
      printf("Back Motor\r\n");
      softPwmWrite(4, 0);//Back Motor
      loop++;
    }
    loop = 0;
    while(loop <= 5000){
      printf("Back Motor\r\n");
      softPwmWrite(4, 90);//Back Motor
      loop++;
    }
  }
  softPwmWrite(1, 0);
  softPwmWrite(2, 0);
  softPwmWrite(3, 0);
  softPwmWrite(4, 0);
}

//---------------------------------------------------------------------------
// Function : update_waypoint()
// Args     : Linked List Pointer, Last Waypoint, Next Waypoint
// Desc     : Update waypoint when one set of waypoints have been traversed
//---------------------------------------------------------------------------
//void update_waypoint(/*struct node *ptr, waypoint *last, waypoint *next*/){
  //waypoint prev = *last;
  //*last = ptr->wp;
  //*next = ptr->next->wp;
  
  //direction_feedback(prev, *last, *next);
//} 

int main()
{ 
  if (wiringPiSetup () == -1)
    exit (1) ;
  
  pinMode (6, INPUT) ; 
  softPwmCreate(1, 0, RANGE);
  softPwmCreate(2, 0, RANGE);
  softPwmCreate(3, 0, RANGE);
  softPwmCreate(4, 0, RANGE);
  
  int count = 0;//Counter for feedback 
  while ( !xnOSWasKeyboardHit()){//Main While Loop
    
    if(digitalRead (6) == 0){//Check if GPS is turned on
      /*------------------------------------------------------------------*/
      /*-------------------Read waypoints from text file------------------*/
      /*-------------------and store into a linked list-------------------*/
      /*------------------------------------------------------------------*/
      char waypointline[100];
      int id = 0;
      FILE *waypoint_file;
      waypoint_file = fopen("/home/pi/data/waypoints.txt","r");
      double wplat[40], wplon[40], bound_val = 0.0;      
      while(fgets(waypointline, 100, waypoint_file) != NULL){
	char *tok;
	tok = strtok(waypointline, ",");
	wplat[id] = atof(tok);
	printf("lat - %f\r\n",wplat[id]);
	while (tok != NULL){
	  if(atof(tok) != wplat[id]){
	    wplon[id] = atof(tok);
	    printf("lon - %f\r\n",wplon[id]);
	  }
	  tok = strtok(NULL,",");
	}
	id++;
      }
      //printf("HEAD : %f and %f\r\n", head->wp.x, head->wp.y);
      fclose(waypoint_file);
      /*------------------------------------------------------------------*/
      /*------------------------------------------------------------------*/
      /*------------------------------------------------------------------*/

      char pointline[100];
      int pid = 0;
      FILE *point_file;
      point_file = fopen("/home/pi/data/wpoint.txt","r");
      double wlat[50000], wlon[50000];//, bound_val = 0.0;
      while(fgets(pointline, 90, point_file) != NULL){
        char *tok;
        tok = strtok(pointline, ",");
        wlat[pid] = atof(tok);
        printf("lat - %f\r\n",wlat[pid]);
        while (tok != NULL){
          if(atof(tok) != wlat[pid]){
            wlon[pid] = -1 * atof(tok);
            printf("lon - %f\r\n",wlon[pid]);
          }
          tok = strtok(NULL,",");
        }
        pid++;
      }
      printf("HEAD : and \r\n");
      fclose(point_file);

      /*------------------------------------------------------------------*/
      /*------------------------------------------------------------------*/
      /*------------------------Read GPS DATA-----------------------------*/
      /*------------------------------------------------------------------*/
      /*------------------------------------------------------------------*/
      
      //Read from GPSPIPE, etc. GPS related stuff
      system("sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock");
      system("sudo echo \" \" > /home/pi/data_buffer.txt");
      system("sudo gpspipe -o /home/pi/pipe.txt -r &");
      //struct node *ptr = head;
      //Set up wiringPi to enable PWM
      /*  if (wiringPiSetup () == -1)
	exit (1) ;
      */
            
      int flag,i = 0;
      
      waypoint last,next;
      int id_wp = 0;
      last.x = wplat[id_wp];
      last.y = wplon[id_wp];
      next.x = wplat[++id_wp];//id_wp = 1
      next.y = wplon[id_wp];
      /*
      last.x = ptr->wp.x;
      last.y = ptr->wp.y;
      next.x = ptr->next->wp.x;
      next.y = ptr->next->wp.y;
      */
      //printf("last.x = %f and last.y = %f\tnext.x = %f and next.y = %f\r\n",last.x, last.y, next.x, next.y);
      double dist_last = distance(last.x, last.y, next);
      FILE *DataLog;
      
      DataLog=fopen("/home/pi/data/Data_Log.txt","a");
      fprintf(DataLog,"\r\nNEW ENTRY\r\n");
      fprintf(DataLog,"Time\tLatitude\tLongitutde\tSatellites\tOut of Bounds\tProximity\r\n");
      fclose(DataLog);
      int c = 0;
      int ip = 0;
      while(digitalRead (6) == 0){
	flag = 0;
	//softPwmWrite(2, 0);
	double lat, lon;
	int num_sat, tim;
	char line[100], bool_bounds;
	
	system("sudo tail /home/pi/pipe.txt --lines=2 > /home/pi/data_buffer.txt");
	FILE *file;
	file=fopen("/home/pi/data_buffer.txt","rt");
	
	while(fgets(line, 150, file) != NULL && !flag){
	  char *tokenized, latitude[40], time_s[40], longitude[40], sat_s[40];
	  int tok_cnt = 1;
	  int check  = 0;
	  tokenized = strtok(line, ",");
	  if(tokenized != NULL && !strcmp(tokenized,"$GPGGA")){ 
	    check = 1;
	  }
	  while (tokenized != NULL && check){ 
	    if(tok_cnt == 2 && tokenized[0]!='*'){
	      strcpy(time_s, tokenized);
	      tim = atoi(time_s);  
	    }
	    if(tok_cnt == 3 && tokenized[0]!='*'){
	      strcpy(latitude, tokenized);
	      lat = atof(latitude);
	    }
	    if(tok_cnt == 5 && tokenized[0]!='*'){
	      strcpy(longitude, tokenized);
	      lon = atof(longitude);
	    }
	    if(tok_cnt ==8 && tokenized[0]!='*'){
	      strcpy(sat_s, tokenized);
	      num_sat = atoi(sat_s);
	    }
		    
	    tokenized = strtok (NULL, ",");
	    tok_cnt++;
	  } // End of while (tokenized != Null)
	  flag = 1;
	} // End of while (fgets)
	
	fclose(file);
	//if(c % 10 ==s ){
	//printf("ip : %d\r\n",ip);
	lat = wlat[ip];
	lon = wlon[ip];
	  
	  //}
	bound_val = bounds(lat,lon,last,next);
	//bound_val = 0.3;
	//IMPLEMENT THIS : Check for what side is right or left based on direction
	if(bound_val < 1.0 && bound_val >= 0.5){
	  // softPwmWrite(1,(int)(bound_val * 100));
	  bool_bounds = 'T';
	}
	else if(bound_val <= -0.5 && bound_val > -1.0){
	  // softPwmWrite(3,(int)(bound_val * -1 * 100));
	  bool_bounds = 'T';
	}
	else if(bound_val >= 1.0){
	  //softPwmWrite(1,100);
	  bool_bounds = 'L';
	}
	else if(bound_val <= -1.0){
	  //softPwmWrite(3,99);
	  bool_bounds = 'R';
	}
	else{
	  //softPwmWrite(1,0);
	  //softPwmWrite(3,0);
	  bool_bounds = 'T';
	}
	
	if(bool_bounds == 'T'){
	  softPwmWrite(1,0);
	  softPwmWrite(3,0);
	}
	else if(bool_bounds == 'L'){
	  softPwmWrite(1,90);
	  softPwmWrite(2, 0);
	}
	else if(bool_bounds == 'R'){
	  softPwmWrite(3,90);
	  softPwmWrite(2, 0);
	}

	double proximity = distance(lat,lon,next);
	//Check if navigating in correct direction
	if(count >= 125){//250 loops = 20 secs
	  int loop = 0;
	  if(proximity < dist_last){
	    //Right direction : vibrate center motor
	    softPwmWrite(2,90);
	    while(loop <5000){
	      printf("Correct Direction \r\n");
	      loop++;
	    }
	  }
	  else{
	    //Wrong direction : vibrate back motor
	    softPwmWrite(4,90);
	    while(loop < 5000){
	      printf("Wrong Direction \r\n");
	      loop++;
	    }
	  }
	  softPwmWrite(2,0);
	  softPwmWrite(4,0);
	  dist_last = proximity;
	  count = 0;
	}
	//Check if Point B has been reached
	/*if(count == 100){
	  printf("proximity change\r\n");
	  proximity = 0.003;
	  }*/
	if (proximity < OFFSET_DISTANCE){  
	  waypoint prev = last;
	  last = next;
	  next.x = wplat[++id_wp];
	  next.y = wplon[id_wp];
	  //update_waypoint(++ptr, &last, &next);
	  direction_feedback(prev, last, next);
	}

	DataLog = fopen("/home/pi/data/Data_Log.txt","a");
	fprintf(DataLog, "%d\t%f\t%f\t%d\t%c\t%f\r\n", tim,lat,-1*lon,num_sat,bool_bounds,proximity);
	fclose(DataLog);
	
	printf("%c\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%f\r\n", bool_bounds, tim, bound_val, lat, lon, last.x, last.y, next.x, next.y, num_sat, proximity);
	
	count++;
	i++;
	c++;
	ip++;
      }
      //stuct node *del = (struct node *)malloc(sizeof(struct node));
      //del = head;
      softPwmWrite(1,0);
      softPwmWrite(2,0);
      softPwmWrite(3,0);
      softPwmWrite(4,0);
      
      system("sudo killall gpsd");
      system("sudo rm -f /home/pi/*.txt");
    }
    
    else if(digitalRead (6) == 1){//Check if Asus Xtion is switched on
      XnStatus nRetVal = XN_STATUS_OK;
      Context context;
      ScriptNode scriptNode;
      EnumerationErrors errors;
      XnUInt32 min_z1, min_z2, min_z3, maxGrad, distVal;
      const char *fn = NULL;
      if(fileExists(SAMPLE_XML_PATH)) fn = SAMPLE_XML_PATH;
      else if (fileExists(SAMPLE_XML_PATH_LOCAL)) fn = SAMPLE_XML_PATH_LOCAL;
      else {
	printf("Could not find '%s' nor '%s'. Aborting.\n" , SAMPLE_XML_PATH, SAMPLE_XML_PATH_LOCAL);
	return XN_STATUS_ERROR;
	//goto end;
      }
      printf("Reading config from: '%s'\n", fn);
      nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);
      if (nRetVal == XN_STATUS_NO_NODE_PRESENT){
	XnChar strError[1024];
	errors.ToString(strError, 1024);
	printf("%s\n", strError);
	return (nRetVal);
	//goto end;
      }
      else if (nRetVal != XN_STATUS_OK){
	printf("Open failed: %s\n", xnGetStatusString(nRetVal));
	return (nRetVal);
	//goto end;
      }
      DepthGenerator depth;
      nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
      CHECK_RC(nRetVal, "Find depth generator");
      XnFPSData xnFPS;
      nRetVal = xnFPSInit(&xnFPS, 180);
      CHECK_RC(nRetVal, "FPS Init");
      DepthMetaData depthMD;
      //if(wiringPiSetup() == -1)
      //exit(1);
      
      /*softPwmCreate(1, 0, RANGE);
      softPwmCreate(2, 0, RANGE);
      softPwmCreate(3, 0, RANGE);
      */
      while(digitalRead (6) == 1){
	//Asus Xtion Code
	nRetVal = context.WaitOneUpdateAll(depth);
	if (nRetVal != XN_STATUS_OK){
	  printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
	  continue;
	}
	xnFPSMarkFrame(&xnFPS);
	depth.GetMetaData(depthMD);
	const XnDepthPixel* pDepthMap = depthMD.Data();
	int XRes = depthMD.XRes();
	int YRes = depthMD.YRes();
	//To find closest pixel value in Zone 1, Zone 2 and Zone 3
	min_z1    = getClosestPixel(  0        , 0, (2* XRes / 5)    , YRes, depthMD);
	min_z2    = getClosestPixel( (1 * XRes / 5), 0, (4 * XRes / 5), YRes, depthMD);
	min_z3    = getClosestPixel( (3 * XRes / 5), 0,  XRes         , YRes, depthMD);
	
	double in_low = 600;
	double in_high = 3000;
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
	//XnUInt32 pwm_val1 = ( (922.0) / ((900.0)*(900.0)*(900.0)) ) * ((1500.0 - min_z1) * (1500.0 - min_z1) * (1500.0 - min_z1)) + 51.0;
	if (pwm_val1 < out_low)
	  pwm_val1 = 0;  // if object too far, set DUTY CYCLE to 0
	if (min_z1 == 9000.0)
	  pwm_val1 = out_high; //if object too close, set DUTY CYCLE to max (here, 95%)
	if (min_z1 < 600)
	  pwm_val1 = out_high;
	// Zone 2 - Center (pin )
	//XnUInt32 pwm_val2 = ( (922.0) / ((900.0)*(900.0)*(900.0)) ) * ((1500.0 - min_z2) * (1500.0 - min_z2) * (1500.0 - min_z2)) + 51.0;
	if (pwm_val2 < out_low)
	  pwm_val2 = 0;  // if object too far, set DUTY CYCLE to 0
	if (min_z2 == 9000.0)
	  pwm_val2 = out_high; //if object too close, set DUTY CYCLE to max (here, 95%)
	if (min_z2 < 600)
	  pwm_val2 = out_high;
	// Zone 3 - Right side (pin )
	//XnUInt32 pwm_val3 = ( (922.0) / ((900.0)*(900.0)*(900.0)) ) * ((1500.0 - min_z3) * (1500.0 - min_z3) * (1500.0 - min_z3)) + 51.0;
	if (pwm_val3 < out_low)
	  pwm_val3 = 0;  // if object too far, set DUTY CYCLE to 0
	if (min_z3 == 9000.0)
	  pwm_val3 = out_high; //if object too close, set DUTY CYCLE to max (here, 95%)
	if (min_z3 < 600)
	  pwm_val3 = out_high;
	
	pwm_val1 = ((pwm_val1 - out_low) / (1.0 * out_diff)) * 100.0;
	pwm_val2 = ((pwm_val2 - out_low) / (1.0 * out_diff)) * 100.0;
	pwm_val3 = ((pwm_val3 - out_low) / (1.0 * out_diff)) * 100.0;
	
	softPwmWrite(1,(int)pwm_val1);//Zone 1 left Side  GPIO 1
	softPwmWrite(2,(int)pwm_val2);//Zone 2 Middle     GPIO 2
	softPwmWrite(3,(int)pwm_val3);//Zone 3 Right Side GPIO 3
	
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
	    printf("Frame %d closest point for Zone2 is: %u. FPS: %f\n", depthMD.FrameID(), min_z2, xnFPSCalc(&xnFPS));
	  }
      }
      
      softPwmWrite(1,0);
      softPwmWrite(2,0);
      softPwmWrite(3,0);
      
      depth.Release();
      scriptNode.Release();
      context.Release();
    }
    //end: 
  }
  return 0;
}

