/***************************************************************************

    file                 : scr_server.cpp
    copyright            : (C) 2007 Daniele Loiacono

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef GET_SENSORS_H_
#define GET_SENSORS_H_
#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <ctime>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include <sensors.h>
#include <SimpleParser.h>
#include <CarControl.h>
#include <ObstacleSensors.h>

#define NBBOTS 10

#define RACE_RESTART 1
//#define __STEP_LIMIT__ 10000
//#define __DISABLE_RESTART__
//#define __PRINT_RACE_RESULTS__

#define __FOCUS_RANGE__ 200

/*** Noise definitions ***/
#define __NOISE_STD__ 0.1
#define __OPP_NOISE_STD__ 0.02
#define __FOCUS_NOISE_STD__ 0.01

#ifdef __PRINT_RACE_RESULTS__
static tdble bestLap[NBBOTS];
static tdble damages[NBBOTS];
static tdble totalTime[NBBOTS];
static int position[NBBOTS];
static int curPosition=0;
static int bonusBest;
static int bonusDamage;
static char *trackName;
#endif

extern float* angle_dqn_main;
extern float* track_dqn_main;
extern float* opponents_main;
extern float* focus_main;
extern float* track_pos_dqn_main;
extern float* speed_x_dqn_main;
extern float* speed_y_dqn_main;
extern float* speed_z_dqn_main;
extern float* wheel_dqn_main;
extern float* rpm_dqn_main;
extern float* damage_main;
extern float* curLapTime_main;
extern float* lastLapTime_main;
extern float* distFromStart_main;
extern float* distRaced_main;
extern float* fuel_main;
extern int* racePos_main;
extern int* gear_main;
extern float* z_main;

extern bool* pis_restart_main_write;
extern double* psteer_main_write;
extern double* pbrake_main_write;
extern double* paccel_main_write;
extern int* pgear_main_write;
extern double* pclutch_main_write;
extern bool* is_ready_dqn_main;
extern bool is_sim_dqn_main;

class get_sensor{
    private:
        double __SENSORS_RANGE__;
        Sensors *trackSens;
        ObstacleSensors *oppSens;
        Sensors *focusSens;//ML
        float trackSensAngle[19];
        tdble prevDist;
        tdble distRaced;
        tTrack	*curTrack;
        float dist_to_middle;
        float angle;
        tCarElt* curCar;
        tSituation *curS;

        double normRand(double avg,double std){
        	 double x1, x2, w, y1, y2;
        
        	    do {
        	            x1 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
        	            x2 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
        	            w = x1 * x1 + x2 * x2;
        	    } while ( w >= 1.0 );
        
        	    w = sqrt( (-2.0 * log( w ) ) / w );
        	    y1 = x1 * w;
        	    y2 = x2 * w;
        	    return y1*std + avg;
        }

    public:
        get_sensor(tTrack* track,  tCarElt* car, tSituation *s);

        void set_sensor();
};
#endif
