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

#include "get_sensors.h"

get_sensors::get_sensors(tTrack* track,  tCarElt* car, tSituation *s){
    __SENSORS_RANGE__ = 200;
    curTrack = track;
    curCar = car;
    curS = s;
    srand(time(NULL));
    float a__[19] = {-45 ,-19, -12 ,-7 ,-4 ,-2.5 ,-1.7 ,-1 ,-0.5, 0 ,0.5 ,1 ,1.7, 2.5 ,4 ,7 ,12 ,19, 45};
    for (int i = 0; i < 19; ++i) {
        trackSensAngle[i] = a__[i];
    }
    
    focusSens = new Sensors(car, 5);//ML
    for (int i = 0; i < 5; ++i) {//ML
        focusSens->setSensor(i,(car->_focusCmd)+i-2.0,200);//ML
    }//ML
    trackSens = new Sensors(car, 19);
    for (int i = 0; i < 19; ++i) {
        trackSens->setSensor(i,trackSensAngle[i],__SENSORS_RANGE__);
    }
    oppSens = new ObstacleSensors(36, curTrack, car, s, (int) __SENSORS_RANGE__);

    prevDist=-1;
}
void get_sensors::set_sensor(){

    dist_to_middle = 2*curCar->_trkPos.toMiddle/(curCar->_trkPos.seg->width);
    angle =  RtTrackSideTgAngleL(&(curCar->_trkPos)) - curCar->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI

//Update focus sensors' angle
    for (int i = 0; i < 5; ++i) {
        focusSens->setSensor(i,(curCar->_focusCmd)+i-2.0,200);
    }

    // update the value of track sensors only as long as the curCar is inside the track
    float trackSensorOut[19];
    float focusSensorOut[5];//ML
    if (dist_to_middle<=1.0 && dist_to_middle >=-1.0 )
    {
        trackSens->sensors_update();
        for (int i = 0; i < 19; ++i)
        {
            trackSensorOut[i] = trackSens->getSensorOut(i);
            if (getNoisy())
                trackSensorOut[i] *= normRand(1,__NOISE_STD__);
        }
        focusSens->sensors_update();//ML
        if ((curCar->_focusCD <= curCar->_curLapTime + curCar->_curTime)//ML Only send focus sensor reading if cooldown is over
            && (curCar->_focusCmd != 360))//ML Only send focus reading if requested by client
        {//ML
            for (int i = 0; i < 5; ++i)
            {
                focusSensorOut[i] = focusSens->getSensorOut(i);
                if (getNoisy())
                    focusSensorOut[i] *= normRand(1,__FOCUS_NOISE_STD__);
            }
            curCar->_focusCD = curCar->_curLapTime + curCar->_curTime + 1.0;//ML Add cooldown [seconds]
        }//ML
        else//ML
        {//ML
            for (int i = 0; i < 5; ++i)//ML
                focusSensorOut[i] = -1;//ML During cooldown send invalid focus reading
        }//ML
    }
    else
    {
        for (int i = 0; i < 19; ++i)
        {
            trackSensorOut[i] = -1;
        }
        for (int i = 0; i < 5; ++i)
        {
            focusSensorOut[i] = -1;
        }
    }
    
    // update the value of opponent sensors
    float oppSensorOut[36];
    oppSens->sensors_update(curS);
    for (int i = 0; i < 36; ++i)
    {
        oppSensorOut[i] = oppSens->getObstacleSensorOut(i);
        if (getNoisy())
            oppSensorOut[i] *= normRand(1,__OPP_NOISE_STD__);
    }

    float wheelSpinVel[4];
    for (int i=0; i<4; ++i)
    {
        wheelSpinVel[i] = curCar->_wheelSpinVel(i);
    }

    if (prevDist<0)
    {
    prevDist = curCar->race.distFromStartLine;
    }
    float curDistRaced = curCar->race.distFromStartLine - prevDist;
    prevDist = curCar->race.distFromStartLine;
    if (curDistRaced>100)
    {
    curDistRaced -= curTrack->length;
    }
    if (curDistRaced<-100)
    {
    curDistRaced += curTrack->length;
    }

    distRaced += curDistRaced;

    if (is_sim_dqn_main){
        *is_ready_dqn_main = false;
        *angle_dqn_main = angle;
        for(int j = 0 ;j < 19 ; j++)
            track_dqn_main[j] = trackSensorOut[j];
        for(int j = 0 ;j < 36 ; j++)
            opponents_main[j] = oppSensorOut[j];
        for(int j = 0 ;j < 5 ; j++)
            focus_main[j] = focusSensorOut[j];
        *track_pos_dqn_main = dist_to_middle;
        *speed_x_dqn_main = curCar->_speed_x  * 3.6;
        *speed_y_dqn_main = curCar->_speed_y  * 3.6;
        *speed_z_dqn_main = curCar->_speed_z  * 3.6;
        for(int j = 0 ;j < 4 ; j++)
            wheel_dqn_main[j] = wheelSpinVel[j];
        *rpm_dqn_main = curCar->_enginerpm * 10;

        *damage_main = curCar->_dammage;
        *curLapTime_main = curCar->_curLapTime;
        *lastLapTime_main = curCar->_lastLapTime;
        *distFromStart_main = curCar->race.distFromStartLine;
        *distRaced_main = distRaced;
        *fuel_main = curCar->_fuel;
        *racePos_main = curCar->race.pos;
        *gear_main = curCar->_gear;
        *z_main = curCar->_pos_Z - RtTrackHeightL(&(curCar->_trkPos));
    
        *is_ready_dqn_main = true;    
        is_sim_dqn_main = false;
    }
    if(*pis_restart_main_write){	
        curCar->ctrl.askRestart = true;
        *pis_restart_main_write = false;
        *is_ready_dqn_main = false;
    }
}

