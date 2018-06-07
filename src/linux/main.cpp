/***************************************************************************

    file                 : main.cpp
    created              : Sat Mar 18 23:54:30 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
    version              : $Id: main.cpp,v 1.14.2.3 2012/06/01 01:59:42 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdlib.h>

#include <GL/glut.h>

#include <tgfclient.h>
#include <client.h>

#include "linuxspec.h"
#include <raceinit.h>


#include <sys/shm.h> 
#define image_width 640
#define image_height 480
#include <iostream> 
#include <unistd.h> 


extern bool bKeepModules;
int key = 1234;
static void
init_args(int argc, char **argv, const char **raceconfig)
{
	int i;
	char *buf;
    
    setNoisy(false);
    setVersion("2013");

	i = 1;

	while(i < argc) {
		if(strncmp(argv[i], "-l", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetLocalDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-L", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetLibDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-D", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetDataDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-s", 2) == 0) {
			i++;
			SetSingleTextureMode();
		} else if (strncmp(argv[i], "-t", 2) == 0) {
		    i++;
		    if (i < argc) {
			long int t;
			sscanf(argv[i],"%ld",&t);
			setTimeout(t);
			printf("UDP Timeout set to %ld 10E-6 seconds.\n",t);
			i++;
		    }
		} else if (strncmp(argv[i], "-nodamage", 9) == 0) {
		    i++;
		    setDamageLimit(false);
		    printf("Car damages disabled!\n");
		} else if (strncmp(argv[i], "-nofuel", 7) == 0) {
		    i++;
		    setFuelConsumption(false);
		    printf("Fuel consumption disabled!\n");
		} else if (strncmp(argv[i], "-noisy", 6) == 0) {
		    i++;
		    setNoisy(true);
		    printf("Noisy Sensors!\n");
		} else if (strncmp(argv[i], "-ver", 4) == 0) {
		    i++;
		    if (i < argc) {
					setVersion(argv[i]);
		    		printf("Set version: \"%s\"\n",getVersion());
		    		i++;
		    }
		} else if (strncmp(argv[i], "-nolaptime", 10) == 0) {
		    i++;
		    setLaptimeLimit(false);
		    printf("Laptime limit disabled!\n");   
		} else if(strncmp(argv[i], "-k", 2) == 0) {
			i++;
			// Keep modules in memory (for valgrind)
			printf("Unloading modules disabled, just intended for valgrind runs.\n");
			bKeepModules = true;
#ifndef FREEGLUT
		} else if(strncmp(argv[i], "-m", 2) == 0) {
			i++;
			GfuiMouseSetHWPresent(); /* allow the hardware cursor */
#endif
		} else if(strncmp(argv[i], "-r", 2) == 0) {
			i++;
			*raceconfig = "";

			if(i < argc) {
				*raceconfig = argv[i];
				i++;
			}

			if((strlen(*raceconfig) == 0) || (strstr(*raceconfig, ".xml") == 0)) {
				printf("Please specify a race configuration xml when using -r\n");
				exit(1);
			}
		} else if(strncmp(argv[i], "--key", 5) == 0){
			char subbuff[4];
			memcpy( subbuff, argv[i]+6, 4 );			
			sscanf(subbuff, "%d", &key);
			i++;
		} else {
			i++;		/* ignore bad args */
		}
	}

#ifdef FREEGLUT
	GfuiMouseSetHWPresent(); /* allow the hardware cursor (freeglut pb ?) */
#endif
}
//zj

struct env_to_read{

	double steer;
	double brake;
	double accel;
	int gear;
	double clutch;

	double speed_x;
	double speed_y;
	double speed_z;	
	double track_angle;
	double track_pos;
	double rpm;
	double radius;
};
struct env_to_read_29{
	float angle_dqn;
	float track_dqn[19];
    float opponents[36];
    float focus[5];
	float track_pos_dqn;
	float speed_x_dqn;
	float speed_y_dqn;
	float speed_z_dqn;
	float wheel_dqn[4];
	float rpm_dqn;
	float damage;
    float curLapTime;
    float lastLapTime;
    float distFromStart;
    float distRaced;
    float fuel;
    int racePos;
    int gear;
    float z;
    float toleft;
    float toright;
    float radius;
};
struct env_to_write{

	bool is_restart;
	double steer;
	double brake;
	double accel;
	int gear;
	double clutch;
};
struct shared_use_st  
{  
    int written;
    uint8_t data[image_width*image_height*3];
    int pause;
    int zmq_flag;   
    int save_flag;  

	struct env_to_write env_write;
	struct env_to_read env_read;
	bool read_flag;
	bool is_hit_wall;
	bool is_finish;
	bool is_stuck;
	struct env_to_read_29 env_read_29;
	bool dqn_ready;
    char map_name[100];	
    char map_ok;	
};

int* pwritten = NULL;
uint8_t* pdata = NULL;
int* ppause = NULL;
int* pzmq_flag = NULL;
int* psave_flag = NULL;

bool* pis_restart_main_write = NULL;
double* psteer_main_write = NULL;
double* pbrake_main_write = NULL;
double* paccel_main_write = NULL;
int* pgear_main_write = NULL;
double* pclutch_main_write = NULL;



double* ptrack_angle_main_read=NULL;
bool* pis_hit_wall_main_read=NULL;
bool* pis_finish_main_read=NULL;
double* pspeed_x_main_read = NULL;
double* pspeed_y_main_read = NULL;
double* pspeed_z_main_read = NULL;
double* psteer_main_read = NULL;
double* pbrake_main_read = NULL;
double* paccel_main_read = NULL;
int* pgear_main_read = NULL;
double* pclutch_main_read = NULL;
double* ptrack_pos_main_read = NULL;
double* prpm_main_read = NULL;
double* ptrack_radius_main_read = NULL;
bool* pis_ready_main_read = NULL;
bool* pis_stuck_main_read = NULL;
char* pmap_name = NULL;
char* pmap_ok = NULL;
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
float* angle_dqn_main = NULL;
float* track_dqn_main = NULL;
float* opponents_main = NULL;
float* focus_main = NULL;
float* track_pos_dqn_main = NULL;
float* speed_x_dqn_main = NULL;
float* speed_y_dqn_main = NULL;
float* speed_z_dqn_main = NULL;
float* wheel_dqn_main = NULL;
float* rpm_dqn_main = NULL;
float* damage_main = NULL;
float* curLapTime_main = NULL;
float* lastLapTime_main = NULL;
float* distFromStart_main = NULL;
float* distRaced_main = NULL;
float* fuel_main = NULL;
int* racePos_main = NULL;
int* gear_main = NULL;
float* z_main = NULL;
float* toleft_main = NULL;
float* toright_main = NULL;
float* radius_main = NULL;

bool* is_ready_dqn_main = NULL;
bool is_sim_dqn_main = true;

void *shm = NULL;
/*
 * Function
 *	main
 *
 * Description
 *	LINUX entry point of TORCS
 *
 * Parameters
 *
 *
 * Return
 *
 *
 * Remarks
 *
 */
int
main(int argc, char *argv[])
{
	const char *raceconfig = "";
    //key = 1234;
	init_args(argc, argv, &raceconfig);
	struct shared_use_st *shared = NULL;
	int shmid; 
	printf("shared key is: %d\n",key);
    // establish memory sharing 
    shmid = shmget((key_t)key, sizeof(struct shared_use_st), 0666|IPC_CREAT);  
    if(shmid == -1)  
    {  
        fprintf(stderr, "shmget failed\n");  
        exit(EXIT_FAILURE);  
    }  
  
    shm = shmat(shmid, 0, 0);  
    if(shm == (void*)-1)  
    {  
        fprintf(stderr, "shmat failed\n");  
        exit(EXIT_FAILURE);  
    }  
    printf("\n********** Memory sharing started, attached at %X **********\n \n", shm);  
    // set up shared memory 
    shared = (struct shared_use_st*)shm;
  
    shared->written = 0;
    shared->pause = 0;
    shared->zmq_flag = 0;  
    shared->save_flag = 0;

    shared->env_write.is_restart = false;
 	shared->env_write.steer = 0;
	shared->env_write.brake = 0;
	shared->env_write.accel = 0;
	shared->env_write.gear = 0;	
	shared->env_write.clutch = 0;

	shared->env_read.speed_x = 0;
	shared->env_read.speed_y = 0;
	shared->env_read.speed_z = 0;
	shared->env_read.steer = 0;
	shared->env_read.brake = 0;
	shared->env_read.accel = 0;
	shared->env_read.gear = 0;	
	shared->env_read.clutch = 0;
	shared->env_read.track_angle = 0;	
	shared->env_read.track_pos = 0;
	shared->env_read.rpm = 0;
	shared->env_read.radius = 0;
	shared->read_flag = false;
	shared->is_hit_wall = false;	
	shared->is_finish = false;
	shared->is_stuck = false;

	shared->env_read_29.angle_dqn = 0;
	shared->env_read_29.track_pos_dqn = 0;
	shared->env_read_29.speed_x_dqn = 0;
	shared->env_read_29.speed_y_dqn = 0;
	shared->env_read_29.speed_z_dqn = 0;
	shared->env_read_29.rpm_dqn = 0;
	shared->env_read_29.damage = 0;
	shared->env_read_29.curLapTime = 0;
	shared->env_read_29.lastLapTime = 0;
	shared->env_read_29.distFromStart = 0;
	shared->env_read_29.distRaced = 0;
	shared->env_read_29.fuel = 0;
	shared->env_read_29.racePos = 0;
	shared->env_read_29.gear = 0;
	shared->env_read_29.z = 0;
	shared->env_read_29.toleft = 0;
	shared->env_read_29.toright = 0;
	shared->env_read_29.radius = 0;
	shared->dqn_ready = false;
	is_sim_dqn_main = true;
	
    pwritten=&shared->written;
    pdata=shared->data;
    ppause=&shared->pause;
    pzmq_flag = &shared->zmq_flag;
    psave_flag = &shared->save_flag;
	
    pis_restart_main_write = &(shared->env_write.is_restart);
	psteer_main_write = &(shared->env_write.steer);
	pbrake_main_write = &(shared->env_write.brake);
	paccel_main_write = &(shared->env_write.accel);
	pgear_main_write = &(shared->env_write.gear);
	pclutch_main_write = &(shared->env_write.clutch);

	pspeed_x_main_read = &(shared->env_read.speed_x);
	pspeed_y_main_read = &(shared->env_read.speed_y);
	pspeed_z_main_read = &(shared->env_read.speed_z);
	psteer_main_read = &(shared->env_read.steer);
	pbrake_main_read = &(shared->env_read.brake);
	paccel_main_read = &(shared->env_read.accel);
	pgear_main_read = &(shared->env_read.gear);
	pclutch_main_read = &(shared->env_read.clutch);
	ptrack_angle_main_read = &(shared->env_read.track_angle);

	ptrack_pos_main_read = &(shared->env_read.track_pos);
	prpm_main_read = &(shared->env_read.rpm);
	ptrack_radius_main_read = &(shared->env_read.radius);
	pis_ready_main_read = &(shared->read_flag);
	pis_hit_wall_main_read =  &(shared->is_hit_wall);	
	pis_finish_main_read = &(shared->is_finish);
	pis_stuck_main_read = &(shared->is_stuck);

	//angle_dqn_main = &(shared->env_read_29.angle_dqn);
	//track_dqn_main = shared->env_read_29.track_dqn;
    //opponents_main = shared->env_read.opponents;
	//track_pos_dqn_main = &(shared->env_read_29.track_pos_dqn);
	//speed_x_dqn_main = &(shared->env_read_29.speed_x_dqn);
	//speed_y_dqn_main = &(shared->env_read_29.speed_y_dqn);
	//speed_z_dqn_main = &(shared->env_read_29.speed_z_dqn);
	//wheel_dqn_main = shared->env_read_29.wheel_dqn;
	//rpm_dqn_main = &(shared->env_read_29.rpm_dqn);
	is_ready_dqn_main = &(shared->dqn_ready);

    angle_dqn_main = &(shared->env_read_29.angle_dqn);
    track_dqn_main = shared->env_read_29.track_dqn;
    opponents_main = shared->env_read_29.opponents;
    focus_main = shared->env_read_29.focus;
    track_pos_dqn_main = &(shared->env_read_29.track_pos_dqn);
    speed_x_dqn_main = &(shared->env_read_29.speed_x_dqn);
    speed_y_dqn_main = &(shared->env_read_29.speed_y_dqn);
    speed_z_dqn_main = &(shared->env_read_29.speed_z_dqn);
    wheel_dqn_main = shared->env_read_29.wheel_dqn;
    rpm_dqn_main = &(shared->env_read_29.rpm_dqn);
    damage_main = &(shared->env_read_29.damage);
    curLapTime_main = &(shared->env_read_29.curLapTime);
    lastLapTime_main = &(shared->env_read_29.lastLapTime);
    distFromStart_main = &(shared->env_read_29.distFromStart);
    distRaced_main = &(shared->env_read_29.distRaced);
    fuel_main = &(shared->env_read_29.fuel);
    racePos_main = &(shared->env_read_29.racePos);
    gear_main = &(shared->env_read_29.gear);
    z_main = &(shared->env_read_29.z);
    toleft_main = &(shared->env_read_29.toleft);
    toright_main = &(shared->env_read_29.toright);
    radius_main = &(shared->env_read_29.radius);
    pmap_name = shared->map_name;
    pmap_ok = &(shared->map_ok);
	printf("pis_restart_main_write:%p\n",(void*)pis_restart_main_write);


	// printf("argv: %d\n",argc);
	// for (int i = 0; i < )
	LinuxSpecInit();			/* init specific linux functions */

	if(strlen(raceconfig) == 0) {
		GfScrInit(argc, argv);	/* init screen */
		TorcsEntry();			/* launch TORCS */
		glutMainLoop();			/* event loop of glut */
	} else {
		// Run race from console, no Window, no OpenGL/OpenAL etc.
		// Thought for blind scripted AI training
		ReRunRaceOnConsole(raceconfig);
	}

	return 0;					/* just for the compiler, never reached */
}

