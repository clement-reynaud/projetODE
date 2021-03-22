/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

 /*
 buggy with suspension.
 this also shows you how to use geom groups.
 */

#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h> 
#include <sys/socket.h>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "math.h"
#include <cmath>
#include "projetODE.h"
#include <random>
#include <algorithm>
#include <iterator>
#include "creationBuggy.h"
#include "deplaceBuggy.h"
#include "projetODE.h"
#include "client.h"
#include "terrain.h"
#include "SolModifie.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

 // select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawTriangle dsDrawTriangleD
#endif

// some constants
#define LENGTH 0.7	// chassis length
#define WIDTH 0.5	// chassis width
#define HEIGHT 0.2	// chassis height
#define RADIUS 0.3	// wheel radius
#define CMASS 1		// chassis mass
#define WMASS 0.2	// wheel mass
#define STARTZ 2	// starting height of chassis

//static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };

//Turret def
//canon
#define TURRRADIUS    0.1
#define TURRLENGTH    0.6

//box
#define BOXLENGTH 0.15    // box length
#define BOXWIDTH 0.15    // box width
#define BOXHEIGHT 0.3    // box height
#define BMASS 0.1        // box mass

//Bullet
#define SPHERERADIUS 0.4

static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };

Buggy buggy[NB_BUGGY];
Turret turr[NB_BUGGY];
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static dGeomID ground_box;

//custom

dBodyID boxBodies[NB_BOX];
dGeomID boxGeoms[NB_BOX];;

static dGeomID obstacle[4];

static dBodyID sphbody;
static dGeomID sphgeom;

dGeomID gheight;
static dGeomID ground;

// things that the user controls

static dReal speed = 0, steer = 0;	// user commands
dReal speedBuggy1 = 0;
dReal speedBuggy2 = 0;
dReal steerBuggy1 = 0;
dReal steerBuggy2 = 0;

static bool lock_cam1 = true;
static bool lock_cam2 = false;
static dReal cannon_angle = 0, cannon_elevation = -1.2;

bool tp;

static int currentRobot;
extern TcpClientsArray tcpClientsArray;

//Sol Glace

#define NUM 10
#define DENSITY (5.0)
#define GPB 3

struct MyObject {
	dBodyID body;			// the body
	dGeomID geom[GPB];		// geometries representing this body
};

static MyObject obj[NUM];
dGeomID TriMesh1;
static dTriMeshDataID TriData1;

void initNetRegister(NetRegister *netRegister) { 
	netRegister->size=0;
}

void addNetRegister(NetRegister *netRegister, dBodyID id) {
	if (netRegister->size < MAX_NET_BODIES-1) {
		netRegister->buf[netRegister->size]=id;
		netRegister->size++;
	}
	else {
		fprintf(stderr, "netRegister overflow !\n");
	}
}

void dBodyAddNet(dBodyID id) {
	addNetRegister(&netRegister, id);
}

void netRecvPositions() {
	if(mcast) 
		dgramWait(&mcastClientChannel, &dgram);
	else
		tcpWait(&tcpClientChannel, &dgram);
		
	dgramSplit(&dgram);		
}

void dgramPushPosRot(DgramBuffer *b, int i) {
	dBodyID bid=(dBodyID)netRegister.buf[i];
	dgramPush(b, &i, sizeof(unsigned short));
	dgramPush(b, dBodyGetPosition(bid), sizeof(dReal)*3);
	dgramPush(b, dBodyGetQuaternion(bid), sizeof(dReal)*4);
}

void dgramPushGeneric(DgramBuffer *b, int i) {
	dgramPush(b, "G", 1);
	dgramPushPosRot(b, i);
}
void dgramPushBox(DgramBuffer *b, int i) {
	dgramPush(b, "B", 1);
	dgramPushPosRot(b, i);
}

void dgramPushCylinder(DgramBuffer *b, int i) {
	dgramPush(b, "C", 1);
	dgramPushPosRot(b, i);
}

void dgramPushSphere(DgramBuffer *b, int i) {
	dgramPush(b, "S", 1);
	dgramPushPosRot(b, i);
}

int dgramSplit(DgramBuffer *b) {
	int len=b->i, i;
	
	//fprintf(stderr, "dgram: len:%d\n", len);
			
	i=0;
	unsigned short body;
	dBodyID bid;
	
	while(i<len) {		
		memcpy(&body, &dgram.dgram[i], sizeof(unsigned short));
		
		dReal r[3];
		memcpy(r, &b->dgram[i+2], sizeof(dReal)*3);
		
		bid=(dBodyID)netRegister.buf[body];
		//fprintf(stderr, "body:%d %x\n", body, bid);
		dBodySetPosition(bid, r[0], r[1], r[2]);
								
		dQuaternion q;
		memcpy(&q, &b->dgram[i+2+sizeof(dReal)*3], sizeof(dReal)*4);
		dBodySetQuaternion(bid, q);

		i=i+2+sizeof(dReal)*3+sizeof(dReal)*4;
		
	}
	return 0;
}



void netSendPositions() {
		static int div=0;
		
		if(div < 2) { div++; return; }
		
		div=0;
		
	//fprintf(stderr, "mcast:%d needTcpCpt:%d\n", mcast, tcpClientsArray.needTcpCpt);
	if(!mcast && tcpClientsArray.needTcpCpt==0) { 
		//fprintf(stderr, "netSendPositions skip !\n"); 
		return;
	}
	
	//fprintf(stderr, "netSendPositions:%d\n", tcpClientsArray.needTcpCpt);
	
	DgramBuffer *b=&dgram;
	
	for(int i=0; i<netRegister.size; ++i) {
		dBodyID bid=(dBodyID)netRegister.buf[i];
		//fprintf(stderr, "push:%d\n", i);
		dgramPush(b, &i, sizeof(unsigned short));
		dgramPush(b, dBodyGetPosition(bid), sizeof(dReal)*3);
		dgramPush(b, dBodyGetQuaternion(bid), sizeof(dReal)*4);	
	}
	
	if(mcast) dgramSend(&mcastServerChannel, b);
	
	int cpt=0;
	
	int nbTcp=tcpClientsArray.needTcpCpt;
	for(int i=0; i <TCP_MAX_CLIENTS ; ++i) {
			if (tcpClientsArray.needTcp[i]>0) {
				//fprintf(stderr, "write to tcp socket :%d\n", dgram.i);
				if(send(tcpClientsArray.socks[i], dgram.dgram, dgram.i, 0) != dgram.i) {
					perror("write to tcp socket !");
					fclose(tcpClientsArray.flux[i]);
					tcpClientsArray.socks[i]=0;
					tcpClientsArray.needTcpCpt--;
				} 
				cpt++;
				if(cpt==nbTcp) break;
			}
	}
	
	if(cpt!=tcpClientsArray.needTcpCpt) {
		fprintf(stderr, "tcpClientsArray.needTcpCpt invalide size !\n");
	}
	
	dgramClear(&dgram);
	
	
}

static void command (int cmd);

void tcpCommand(int i, char *cmd) {
	//fprintf(stderr, "tcpCommand: %d %s\n", i, cmd);
	if(!strcmp(cmd, "tcp bye\n")) {
		tcpClientsArray.needTcp[i]=0;
		fclose(tcpClientsArray.flux[i]);
		close(tcpClientsArray.socks[i]);
		
		tcpClientsArray.needTcpCpt--;	
	}
	if(!strcmp(cmd, "tcp please!\n")) {
		tcpClientsArray.needTcp[i]=1;
		tcpClientsArray.needTcpCpt++;	
		return;
	}
	int num;
	char c;
	if(sscanf(cmd, "%d %c", &num, &c) == 2) {
		int lastCurrent=currentRobot;
		currentRobot=num;
		command(c);
		currentRobot=lastCurrent;
	}
}


void netRecvCommands(void) {
	int cpt=0;
	char line[MAX_CMD_CHAR];
	//fprintf(stderr, "netRecvCommands\n");
	
	for(int i=0; i<TCP_MAX_CLIENTS; ++i) {
			if (tcpClientsArray.socks[i]>0) {
				//fprintf(stderr, "Try fgets...");
				while(fgets(line, MAX_CMD_CHAR, tcpClientsArray.flux[i]) != NULL) {
					tcpCommand(i, line);
				}
				//fprintf(stderr, "Try fgets done");
				cpt++;
				if(cpt==tcpClientsArray.size) break;
			}
	}

}

void tcpServerLoop() {
	struct sockaddr_storage src_addr;
	socklen_t len_src_addr;
	
	// new connection
	
	len_src_addr=sizeof src_addr;
	int sock;
	if(tcpClientsArray.size < TCP_MAX_CLIENTS-1) {
		//fprintf(stderr, "test accept (%d)...", tcpServerChannel.tcpServerSocket);
		sock=accept4(tcpServerChannel.tcpServerSocket, (struct sockaddr *)&src_addr, &len_src_addr, SOCK_NONBLOCK);
		if(sock!=-1) {
			
			int i;
			for(i=0; i < TCP_MAX_CLIENTS; ++i) {
				if (tcpClientsArray.socks[i]<=0)
					break;
			}
			if(i<TCP_MAX_CLIENTS) {
				tcpClientsArray.socks[i]=sock;
				if((tcpClientsArray.flux[i]=fdopen(sock, "rw"))==NULL) {
					perror("fdopen");
				}
				tcpClientsArray.size++;
			}				
		}
    }
 
    netRecvCommands();
}

void tcpClientLoop() {
	static unsigned long lastSec;
	unsigned long sec=time(NULL);
	if(sec!=lastSec) {
		//fprintf(stderr, "send hello\n");
		netSendCommand("Hello");
		lastSec=sec;
	}
}	

void myRand() {
	int i;
	for (i=0;i <20; ++i) {
		fprintf(stderr,"%d\n", rand());
	}
}

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback(void*, dGeomID o1, dGeomID o2)
{
    int i, n;
    
	//Sol Glace

	if ((o1 == obj[0].geom[0] && o2 != obj[0].geom[0] && o2 != ground_box && o2 != ground) || (o2 == obj[0].geom[0] && o1 != obj[0].geom[0] && o1 != ground && o1 != ground_box)) {
		const int N = 10;
		dContact contact[N];
		n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
		if (n > 0) {
			for (i = 0; i < n; i++) {
				contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
					dContactSoftERP | dContactSoftCFM | dContactApprox1;
				contact[i].surface.mu = 0.2;
				contact[i].surface.slip1 = 10;
				contact[i].surface.slip2 = 0.1;
				contact[i].surface.soft_erp = 0.5;
				contact[i].surface.soft_cfm = 0.3;
				dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
				dJointAttach(c,
					dGeomGetBody(contact[i].geom.g1),
					dGeomGetBody(contact[i].geom.g2));
			}
		}
		return;
	}

    // only collide things with the ground
    int g1 = (o1 == ground || o1 == ground_box);
    int g2 = (o2 == ground || o2 == ground_box);

     if ((o1 == ground_box && o2 != ground ) || (o2 == ground_box && o1 != ground)) {
        const int N = 10;
        dContact contact[N];
        //fprintf(stderr, "contact ground box\n");
        n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
        if (n > 0) {
            for (i = 0; i < n; i++) {
                contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                    dContactSoftERP | dContactSoftCFM | dContactApprox1;
                contact[i].surface.mu = 0.2; //dInfinity;
                contact[i].surface.slip1 = 2;
                contact[i].surface.slip2 = 2;
                contact[i].surface.soft_erp = 0.5;
                contact[i].surface.soft_cfm = 0.3;
                dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
                dJointAttach(c,
                    dGeomGetBody(contact[i].geom.g1),
                    dGeomGetBody(contact[i].geom.g2));
            }
        }
        return;
    }
    
    if (o1 == sphgeom || o2 == sphgeom) {
        const int N = 10;
        dContact contact[N];
        n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
        if (n > 0) {
            for (i = 0; i < n; i++) {
                contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                    dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
                contact[i].surface.mu = dInfinity;
                contact[i].surface.slip1 = 0.1;
                contact[i].surface.slip2 = 0.1;
                contact[i].surface.soft_erp = 0.5;
                contact[i].surface.soft_cfm = 0.3;
                contact[i].surface.bounce = 1.65;
                contact[i].surface.bounce_vel = 1;
                dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
                dJointAttach(c,
                    dGeomGetBody(contact[i].geom.g1),
                    dGeomGetBody(contact[i].geom.g2));
            }
        }
        return;
    }

    const int N = 10;
    dContact contact[N];
    n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
    if (n > 0) {
		if(dGeomGetData(o1)==(void *) G_BOX || dGeomGetData(o2)==(void *) G_BOX) {
			if(dGeomGetData(o1) != (void *) G_GROUND && dGeomGetData(o2) != (void *) G_GROUND)  {
				//fprintf(stderr, "Box contact !\n");
				dGeomID g=o2;
				if(dGeomGetData(o1)==(void *) G_BOX) g=o1;
				//	fprintf(stderr, "Disable body 1 !\n");
				dBodyID b=dGeomGetBody(g);
				dBodyDisable (b);
								
				return;
			}
		}
        for (i = 0; i < n; i++) {
            contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                dContactSoftERP | dContactSoftCFM | dContactApprox1;
            contact[i].surface.mu = 0.9; //dInfinity;
            contact[i].surface.slip1 = 0.1;
            contact[i].surface.slip2 = 0.1;
            contact[i].surface.soft_erp = 0.5;
            contact[i].surface.soft_cfm = 0.3;
            dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
            dJointAttach(c,
                dGeomGetBody(contact[i].geom.g1),
                dGeomGetBody(contact[i].geom.g2));
        }
        return;
    }
    
   

}

// start simulation - set viewpoint

static float xyz[3] = { 0,-10,0 };
static float hpr[3] = { 0.8317f,-0.9817f,0.8000f };
//static float xyz[3] = { 0,-90,0 };
//static float hpr[3] = { 0.8317f,-90.9817f,0.8000f };
static void start()
{
	//buggy[1].moveBuggy.speedBuggy = 50;
    buggy[0].moveBuggy.lock_cam = true;
    //dsSetDrawMode(1);
    initTerrain();

    dAllocateODEDataForThread(dAllocateMaskAll);

    dsSetViewpoint(xyz, hpr);
    printf("Press:\t'a' to increase speed.\n"
        "\t's' to decrease speed.\n"
        "\t'q' to steer left.\n"
        "\t'd' to steer right.\n"
        "\t'l' to lock the camera or to unlock it.\n"
        "\t' ' to reset speed and steering.\n"
        "\tUse caps to control the second buggy.\n"
        "\t'5' to turn the buggy over.\n"
        "\t'6' to turn the second buggy over.\n"
        "\t'1' to save the current state to 'state.dif'.\n"
    );
}

static void command(int cmd)
{
	float accel=1;
	char extCmd[MAX_CMD_CHAR];
	
	//fprintf(stderr, "cmd for current: %d: %c %d\n", currentRobot, cmd, cmd);
	
	if(cmd>='0' && cmd <= '9') {
		int n=cmd-'0';
		currentRobot=n;
		camPos(buggy[currentRobot], xyz, hpr);
		buggy[currentRobot].moveBuggy.lock_cam = true;
		return;
	}

	sprintf(extCmd,"%d %c\n", currentRobot, cmd);
    switch (cmd) {
        case 'z':
        case 'Z' :
			//fprintf(stderr, "z : %d\n", currentRobot);
			if(server) {
	
				if (!(buggy[currentRobot].moveBuggy.speedBuggy >= 10)) {
					buggy[currentRobot].moveBuggy.speedBuggy += accel;
				}
			}
			else netSendCommand(extCmd);
            break;
       
        case 's':
        case 'S':
            if(server) {
				if (!(buggy[currentRobot].moveBuggy.speedBuggy <= -10)) {
					buggy[currentRobot].moveBuggy.speedBuggy -= accel;
				}
			
			}
			else netSendCommand(extCmd);
            break;
      
        case 'q':
			if(server) buggy[currentRobot].moveBuggy.steerBuggy -= 0.3;
            else netSendCommand(extCmd);
            break;

        case 'd':
            if(server) buggy[currentRobot].moveBuggy.steerBuggy += 0.3;
            else netSendCommand(extCmd);
            break;

        case 'h':
			buggy[currentRobot].moveBuggy.lock_cam = 0;
			{
				float xyz[3] = { 0, 0, 100 };
				float hpr[3] = { 0, -90, 0 };
				dsSetViewpoint(xyz, hpr);
			}
            break;
			
        case 't':
			if(server) tirer(&buggy[currentRobot], SPHERERADIUS, space, world);
            else netSendCommand(extCmd);
            break;
        
        case 'l':
            buggy[currentRobot].moveBuggy.lock_cam = !buggy[currentRobot].moveBuggy.lock_cam;
            break;
            
        case 'r':
            if(server) retournerBuggy(buggy[currentRobot]);
            else netSendCommand(extCmd);
            break;        
       
        case ':':
			
            if(server) arreterBuggy(buggy[currentRobot]);
            else netSendCommand(extCmd);
            break;
            
        case '1': {
            FILE* f = fopen("state.dif", "wt");
            if (f) {
                dWorldExportDIF(world, f, "");
                fclose(f);
            }
        }
    }
}

// simulation loop
static void simLoop(int pause)
{
	
	static int lastCptBox=0;
	int cptBox=0;
	
    int i;
    
    if(server) tcpServerLoop(); else tcpClientLoop();
    
    if (server) {
		if (!pause) {
			for(i=0; i<NB_BUGGY; ++i) {
				deplacementBuggy(&buggy[i]);
			}
			dSpaceCollide(space, 0, &nearCallback);
			dWorldStep(world, 0.05);

			// remove all contact joints
			dJointGroupEmpty(contactgroup);
			
			netSendPositions();
		}
	}
    else { // client		
		netRecvPositions();
	}
	
	//Draw Sol Glace

	dsSetColor(0, 0, 0);
	dTriIndex* TrianglesGlace = (dTriIndex*)::TrianglesGlace;



	{const dReal* Pos = dGeomGetPosition(TriMesh1);
	const dReal* Rot = dGeomGetRotation(TriMesh1);

	{for (int i = 0; i < Nomdre_de_triangles / 3; i++) {
		const dReal v[9] = { // explicit conversion from float to dReal
		  ArêtesGlace[TrianglesGlace[i * 3 + 0] * 3 + 0],
		  ArêtesGlace[TrianglesGlace[i * 3 + 0] * 3 + 1],
		  ArêtesGlace[TrianglesGlace[i * 3 + 0] * 3 + 2],
		  ArêtesGlace[TrianglesGlace[i * 3 + 1] * 3 + 0],
		  ArêtesGlace[TrianglesGlace[i * 3 + 1] * 3 + 1],
		  ArêtesGlace[TrianglesGlace[i * 3 + 1] * 3 + 2],
		  ArêtesGlace[TrianglesGlace[i * 3 + 2] * 3 + 0],
		  ArêtesGlace[TrianglesGlace[i * 3 + 2] * 3 + 1],
		  ArêtesGlace[TrianglesGlace[i * 3 + 2] * 3 + 2]
		};

		dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
	}}}

	// fixed or not-fixed camera
	
		if(client) {
			dReal sides[3] = { LENGTH,WIDTH,HEIGHT };

			//Draw buggy 1 & 2
			dsSetTexture(DS_WOOD);
			
			
		
		dsSetColor(1.0, 0.0, 0.1);
		for(int i=0; i< NB_BOX; ++i) {
			dReal sides[3] = { BOX_LENGTH, BOX_WIDTH, BOX_HEIGHT };
			const dReal *pos=dBodyGetPosition(boxBodies[i]);
			if(dBodyIsEnabled (boxBodies[i])) {
				dsDrawBox(dBodyGetPosition(boxBodies[i]), dBodyGetRotation(boxBodies[i]), sides);
			}
			else {
				dBodySetPosition(boxBodies[i], pos[0], pos[1], -2);
			}
			pos=dBodyGetPosition(boxBodies[i]);
			if(pos[2] > 0.0) ++cptBox;
			
		}
		
		if(cptBox!=lastCptBox) {
			fprintf(stderr,"cptBox:%d\n", cptBox);
			lastCptBox=cptBox;
		}
		
		if (buggy[currentRobot].moveBuggy.lock_cam) {
			camPos(buggy[currentRobot], xyz, hpr);
		}
		
		for(int i=0; i< NB_BUGGY; ++i) {
			drawBuggy(buggy[i], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
		}
		/*
			dsSetTexture(DS_WOOD);
			drawBuggy(buggy[1], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
			
			//Draw buggy 3, 4, 5 & 6
			dsSetTexture(DS_WOOD);
			drawBuggy(buggy[2], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);

			dsSetTexture(DS_WOOD);
			drawBuggy(buggy[3], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
			
			dsSetTexture(DS_WOOD);
			drawBuggy(buggy[4], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
			
			dsSetTexture(DS_WOOD);
			drawBuggy(buggy[5], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
			*/
			
			dsSetColor(1, 1, 1);
			
			dVector3 ss;
			dsSetColor(0.5, 0.5, 1);
			//dsSetColorAlpha(240/255.0, 130/255.0, 30/255.0, 1);
			
			dGeomBoxGetLengths(ground_box, ss);
			dsDrawBox(dGeomGetPosition(ground_box), dGeomGetRotation(ground_box), ss);
			

			dsSetTexture(DS_WOOD);
			dsSetTexture(DS_WOOD);
			dsSetColorAlpha(0, 0, 0, 1);
			for (i = 0; i < 4; i++) {
					dGeomBoxGetLengths(obstacle[i], ss);
					dsDrawBox(dGeomGetPosition(obstacle[i]), dGeomGetRotation(obstacle[i]), ss);
			}

			dsSetColorAlpha(0, 1, 0, 1);
			const dReal* SPos = dBodyGetPosition(sphbody);
			const dReal* SRot = dBodyGetRotation(sphbody);
			const dReal spos[3] = { SPos[0], SPos[1], SPos[2] };
			const dReal srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };
			dsDrawSphere
			(
					spos,
					srot,
					SPHERERADIUS
			); // single precision

			drawHFieldGeom(getHField(), 0, 0, 0);
			
		}
}


int main(int argc, char** argv){
    int i;
    dMass m;
    
    netInit(argc, argv);

    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

    // create world
    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -0.98);
    
		ground = dCreatePlane(space, 0, 0, 1, 0);
		dGeomSetData(ground, (void *)G_GROUND);
	
		
	double colors[6][3]={{0,0,1}, {0,1,0}, {0,1,1}, {1,0,0}, {1,0,1}, {1,1,0}};
	for (int i=0; i<NB_BUGGY; ++i) {
		int radius=WORLD_DIAMETER/2;
		double angleZ=i*2*M_PI/NB_BUGGY;
		double x=cos(angleZ)*radius, y=sin(angleZ)*radius;
		fprintf(stderr,"angle[%d]=%f\n", i, angleZ);
		
		buggy[i].colors[0]=colors[i%6][0]; buggy[i].colors[1] = colors[i%6][1]; buggy[i].colors[2] = colors[i%6][2];
		
		createABuggy(&buggy[i], m, BMASS, CMASS, WMASS, LENGTH, WIDTH, HEIGHT, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, RADIUS, x, y, STARTZ, angleZ, yunit, zunit, space, world);
    }
    
    float l=BOX_LENGTH; float w=BOX_WIDTH; float h=BOX_HEIGHT;
    for (int i=0; i<NB_BOX; ++i) {
		double x,y,z;
		dBodyID b;
		dGeomID g;
		b=dBodyCreate(world);
		dBodyAddNet(b);
				
		double angle=(randFixed()%360)*2*M_PI/360;
		double dist=randFixed()%((int)(WORLD_DIAMETER/3));
		x=cos(angle)*dist;
		y=sin(angle)*dist;
		
		//x=randFixed()%20;
		//y=randFixed()%20;
		z=3.0;
		
		dBodySetPosition(b, x, y, z);
		dMassSetBox(&m, 1, l, w, h);
		dMassAdjust(&m, CMASS);
		dBodySetMass(b, &m);
		g = dCreateBox(0, l, w, h);
		dGeomSetBody(g, b);	
		dGeomSetData(g, (void *)G_BOX);
		boxBodies[i]=b;
		boxGeoms[i]=g;
		dSpaceAdd(space, boxGeoms[i]);
	}
	
	 // sphere
    sphbody = dBodyCreate(world);		
    dBodyAddNet(sphbody);

    dMassSetSphere(&m, 0.2, SPHERERADIUS);
    dBodySetMass(sphbody, &m);
    sphgeom = dCreateSphere(0, SPHERERADIUS);
    dGeomSetBody(sphgeom, sphbody);
    dBodySetPosition(sphbody, 15, 0, 1);
    dSpaceAdd(space, sphgeom);
    
    // obstacle
    for (i = 0; i < 4; i++) {
        if (i % 2 == 0) {
            obstacle[i] = dCreateBox(space, 2, 1, 10);
        }
        else {
            obstacle[i] = dCreateBox(space, 2, 10, 1);
            
        }
    }

    dGeomSetPosition(obstacle[1], 5, 0, 1-100);
    dGeomSetPosition(obstacle[3], -5, 0, 1-100);
    dGeomSetPosition(obstacle[0], 0, 5, 1-100);
    dGeomSetPosition(obstacle[2], 0, -5, 1-100);

    dMatrix3 R1;
    dRFromAxisAndAngle(R1, 0, 1, 0, 1.55);

    for (i = 0; i < 4; i++) {
        dGeomSetRotation(obstacle[i], R1);
    }

    
    // ====
        
    // environment
    ground_box = dCreateBox(space, WORLD_DIAMETER/4*3, WORLD_DIAMETER/4*3, 1);            
	//dMatrix3 R;
    //dRFromAxisAndAngle(R, 0, 1, 0, -0.15);
    dGeomSetPosition(ground_box, 0, 0, -0.47);
    //dGeomSetRotation(ground_box, R);

    createHField(space);

	//Sol glace
	const unsigned preprocessFlags = (1U << dTRIDATAPREPROCESS_BUILD_CONCAVE_EDGES) | (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES);
	TriData1 = dGeomTriMeshDataCreate();
	dGeomTriMeshDataBuildSingle(TriData1, &ArêtesGlace[0], 3 * sizeof(float), Nombre_de_sommets, (dTriIndex*)&TrianglesGlace[0], Nomdre_de_triangles, 3 * sizeof(dTriIndex));
	dGeomTriMeshDataPreprocess2(TriData1, preprocessFlags, NULL);
	TriMesh1 = dCreateTriMesh(space, TriData1, 0, 0, 0);
	dGeomSetData(TriMesh1, TriData1);

	dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();
	dGeomTriMeshDataBuildSingle(new_tmdata, &ArêtesGlace[0], 3 * sizeof(float), Nombre_de_sommets,
		(dTriIndex*)&TrianglesGlace[0], Nomdre_de_triangles, 3 * sizeof(dTriIndex));
	dGeomTriMeshDataPreprocess2(new_tmdata, (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL);


	obj[0].geom[0] = dCreateTriMesh(space, new_tmdata, 0, 0, 0);

	// remember the mesh's dTriMeshDataID on its userdata for convenience.
	dGeomSetData(obj[0].geom[0], new_tmdata);

	dMassSetTrimesh(&m, DENSITY, obj[0].geom[0]);
	printf("mass at %f %f %f\n", m.c[0], m.c[1], m.c[2]);
	dGeomSetPosition(obj[0].geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
        
    // run simulation
    dsSimulationLoop(argc, argv, 1000, 800, &fn);

    destroyBuggy(buggy[0]);
    destroyBuggy(buggy[1]);
    destroyBuggy(buggy[2]);
    destroyBuggy(buggy[3]);
    destroyBuggy(buggy[4]);
    destroyBuggy(buggy[5]);

    dGeomDestroy(ground_box);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
	fprintf(stderr, "bye\n");    
    return 0;
}
