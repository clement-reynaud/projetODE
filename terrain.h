#ifndef TERRAINH
#define TERRAINH
#include <ode/ode.h>
#include <random>
#include <drawstuff/drawstuff.h>
#include "projetODE.h"

// Heightfield dimensions
#define HFIELD_WSTEP			20		// Vertex count along edge >= 2
#define HFIELD_DSTEP			20

#define HFIELD_WIDTH			REAL(WORLD_DIAMETER*2 )
#define HFIELD_DEPTH			REAL(WORLD_DIAMETER*2 )

#define HFIELD_WSAMP			( HFIELD_WIDTH / ( HFIELD_WSTEP-1 ) )
#define HFIELD_DSAMP			( HFIELD_DEPTH / ( HFIELD_DSTEP-1 ) )

#define	DEGTORAD			0.01745329251994329577f				//!< PI / 180.0, convert degrees to radians

#define dsDrawTriangle dsDrawTriangleD

void initTerrain(void);
dReal heightfield_callback(void*, int x, int z); 
void drawHFieldGeom(dGeomID g, const dReal* pos, const dReal* R, int show_aabb);
void createHField(dSpaceID space);
dGeomID getHField(void);

#endif
