#pragma once
#include <math.h>
#include <ode/common.h>

const int Nombre_de_sommets = 18;
const int Nomdre_de_triangles = 48 * 3;
const float HauteurMesh = 1.05f;
const float R = 2.0f;
float z = cos(54.0);
float tabx[9] = {
	0,
	cos(45) * R,
	cos(45) * R,
	-cos(45) * R,
	-cos(45) * R
};
float taby[9] = {
	0,
	sin(45) * R,
	-sin(45) * R,
	-sin(45) * R,
	sin(45) * R
};

float tabModif[8] = {
	1.6,
	0.8,
	5.6,
	2,
	0.6,
	3.1,
	1.1,
	0.2
};

float AretesGlace[Nombre_de_sommets * 3] = {
	0.0f, 0.0f, HauteurMesh,
	0.0f * tabModif[0], R * tabModif[0], HauteurMesh,
	tabx[1] * tabModif[1], taby[1] * tabModif[1], HauteurMesh,
	R * tabModif[2], 0.0f * tabModif[2], HauteurMesh,
	tabx[2] * tabModif[3], taby[2] * tabModif[3], HauteurMesh,
	0.0f * tabModif[4], -R * tabModif[4], HauteurMesh,
	tabx[3] * tabModif[5], taby[3] * tabModif[5], HauteurMesh,
	-R * tabModif[6], 0.0f * tabModif[6], HauteurMesh,
	tabx[4] * tabModif[7], taby[4] * tabModif[7], HauteurMesh,

	0.0f, 0.0f, -1.0f,
	0.0f * tabModif[0], R * tabModif[0], -1.0f,
	tabx[1] * tabModif[1], taby[1] * tabModif[1], -1.0f,
	R * tabModif[2], 0.0f * tabModif[2], -1.0f,
	tabx[2] * tabModif[3], taby[2] * tabModif[3], -1.0f,
	0.0f * tabModif[4], -R * tabModif[4], -1.0f,
	tabx[3] * tabModif[5], taby[3] * tabModif[5], -1.0f,
	-R * tabModif[6], 0.0f * tabModif[6], -1.0f,
	tabx[4] * tabModif[7], taby[4] * tabModif[7], -1.0f,
};

dTriIndex TrianglesGlace[Nomdre_de_triangles / 3][3] = {
	//{0,1,2},
	{2,1,0},

	//{0,2,3},
	{3,2,0},

	//{0,3,4},
	{4,3,0},

	//{0,4,5},
	{5,4,0},

	//{0,5,6},
	{6,5,0},

	//{0,6,7},
	{7,6,0},

	//{0,7,8},
	{8,7,0},

	//{0,8,1},
	{1,8,0},


	//{9,10,11},
	{11,10,9},

	//{9,11,12},
	{12,11,9},

	//{9,12,13},
	{13,12,9},

	//{9,13,14},
	{14,13,9},

	//{9,14,15},
	{15,14,9},

	//{9,15,16},
	{16,15,9},

	//{9,16,17},
	{17,16,9},

	//{9,17,10},
	{10,17,9},


	{1,10,2},
	{2,10,1},

	{2,10,11},
	{11,10,2},

	{2,11,3},
	{3,11,2},

	{3,11,12},
	{12,11,3},

	{3,12,4},
	{4,12,3},

	{4,12,13},
	{13,12,4},

	{4,13,5},
	{5,13,4},

	{5,13,14},
	{14,13,5},

	{5,14,6},
	{6,14,5},

	{6,14,15},
	{15,14,6},

	{6,15,7},
	{7,15,6},

	{7,15,16},
	{16,15,7},

	{7,16,8},
	{8,16,7},

	{8,16,17},
	{17,16,8},

	{8,17,1},
	{1,17,8},

	{1,10,17},
	{17,10,1}
};