// Mars lander simulator
// Version 1.11
// extra data types
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#ifndef __EXTRA_DATA_INCLUDED__
#define __EXTRA_DATA_INCLUDED__

#include "vector3d.h"
//#define N_TRACK 1000

using namespace std;

// Data type for recording lander's previous positions
struct track_t {
	unsigned short n;
	unsigned short p;
	vector3d pos[N_TRACK];
};

// Quaternions for orbital view transformation
struct quat_t {
	vector3d v;
	double s;
};

// Data structure for the state of the close-up view's coordinate system
struct closeup_coords_t {
	bool initialized;
	bool backwards;
	vector3d right;
};

// Enumerated data types
enum parachute_status_t { NOT_DEPLOYED = 0, DEPLOYED = 1, LOST = 2 };
enum autopilot_status_t { LAND = 0, INJECTION = 1, ORBITAL_TRANSFER = 2 };

#endif