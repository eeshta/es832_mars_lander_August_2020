// Mars lander simulator
// Version 1.11
//  variables 1 header
// Gabor Csanyi and Andrew Gee, October 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#ifndef __VARIABLES_1_INCLUDED__
#define __VARIABLES_1_INCLUDED__


#ifdef _WIN32
#define _USE_MATH_DEFINES
#include <windows.h>
#include <irrKlang.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#include <irrKlang.h>
#else
#include <GL/glut.h>
#endif
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cstdlib>

#include "program_constants.h"
#include "vector3d.h"
#include "model.h"
#include "extra_data.h"

using namespace std;


//extern bool new_autopilot = true;
extern bool stabilized_attitude, autopilot_enabled, infinite_fuel;
extern int autopilot_state;
extern double throttle, fuel, desired_altitude;
extern unsigned short scenario;
extern string scenario_description[];
extern vector3d position, orientation, velocity, closeup_coords_right;
extern parachute_status_t parachute_status;
extern autopilot_status_t autopilot_status;
extern int stabilized_attitude_angle, out_of_plane_angle;

// Function prototypes
void invert(double m[], double mout[]);
void xyz_euler_to_matrix(vector3d ang, double m[]);
vector3d matrix_to_xyz_euler(double m[]);
void normalize_quat(quat_t& q);
quat_t axis_to_quat(vector3d a, const double phi);
double project_to_sphere(const double r, const double x, const double y);
quat_t add_quats(quat_t q1, quat_t q2);
void quat_to_matrix(double m[], const quat_t q);
quat_t track_quats(const double p1x, const double p1y, const double p2x, const double p2y);
void microsecond_time(unsigned long long& t);
void fghCircleTable(double** sint, double** cost, const int n);
void glutOpenHemisphere(GLdouble radius, GLint slices, GLint stacks);
void glutMottledSphere(GLdouble radius, GLint slices, GLint stacks);
void glutCone(GLdouble base, GLdouble height, GLint slices, GLint stacks, bool closed);
void enable_lights(void);
void setup_lights(void);
void glut_print(float x, float y, string s);
double atmospheric_density(vector3d pos);
void draw_dial(double cx, double cy, double val, string title, string units);
void draw_control_bar(double tlx, double tly, double val, double red, double green, double blue, string title);
void draw_indicator_lamp(double tcx, double tcy, string off_text, string on_text, bool on);
void draw_instrument_window(void);
void display_help_arrows(void);
void display_help_prompt(void);
void display_help_text(void);
void draw_orbital_window(void);
void draw_parachute_quad(double d);
void draw_parachute(double d);
bool generate_terrain_texture(void);
void update_closeup_coords(void);
void draw_closeup_window(void);
void draw_main_window(void);
void refresh_all_subwindows(void);
bool safe_to_deploy_parachute(void);
void update_visualization(void);
void attitude_stabilization(void);
vector3d thrust_wrt_world(void);
void autopilot(void);
void numerical_dynamics(void);
void initialize_simulation(void);
void update_lander_state(void);
void reset_simulation(void);
void set_orbital_projection_matrix(void);
void reshape_main_window(int width, int height);
void orbital_mouse_button(int button, int state, int x, int y);
void orbital_mouse_motion(int x, int y);
void closeup_mouse_button(int button, int state, int x, int y);
void closeup_mouse_motion(int x, int y);
void glut_special(int key, int x, int y);
void glut_key(unsigned char k, int x, int y);


#endif

