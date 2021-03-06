// Mars lander simulator
// Version 1.11
// Header file
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#ifdef _WIN32
#define _USE_MATH_DEFINES
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <time.h>

// GLUT mouse wheel operations work under Linux only
#if !defined (GLUT_WHEEL_UP)
#define GLUT_WHEEL_UP 3
#define GLUT_WHEEL_DOWN 4
#endif

// Graphics constants
#define GAP 5
#define SMALL_NUM 0.0000001
#define N_RAND 20000
#define PREFERRED_WIDTH 1024
#define PREFERRED_HEIGHT 768
#define MIN_INSTRUMENT_WIDTH 1024
#define INSTRUMENT_HEIGHT 300
#define GROUND_LINE_SPACING 20.0
#define CLOSEUP_VIEW_ANGLE 30.0
#define TRANSITION_ALTITUDE 10000.0
#define TRANSITION_ALTITUDE_NO_TEXTURE 4000.0
#define TERRAIN_TEXTURE_SIZE 1024
#define INNER_DIAL_RADIUS 65.0
#define OUTER_DIAL_RADIUS 75.0
#define MAX_DELAY 160000
#define N_TRACK 1000
#define TRACK_DISTANCE_DELTA 100000.0
#define TRACK_ANGLE_DELTA 0.999
#define HEAT_FLUX_GLOW_THRESHOLD 1000000.0

// Mars constants
#define MARS_RADIUS 3386000.0 // (m)
#define MARS_MASS 6.42E23 // (kg)
#define GRAVITY 6.673E-11 // (m^3/kg/s^2)
#define EXOSPHERE 200000.0 // (m)

// Day constants from https://en.wikipedia.org/wiki/Rotation_period
#define SUN_DAY 2192831.568 // (s)
#define MERCURY_DAY 5067031.68 // (s)
#define VENUS_DAY -20996815.68 // (s) in opposite direction
#define EARTH_DAY 86164.1 // (s)
#define MOON_DAY 2360591.5104 // (s)
#define MARS_DAY 88642.65 // (s)
#define JUPITER_DAY 35430.048 // (s)
#define SATURN_DAY 36806.4 // (s)
#define URANUS_DAY -62063.712 // (s) in opposite direction
#define NEPTUNE_DAY 57996 // (s)
#define PLUTO_DAY -551852.352 // (s) in opposite direction
#define DEATH_STAR_DAY 86164.1 // (s)

// Lander constants
#define LANDER_SIZE 1.0 // (m)
#define AREA_LANDER M_PI * LANDER_SIZE * LANDER_SIZE // (m^2)
#define AREA_CHUTE 5 * 2.0 * LANDER_SIZE * 2.0 * LANDER_SIZE // (m^2)
#define UNLOADED_LANDER_MASS 100.0 // (kg)
#define FUEL_CAPACITY 100.0 // (l)
#define FUEL_RATE_AT_MAX_THRUST 0 // 0.5 // (l/s)
#define FUEL_DENSITY 1.0 // (kg/l)
// MAX_THRUST, as defined below, is 1.5 * weight of fully loaded lander at surface
#define MAX_THRUST (1.5 * (FUEL_DENSITY*FUEL_CAPACITY+UNLOADED_LANDER_MASS) * (GRAVITY*MARS_MASS/(MARS_RADIUS*MARS_RADIUS))) // (N)
#define ENGINE_LAG 0.0 // (s)
#define ENGINE_DELAY 0.0 // (s)
#define DRAG_COEF_CHUTE 2.0
#define DRAG_COEF_LANDER 1.0
#define MAX_PARACHUTE_DRAG 20000.0 // (N)
#define MAX_PARACHUTE_SPEED 500.0 // (m/s)
#define THROTTLE_GRANULARITY 20 // for manual control
#define MAX_IMPACT_GROUND_SPEED 1.0 // (m/s)
#define MAX_IMPACT_DESCENT_RATE 1.0 // (m/s)
#define MAX_DRAG_HEIGHT 60737 // (m)

using namespace std;

class vector3d {
  // Utility class for three-dimensional vector operations
public:
  vector3d() {x=0.0; y=0.0; z=0.0;}
  vector3d (double a, double b, double c=0.0) {x=a; y=b; z=c;}
  bool operator== (const vector3d &v) const { if ((x==v.x)&&(y==v.y)&&(z==v.z)) return true; else return false; }
  bool operator!= (const vector3d &v) const { if ((x!=v.x)||(y!=v.y)||(z!=v.z)) return true; else return false; }
  vector3d operator+ (const vector3d &v) const { return vector3d(x+v.x, y+v.y, z+v.z); }
  vector3d operator- (const vector3d &v) const { return vector3d(x-v.x, y-v.y, z-v.z); }
  friend vector3d operator- (const vector3d &v) { return vector3d(-v.x, -v.y, -v.z); }
  vector3d& operator+= (const vector3d &v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
  vector3d& operator-= (const vector3d &v) { x-=v.x; y-=v.y; z-=v.z; return *this; }
  vector3d operator^ (const vector3d &v) const { return vector3d(y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x); }
  double operator* (const vector3d &v) const { return (x*v.x + y*v.y +z*v.z); }
  friend vector3d operator* (const vector3d &v, const double &a) { return vector3d(v.x*a, v.y*a, v.z*a); }
  friend vector3d operator* (const double &a, const vector3d &v) { return vector3d(v.x*a, v.y*a, v.z*a); }
  vector3d& operator*= (const double &a) { x*=a; y*=a; z*=a; return *this; }
  vector3d operator/ (const double &a) const { return vector3d(x/a, y/a, z/a); }
  vector3d& operator/= (const double &a) { x/=a; y/=a; z/=a; return *this; }
  double abs2() const { return (x*x + y*y + z*z); }
  double abs() const { return sqrt(this->abs2()); }
  vector3d norm() const { double s(this->abs()); if (s==0) return *this; else return vector3d(x/s, y/s, z/s); }
  friend ostream& operator << (ostream &out, const vector3d &v) { out << v.x << ' ' << v.y << ' ' << v.z; return out; }
  double x, y, z;
private:
};

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

typedef struct {
	double r;       // a fraction between 0 and 1
	double g;       // a fraction between 0 and 1
	double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
	double h;       // angle in degrees
	double s;       // a fraction between 0 and 1
	double v;       // a fraction between 0 and 1
} hsv;

// Enumerated data type for parachute status
enum parachute_status_t { NOT_DEPLOYED = 0, DEPLOYED = 1, LOST = 2 };

// Enumerated data type for autopilot status
enum autopilot_status_t { IDLE = 0, MAX_SPEED = 1, FREE_FALL = 2, FULL_THRUST = 3, LAND = 4};

#ifdef DECLARE_GLOBAL_VARIABLES // actual declarations of all global variables for lander_graphics.cpp

// GL windows and objects
int main_window, closeup_window, orbital_window, instrument_window, view_width, view_height, win_width, win_height;
GLUquadricObj *quadObj;
GLuint terrain_texture;
GLuint parachute_texture;

// Planet textures from http://planetpixelemporium.com/planets.html
// Death Star from http://images.planet-mofo.com/A-Delusion/mofo%20art%20from%20ATD/
int planet;
GLuint sun_texture_HD;
GLuint mercury_texture_HD;
GLuint venus_texture_HD;
GLuint earth_day_texture_HD;
GLuint earth_night_texture_HD;
GLuint moon_texture_HD;
GLuint mars_texture_HD;
GLuint jupiter_texture_HD;
GLuint saturn_texture_HD;
GLuint uranus_texture_HD;
GLuint neptune_texture_HD;
GLuint pluto_texture_HD;
GLuint death_star_texture_HD;

short throttle_control;
track_t track;
bool texture_available;
bool show_planet_lines;
hsv lander_colour_hsv;
rgb lander_colour_rgb;
clock_t real_time;
bool music_playing;

// Simulation parameters
bool help = false;
bool paused = false;
bool landed = false;
bool crashed = false;
int last_click_x = -1;
int last_click_y = -1;
short simulation_speed = 5;
double delta_t, simulation_time;
unsigned short scenario = 0;
string scenario_description[10];
bool static_lighting = false;
closeup_coords_t closeup_coords;
float randtab[N_RAND];
bool do_texture = true;
unsigned long throttle_buffer_length, throttle_buffer_pointer;
double* throttle_buffer = NULL;
unsigned long long time_program_started;

// Lander state - the visualization routines use velocity_from_positions, so not sensitive to 
// any errors in the velocity update in numerical_dynamics
vector3d position, previous_position, orientation, velocity, velocity_from_positions, last_position;
double climb_speed, ground_speed, altitude, throttle, fuel, k, mass;
bool stabilized_attitude, autopilot_enabled, parachute_lost;
parachute_status_t parachute_status;
autopilot_status_t autopilot_status;
int stabilized_attitude_angle;
ofstream fout;

// Orbital and closeup view parameters
double orbital_zoom, save_orbital_zoom, closeup_offset, closeup_xr, closeup_yr, terrain_angle;
quat_t orbital_quat;

// For GL lights
GLfloat plus_y[] = { 0.0, 1.0, 0.0, 0.0 };
GLfloat minus_y[] = { 0.0, -1.0, 0.0, 0.0 };
GLfloat plus_z[] = { 0.0, 0.0, 1.0, 0.0 };
GLfloat top_right[] = { 1.0, 1.0, 1.0, 0.0 };
GLfloat straight_on[] = { 0.0, 0.0, 1.0, 0.0 };

#else // extern declarations of those global variables used in lander.cpp

extern bool stabilized_attitude, autopilot_enabled;
extern double delta_t, simulation_time, throttle, fuel, k, mass;
extern unsigned short scenario;
extern string scenario_description[];
extern vector3d position, previous_position, orientation, velocity;
extern parachute_status_t parachute_status;
extern autopilot_status_t autopilot_status;
extern int stabilized_attitude_angle;
extern ofstream fout;

#endif

// Function prototypes
void invert (double m[], double mout[]);
void xyz_euler_to_matrix (vector3d ang, double m[]);
vector3d matrix_to_xyz_euler (double m[]);
void normalize_quat (quat_t &q);
quat_t axis_to_quat (vector3d a, const double phi);
double project_to_sphere (const double r, const double x, const double y);
quat_t add_quats (quat_t q1, quat_t q2);
void quat_to_matrix (double m[], const quat_t q);
quat_t track_quats (const double p1x, const double p1y, const double p2x, const double p2y);
void microsecond_time (unsigned long long &t);
void fghCircleTable (double **sint, double **cost, const int n);
void glutOpenHemisphere (GLdouble radius, GLint slices, GLint stacks);
void glutMottledSphere (GLdouble radius, GLint slices, GLint stacks);
void glutCone (GLdouble base, GLdouble height, GLint slices, GLint stacks, bool closed);
void enable_lights (void);
void setup_lights (void);
void glut_print (float x, float y, string s);
double atmospheric_density (vector3d pos);
void draw_dial (double cx, double cy, double val, string title, string units);
void draw_control_bar (double tlx, double tly, double val, double red, double green, double blue, string title);
void draw_indicator_lamp (double tcx, double tcy, string off_text, string on_text, bool on);
void draw_instrument_window (void);
void display_help_arrows (void);
void display_help_prompt (void);
void display_help_text (void);
void draw_orbital_window (void);
void draw_parachute_quad (double d);
void draw_parachute (double d);
void load_parachute_texture(void);

void load_sun_texture(void);
void load_mercury_texture(void);
void load_venus_texture(void);
void load_earth_day_texture(void);
void load_earth_night_texture(void);
void load_moon_texture(void);
void load_mars_texture(void);
void load_jupiter_texture(void);
void load_saturn_texture(void);
void load_uranus_texture(void);
void load_neptune_texture(void);
void load_pluto_texture(void);
void load_death_star_texture(void);

bool generate_terrain_texture (void);
void update_closeup_coords (void);
void draw_closeup_window (void);
void draw_main_window (void);
void refresh_all_subwindows (void);
bool safe_to_deploy_parachute (void);
void update_visualization (void);
void attitude_stabilization (void);
void set_attitude_to_velocity(void);
void turn_attitude_about_x(float magnitude);
void turn_attitude_about_y(float magnitude);
void turn_attitude_about_z(float magnitude);
vector3d get_orientation_vector(void);
vector3d thrust_wrt_world (void);
void autopilot (void);
double calc_land_P (void);
bool will_land_in_full_thrust (void);
double speed_or_throttle_after_delay (bool isSpeed);
unsigned long get_throttle_buffer_length (void);
unsigned long get_throttle_buffer_pointer (void);
double* get_throttle_buffer (void);
void numerical_dynamics (void);
void initialize_simulation (void);
void update_lander_state (void);
void reset_simulation (void);
void set_orbital_projection_matrix (void);
void reshape_main_window (int width, int height);
void orbital_mouse_button (int button, int state, int x, int y);
void orbital_mouse_motion (int x, int y);
void closeup_mouse_button (int button, int state, int x, int y);
void closeup_mouse_motion (int x, int y);
void glut_special (int key, int x, int y);
void glut_key (unsigned char k, int x, int y);
void play_music(int previou_planet, int current_planet);

// Copied from https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
hsv   rgb2hsv(rgb in);
rgb   hsv2rgb(hsv in);