#ifndef main_H
#define main_H

#include "uv_camera.h"
#include "custom_math.h"

#include <cstdlib>
#include <GL/glut.h>       //GLUT Library

#include <iostream>
using std::cout;
using std::endl;

#include <iomanip>
using std::setprecision;

#include <vector>
using std::vector;

#include <string>
using std::string;

#include <sstream>
using std::ostringstream;
using std::istringstream;

#include <fstream>
using std::ofstream;
using std::ifstream;

#include <set>
using std::set;

#include <map>
using std::map;

#include <utility>
using std::pair;


void idle_func(void);
void init_opengl(const int &width, const int &height);
void reshape_func(int width, int height);
void display_func(void);
void keyboard_func(unsigned char key, int x, int y);
void mouse_func(int button, int state, int x, int y);
void motion_func(int x, int y);
void passive_motion_func(int x, int y);

void render_string(int x, const int y, void *font, const string &text);
void draw_objects(void);


double court_width = 36;
double half_court_width = court_width / 2.0;
double court_length = 75;
double half_court_length = court_length / 2.0;
double net_height = 3;

custom_math::vector_3 server_pos(10, 4, 10);
custom_math::vector_3 server_velocity(-10, 3, -15);
custom_math::vector_3 server_angular_velocity(50, 5, 0);

vector<custom_math::vector_3> positions;

custom_math::vector_3 target_pos(15, 0, -15);


custom_math::vector_3 grav_and_magnus_acceleration(const custom_math::vector_3 &pos, const custom_math::vector_3 &vel)
{
	custom_math::vector_3 accel(0, -9.81, 0);

	custom_math::vector_3 magnus_accel = server_angular_velocity.cross(server_velocity)*0.5*0.1*0.1*1.0; // fluid_density*drag_coeff*ball_cross_section_area
	//magnus_accel.x /= 1;// accel = force/ball_mass
	//magnus_accel.y /= 1;// accel = force/ball_mass
	//magnus_accel.z /= 1;// accel = force/ball_mass

	return custom_math::vector_3(accel.x + magnus_accel.x, accel.y + magnus_accel.y, accel.z + magnus_accel.z);
}

void proceed_rk4(custom_math::vector_3 &pos, custom_math::vector_3 &vel)
{
	static const double one_sixth = 1.0 / 6.0;
	static const double dt = 0.0001;

	custom_math::vector_3 k1_velocity = vel;
	custom_math::vector_3 k1_acceleration = grav_and_magnus_acceleration(pos, k1_velocity);
	custom_math::vector_3 k2_velocity = vel + k1_acceleration * dt*0.5;
	custom_math::vector_3 k2_acceleration = grav_and_magnus_acceleration(pos + k1_velocity * dt*0.5, k2_velocity);
	custom_math::vector_3 k3_velocity = vel + k2_acceleration * dt*0.5;
	custom_math::vector_3 k3_acceleration = grav_and_magnus_acceleration(pos + k2_velocity * dt*0.5, k3_velocity);
	custom_math::vector_3 k4_velocity = vel + k3_acceleration * dt;
	custom_math::vector_3 k4_acceleration = grav_and_magnus_acceleration(pos + k3_velocity * dt, k4_velocity);

	vel += (k1_acceleration + (k2_acceleration + k3_acceleration)*2.0 + k4_acceleration)*one_sixth*dt;
	pos += (k1_velocity + (k2_velocity + k3_velocity)*2.0 + k4_velocity)*one_sixth*dt;
}

short unsigned int get_positions(vector<custom_math::vector_3> &p, const custom_math::vector_3 &target_position)
{
	p.clear();

	p.push_back(server_pos);

	custom_math::vector_3 last_pos = server_pos;
	custom_math::vector_3 last_vel = server_velocity;	

	while (1)
	{
		custom_math::vector_3 curr_pos = last_pos;
		custom_math::vector_3 curr_vel = last_vel;
		proceed_rk4(curr_pos, curr_vel);

		p.push_back(curr_pos);	

		// if collides with surface
		if (curr_pos.y <= 0)
			break;

		bool is_near_net = (curr_pos.z < 0 && last_pos.z >= 0);
		
		// if collides with net, reflect vector
		if (is_near_net &&
			curr_pos.y >= 0 && curr_pos.y <= net_height &&
			curr_pos.x >= -half_court_width && curr_pos.x <= half_court_width)
		{
			custom_math::vector_3 N(0, 0, 1);
			curr_vel = -(N * curr_vel.dot(N)*2.0 - curr_vel);
		}

		last_pos = curr_pos;
		last_vel = curr_vel;
	}

	return 0;
}



custom_math::vector_3 background_colour(0.5f, 0.5f, 0.5f);
custom_math::vector_3 control_list_colour(1.0f, 1.0f, 1.0f);

bool draw_axis = true;
bool draw_control_list = true;

uv_camera main_camera;

GLint win_id = 0;
GLint win_x = 800, win_y = 600;
float camera_w = 100;

float camera_fov = 45;
float camera_x_transform = 0;
float camera_y_transform = 0;
float u_spacer = 0.01;
float v_spacer = 0.5*u_spacer;
float w_spacer = 0.1;
float camera_near = 0.1;
float camera_far = 10000;

bool lmb_down = false;
bool mmb_down = false;
bool rmb_down = false;
int mouse_x = 0;
int mouse_y = 0;


#endif
