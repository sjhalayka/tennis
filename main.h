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



vector<vector<custom_math::vector_3> > all_paths;


class d
{
public:
	size_t index;
	double val;

	bool operator<(const d &rhs) const
	{
		if (val < rhs.val)
			return true;

		return false;
	}
};



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


double court_width = 10.9728; // 36 feet
double half_court_width = court_width / 2.0;
double court_length = 22.86; // 75 feet
double half_court_length = court_length / 2.0;
double net_height = 0.9144; // 3 feet

custom_math::vector_3 in_server_pos(3, 1, 3);
custom_math::vector_3 in_server_vel(-3, 1, -5);
custom_math::vector_3 in_server_ang_vel(0, 100, 0);
custom_math::vector_3 in_target_pos(5, 0, -5);

custom_math::vector_3 out_server_vel_1;
custom_math::vector_3 out_server_ang_vel_1;
custom_math::vector_3 out_server_vel_2;
custom_math::vector_3 out_server_ang_vel_2;

vector<custom_math::vector_3> out_p_1;
vector<custom_math::vector_3> out_p_2;

double dt = 0.1;

const size_t num_vectors = 10;
const size_t num_hone_iterations = 1;
const size_t num_length_adjustment_iterations = 20;	

#define REGION_PLAYER_IN_BOUNDS 0
#define REGION_PLAYER_OUT_OF_BOUNDS 1
#define REGION_OPPONENT_IN_BOUNDS 2
#define REGION_OPPONENT_OUT_OF_BOUNDS 3



size_t get_ball_region(const double x, const double z)
{
	if (z > 0) // landed on player's side
	{
		if (x > -half_court_width && x < half_court_width && z < half_court_length)
			return REGION_PLAYER_IN_BOUNDS;
		else
			return REGION_PLAYER_OUT_OF_BOUNDS;
	}
	else // landed on opponent's side
	{
		if (x > -half_court_width && x < half_court_width && z > -half_court_length)
			return REGION_OPPONENT_IN_BOUNDS;
		else
			return REGION_OPPONENT_OUT_OF_BOUNDS;
	}

	return 4; // this will never be executed
}



custom_math::vector_3 lerp(const custom_math::vector_3 &A, const custom_math::vector_3 &B, double t)
{
	custom_math::vector_3 a = A;
	a *= t;

	custom_math::vector_3 b = B;
	b *= (1.0 - t);

	return a + b;
}

custom_math::vector_3 acceleration(custom_math::vector_3 pos, custom_math::vector_3 vel, custom_math::vector_3 ang_vel)
{
	// http://twu.tennis-warehouse.com/learning_center/aerodynamics2.php
	const double air_density = 1.225;
	const double lift_coeff = 0.05;
	const double drag_coeff = 0.55;
	const double ball_cross_section = 0.0034;
	const double ball_mass = 0.0585;

	// Gravitation, in metres per second, per second
	custom_math::vector_3 grav_accel(0, -9.81, 0);

	// Magnus effect, in metres per second, per second
	// http://farside.ph.utexas.edu/teaching/329/lectures/node43.html
	// http://spiff.rit.edu/richmond/baseball/traj/traj.html
	custom_math::vector_3 magnus_accel = vel.cross(ang_vel)*0.5*air_density*lift_coeff*ball_cross_section / ball_mass;

	// Wind and drag, in metres per second, per second
	custom_math::vector_3 wind_vel(2.77778, 0, 0); // in metres per second
	custom_math::vector_3 drag_vel = wind_vel - vel;
	custom_math::vector_3 drag_accel = drag_vel*drag_vel.length()*0.5*air_density*drag_coeff*ball_cross_section / ball_mass;
	
	return grav_accel + magnus_accel + drag_accel;
}

void proceed_Euler(custom_math::vector_3 &pos, custom_math::vector_3 &vel, const custom_math::vector_3 &ang_vel)
{
	vel += acceleration(pos, vel, ang_vel) * dt;
	pos += vel * dt;
}

void proceed_rk4(custom_math::vector_3 &pos, custom_math::vector_3 &vel, const custom_math::vector_3 &ang_vel)
{
	static const double one_sixth = 1.0 / 6.0;

	custom_math::vector_3 k1_velocity = vel;
	custom_math::vector_3 k1_acceleration = acceleration(pos, k1_velocity, ang_vel);
	custom_math::vector_3 k2_velocity = vel + k1_acceleration * dt*0.5;
	custom_math::vector_3 k2_acceleration = acceleration(pos + k1_velocity * dt*0.5, k2_velocity, ang_vel);
	custom_math::vector_3 k3_velocity = vel + k2_acceleration * dt*0.5;
	custom_math::vector_3 k3_acceleration = acceleration(pos + k2_velocity * dt*0.5, k3_velocity, ang_vel);
	custom_math::vector_3 k4_velocity = vel + k3_acceleration * dt;
	custom_math::vector_3 k4_acceleration = acceleration(pos + k3_velocity * dt, k4_velocity, ang_vel);

	vel += (k1_acceleration + (k2_acceleration + k3_acceleration)*2.0 + k4_acceleration)*one_sixth*dt;
	pos += (k1_velocity + (k2_velocity + k3_velocity)*2.0 + k4_velocity)*one_sixth*dt;
}

short unsigned int get_path(
	vector<custom_math::vector_3> &p,
	custom_math::vector_3 server_position,
	custom_math::vector_3 server_velocity,
	custom_math::vector_3 server_angular_velocity,
	custom_math::vector_3 target_position)
{
	p.clear();

	p.push_back(server_position);

	custom_math::vector_3 last_pos = server_position;
	custom_math::vector_3 last_vel = server_velocity;

	while (p.size() < 100000) // abort those paths that do not land on the ground in sufficient time
	{
		custom_math::vector_3 curr_pos = last_pos;	
		custom_math::vector_3 curr_vel = last_vel;
		//proceed_rk4(curr_pos, curr_vel, server_angular_velocity);
		proceed_Euler(curr_pos, curr_vel, server_angular_velocity);

		p.push_back(curr_pos);

		// if collides with the ground
		if (curr_pos.y < 0 && last_pos.y >= 0)
		{
			// Take a step back
			curr_pos = last_pos;
			curr_vel = last_vel;
			p.pop_back();

			// Crank up the resolution to find the collision location
			dt = 0.0001;

			// Step forward until the ball hits the ground
			while (curr_pos.y > 0)
			{
				//proceed_rk4(curr_pos, curr_vel, server_angular_velocity);
				proceed_Euler(curr_pos, curr_vel, server_angular_velocity);
				p.push_back(curr_pos);
			}
			
			// Reset to the default resolution
			dt = 0.01;

			break;
		}

		bool is_near_net = (curr_pos.z < 0 && last_pos.z >= 0);

		// if collides with net
		if (is_near_net &&
			curr_pos.y >= 0 && curr_pos.y <= net_height &&
			curr_pos.x >= -half_court_width && curr_pos.x <= half_court_width)
		{
			// Take a step back
			curr_pos = last_pos;
			curr_vel = last_vel;
			p.pop_back();

			// Crank up the resolution to find the collision location
			dt = 0.0001;

			// Step forward until the ball hits the net
			while (curr_pos.z > 0)
			{
				//proceed_rk4(curr_pos, curr_vel, server_angular_velocity);
				proceed_Euler(curr_pos, curr_vel, server_angular_velocity);
				p.push_back(curr_pos);
			}

			// Reset to the default resolution
			dt = 0.01;

			// reflect vector
			custom_math::vector_3 N(0, 0, 1);
			curr_vel = -(N * curr_vel.dot(N)*2.0 - curr_vel);
		}

		last_pos = curr_pos;
		last_vel = curr_vel;
	}

	return 0;
}

short unsigned int hone_path(
	vector<custom_math::vector_3> &p,
	custom_math::vector_3 server_position,
	custom_math::vector_3 &server_velocity,
	custom_math::vector_3 &server_angular_velocity,
	custom_math::vector_3 target_position,
	const size_t num_length_adjustment_iterations)
{
	get_path(
		p,
		server_position,
		server_velocity,
		server_angular_velocity,
		target_position);

	for (size_t i = 0; i < num_length_adjustment_iterations; i++)
	{
		// adjust velocity length to get closer to the target position
		custom_math::vector_3 begin_pos = p[0];
		custom_math::vector_3 end_pos = p[p.size() - 1];
		custom_math::vector_3 diff_a = end_pos - begin_pos;
		custom_math::vector_3 diff_b = target_position - begin_pos;
		double len_a = diff_a.length();
		double len_b = diff_b.length();
		double slope = len_a / len_b;

		server_velocity /= slope;

		get_path(
			p,
			server_position,
			server_velocity,
			server_angular_velocity,
			target_position);
	}

	// rotate vectors on y axis to get closer to the target position
	custom_math::vector_3 end_position = p[p.size() - 1];
	custom_math::vector_3 v1 = server_position - target_position;
	custom_math::vector_3 v2 = server_position - end_position;
	v1.normalize();
	v2.normalize();

	double angle = acos(v1.dot(v2));

	server_velocity.rotate_y(-angle);
	//server_angular_velocity.rotate_y(-angle);

	get_path(
		p,
		server_position,
		server_velocity,
		server_angular_velocity,
		target_position);

	return 0;
}

void get_targets(
	custom_math::vector_3 in_server_pos,
	custom_math::vector_3 in_server_vel,
	custom_math::vector_3 in_server_ang_vel,
	custom_math::vector_3 in_target_pos,
	custom_math::vector_3 &out_server_vel_1,
	custom_math::vector_3 &out_server_ang_vel_1,
	custom_math::vector_3 &out_server_vel_2,
	custom_math::vector_3 &out_server_ang_vel_2,
	vector<custom_math::vector_3> &p_1,
	vector<custom_math::vector_3> &p_2
)
{
	vector< vector<custom_math::vector_3> > paths;
	paths.resize(num_vectors);

	in_server_vel = in_target_pos - in_server_pos;
	const double server_vel_len = in_server_vel.length();
	const custom_math::vector_3 up(0, server_vel_len, 0);
	double step_size = 1.0 / num_vectors;

	vector<custom_math::vector_3> server_vels;
	vector<custom_math::vector_3> server_ang_vels;

	for (size_t i = 0; i < num_vectors; i++)
	{
		in_server_vel = lerp(in_target_pos - in_server_pos, up, i*step_size);
		in_server_vel.normalize();
		in_server_vel *= server_vel_len;

		server_vels.push_back(in_server_vel);
		server_ang_vels.push_back(in_server_ang_vel);

		get_path(paths[i], in_server_pos, in_server_vel, in_server_ang_vel, in_target_pos);
	}

	all_paths = paths;

	// find two closest path ends
	vector<d> index_double;

	for (size_t i = 0; i < num_vectors; i++)
	{
		custom_math::vector_3 end_point = paths[i][paths[i].size() - 1];
		custom_math::vector_3 diff = end_point - in_target_pos;

		double val = diff.length();

		d dval;
		dval.index = i;
		dval.val = val;

		index_double.push_back(dval);
	}

	sort(index_double.begin(), index_double.end());
	size_t smallest_index = index_double[0].index;
	size_t second_smallest_index = index_double[1].index;

	for (size_t i = 0; i < num_vectors; i++)
	{
		if (i == smallest_index)
		{
			for(size_t j = 0; j < num_hone_iterations; j++)
				hone_path(paths[i], in_server_pos, server_vels[i], server_ang_vels[i], in_target_pos, num_length_adjustment_iterations);
		
			out_server_vel_1 = server_vels[i];
			out_server_ang_vel_1 = server_ang_vels[i];
			p_1 = paths[i];
		}

		if (i == second_smallest_index)
		{
			for (size_t j = 0; j < num_hone_iterations; j++)
				hone_path(paths[i], in_server_pos, server_vels[i], server_ang_vels[i], in_target_pos, num_length_adjustment_iterations);
		
			out_server_vel_2 = server_vels[i];
			out_server_ang_vel_2= server_ang_vels[i];
			p_2 = paths[i];
		}
	}
}



custom_math::vector_3 background_colour(0.5f, 0.5f, 0.5f);
custom_math::vector_3 control_list_colour(1.0f, 1.0f, 1.0f);

bool draw_axis = true;
bool draw_control_list = true;

uv_camera main_camera;

GLint win_id = 0;
GLint win_x = 800, win_y = 600;
float camera_w = 30;

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
