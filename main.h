#ifndef main_H
#define main_H

#include "uv_camera.h"
#include "custom_math.h"

#include <cstdlib>

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
const double net_height_at_centre = 0.914; // 3 feet
const double net_height_at_edges = 1.07; // 3.5 feet
double ball_radius = 0.0335;

custom_math::vector_3 in_server_pos(3, 1, 3);
custom_math::vector_3 in_server_vel(-3, 1, -5);
custom_math::vector_3 in_server_ang_vel(0, 100, 0);

custom_math::vector_3 in_out_target_pos(5, 0, -5);

custom_math::vector_3 out_server_vel;
custom_math::vector_3 out_server_ang_vel;

vector<custom_math::vector_3> out_path;


double dt = 0.01;

const size_t num_vectors = 10;
const size_t num_length_adjustment_iterations = 5;	
const size_t max_bounce_count = 1;//;
const bool pro_mode = true;



void (*integrator_func_pointer)(custom_math::vector_3 &, custom_math::vector_3 &, const custom_math::vector_3 &);


#define REGION_PLAYER_IN_BOUNDS 0
#define REGION_PLAYER_OUT_OF_BOUNDS 1
#define REGION_OPPONENT_IN_BOUNDS 2
#define REGION_OPPONENT_OUT_OF_BOUNDS 3


double get_net_height(double x)
{
	if (abs(x) >= half_court_width)
		return net_height_at_edges;

	x = abs(x) / half_court_width;
	return net_height_at_centre + (net_height_at_edges - net_height_at_centre) * x;
}

// todo: add posts


class posts
{
public:
	vector<custom_math::triangle> tris;

	void init_regulation_posts(double width, double height, double half_court_width)
	{
		tris.clear();

		// do one post
		double begin = -half_court_width;
		double end = -half_court_width - width;
		
		custom_math::vector_3 p0(end, height, 0);
		custom_math::vector_3 p1(begin, height, 0);
		custom_math::vector_3 p2(begin, 0, 0);
		custom_math::vector_3 p3(end, 0, 0);

		custom_math::triangle t1;
		t1.A = p0;
		t1.B = p1;
		t1.C = p3;

		custom_math::triangle t2;
		t2.A = p1;
		t2.B = p2;
		t2.C = p3;

		tris.push_back(t1);
		tris.push_back(t2);

		// now the other triangle
		begin = half_court_width;
		end = half_court_width + width;

		p0 = custom_math::vector_3(end, height, 0);
		p1 = custom_math::vector_3(begin, height, 0);
		p2 = custom_math::vector_3(begin, 0, 0);
		p3 = custom_math::vector_3(end, 0, 0);

		t1.A = p0;
		t1.B = p1;
		t1.C = p3;

		t2.A = p1;
		t2.B = p2;
		t2.C = p3;

		tris.push_back(t1);
		tris.push_back(t2);
	}
};




class net
{
public:
	vector<custom_math::triangle> tris;

	void init_regulation_net(double net_height_at_centre, double net_height_at_edges, double half_court_width)
	{
		tris.clear();

		custom_math::vector_3 p0(-half_court_width, net_height_at_edges, 0);
		custom_math::vector_3 p1(0, net_height_at_centre, 0);
		custom_math::vector_3 p2(half_court_width, net_height_at_edges, 0);
		custom_math::vector_3 p3(half_court_width, 0, 0);
		custom_math::vector_3 p4(-half_court_width, 0, 0);

		custom_math::triangle t1;
		t1.A = p0;
		t1.B = p1;
		t1.C = p4;

		custom_math::triangle t2;
		t2.A = p1;
		t2.B = p2;
		t2.C = p3;

		custom_math::triangle t3;
		t3.A = p1;
		t3.B = p3;
		t3.C = p4;

		tris.push_back(t1);
		tris.push_back(t2);
		tris.push_back(t3);
	}
};


net n;
posts the_posts;


// http://realtimecollisiondetection.net/blog/?p=103
bool is_separated(custom_math::vector_3 A, custom_math::vector_3 B, custom_math::vector_3 C, custom_math::vector_3 P, double r)
{
	A = A - P;
	B = B - P;
	C = C - P;

	double rr = r * r;
	custom_math::vector_3 V = (B - A).cross(C - A);
	double d = A.dot(V);
	double e = V.dot(V);
	bool sep1 = d * d > rr * e;

	if (sep1)
		return true;

	double aa = A.dot(A);
	double ab = A.dot(B);
	double ac = A.dot(C);
	bool sep2 = (aa > rr) & (ab > aa) & (ac > aa);

	if (sep2)
		return true;

	double bb = B.dot(B);
	double bc = B.dot(C);
	bool sep3 = (bb > rr) & (ab > bb) & (bc > bb);

	if (sep3)
		return true;

	double cc = C.dot(C);
	bool sep4 = (cc > rr) & (ac > cc) & (bc > cc);

	if (sep4)
		return true;

	custom_math::vector_3 AB = B - A;
	custom_math::vector_3 BC = C - B;
	custom_math::vector_3 CA = A - C;

	double d1 = ab - aa;
	double e1 = AB.dot(AB);

	custom_math::vector_3 Q1 = A * e1 - AB * d1;
	custom_math::vector_3 QC = C * e1 - Q1;
	bool sep5 = (Q1.dot(Q1) > rr * e1 * e1) & (Q1.dot(QC) > 0);

	if (sep5)
		return true;

	double d2 = bc - bb;
	double e2 = BC.dot(BC);

	custom_math::vector_3 Q2 = B * e2 - BC * d2;
	custom_math::vector_3 QA = A * e2 - Q2;
	bool sep6 = (Q2.dot(Q2) > rr * e2 * e2) & (Q2.dot(QA) > 0);

	if (sep6)
		return true;

	double d3 = ac - cc;
	double e3 = CA.dot(CA);

	custom_math::vector_3 Q3 = C * e3 - CA * d3;
	custom_math::vector_3 QB = B * e3 - Q3;
	bool sep7 = (Q3.dot(Q3) > rr * e3 * e3) & (Q3.dot(QB) > 0);

	if (sep7)
		return true;

	return false;
}

bool is_colliding(custom_math::vector_3 A, custom_math::vector_3 B, custom_math::vector_3 C, custom_math::vector_3 P, double r)
{
	return !is_separated(A, B, C, P, r);
}

bool is_colliding(const vector<custom_math::triangle> &tris, custom_math::vector_3 P, double r)
{
	for (size_t i = 0; i < tris.size(); i++)
		if (true == is_colliding(tris[i].A, tris[i].B, tris[i].C, P, r))
			return true;

	return false;
}




size_t get_first_ground_hit(const vector<custom_math::vector_3> &path)
{
	size_t index = 0;

	for (size_t i = 0; i < path.size(); i++)
	{
		if (path[i].y <= 0.0)
		{
			index = i;
			break;
		}
	}

	return index;
}

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

// https://en.wikipedia.org/wiki/Symplectic_integrator
// Also known as Verlet integration
void proceed_symplectic2(custom_math::vector_3 &pos, custom_math::vector_3 &vel, const custom_math::vector_3 &ang_vel)
{
	static const double c[2] = { 0, 1 };
	static const double d[2] = { 0.5, 0.5 };

// 	pos += vel * c[0] * dt; // first element c[0] is always 0
	vel += acceleration(pos, vel, ang_vel) * d[0] * dt;

	pos += vel * c[1] * dt;
	vel += acceleration(pos, vel, ang_vel) * d[1] * dt;
}

// https://www.gamedev.net/forums/topic/701376-weird-circular-orbit-problem/?do=findComment&comment=5402054
// https://en.wikipedia.org/wiki/Symplectic_integrator
void proceed_symplectic4(custom_math::vector_3 &pos, custom_math::vector_3 &vel, const custom_math::vector_3 &ang_vel)
{
	static double const cr2 = pow(2.0, 1.0 / 3.0);

	static const double c[4] =
	{
		1.0 / (2.0*(2.0 - cr2)),
		(1.0 - cr2) / (2.0*(2.0 - cr2)),
		(1.0 - cr2) / (2.0*(2.0 - cr2)),
		1.0 / (2.0*(2.0 - cr2))
	};

	static const double d[4] =
	{
		1.0 / (2.0 - cr2),
		-cr2 / (2.0 - cr2),
		1.0 / (2.0 - cr2),
		0.0
	};

	pos += vel * c[0] * dt;
	vel += acceleration(pos, vel, ang_vel) * d[0] * dt;

	pos += vel * c[1] * dt;
	vel += acceleration(pos, vel, ang_vel) * d[1] * dt;

	pos += vel * c[2] * dt;
	vel += acceleration(pos, vel, ang_vel) * d[2] * dt;

	pos += vel * c[3] * dt;
//	vel += acceleration(pos, vel, ang_vel) * d[3] * dt; // last element d[3] is always 0
}

void proceed_Euler(custom_math::vector_3 &pos, custom_math::vector_3 &vel, const custom_math::vector_3 &ang_vel)
{
	vel += acceleration(pos, vel, ang_vel) * dt;
	pos += vel * dt;
}

inline void proceed_RK2(custom_math::vector_3 &pos, custom_math::vector_3 &vel, const custom_math::vector_3 &ang_vel)
{
	custom_math::vector_3 k1_velocity = vel;
	custom_math::vector_3 k1_acceleration = acceleration(pos, k1_velocity, ang_vel);
	custom_math::vector_3 k2_velocity = vel + k1_acceleration * dt*0.5;
	custom_math::vector_3 k2_acceleration = acceleration(pos + k1_velocity * dt*0.5, k2_velocity, ang_vel);

	vel += k2_acceleration * dt;
	pos += k2_velocity * dt;
}

void proceed_RK4(custom_math::vector_3 &pos, custom_math::vector_3 &vel, const custom_math::vector_3 &ang_vel)
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
	custom_math::vector_3 target_position,
	size_t max_bounces)
{
	p.clear();

	p.push_back(server_position);

	custom_math::vector_3 last_pos = server_position;
	custom_math::vector_3 last_vel = server_velocity;

	size_t bounce_count = 0;

	while (p.size() < 100000) // abort those paths that do not land on the ground in sufficient time
	{
		custom_math::vector_3 curr_pos = last_pos;
		custom_math::vector_3 curr_vel = last_vel;
		integrator_func_pointer(curr_pos, curr_vel, server_angular_velocity);
		p.push_back(curr_pos);

		// if collides with the ground
		if (curr_pos.y < 0 && last_pos.y >= 0)
		{
			// Take a step back
			curr_pos = last_pos;
			curr_vel = last_vel;
			p.pop_back();

			// Crank up the resolution to find the collision location
			double default_dt = dt;
			dt = 0.0001;

			// Step forward until the ball hits the ground
			while (curr_pos.y > 0)
			{
				integrator_func_pointer(curr_pos, curr_vel, server_angular_velocity);
				p.push_back(curr_pos);
			}

			// Reset to the default resolution
			dt = default_dt;

			custom_math::vector_3 N(0, 1, 0);
			curr_vel = -(N * curr_vel.dot(N)*2.0 - curr_vel);

			if (bounce_count == max_bounces)
				break;

			bounce_count++;
		}

		bool is_near_net = (curr_pos.z < 0 && last_pos.z >= 0);

		// if collides with net
		if (is_near_net)
		{
			// Take a step back
			curr_pos = last_pos;
			curr_vel = last_vel;
			p.pop_back();

			// Crank up the resolution to find the collision location
			double default_dt = dt;
			dt = 0.0001;

			// Step forward until the ball hits the net's z location
			while (curr_pos.z > 0)
			{
				integrator_func_pointer(curr_pos, curr_vel, server_angular_velocity);
				p.push_back(curr_pos);
			}

			// Reset to the default resolution
			dt = default_dt;

			// Check to see if the ball collides with any of the posts' triangles
			if (is_colliding(the_posts.tris, curr_pos, ball_radius))
			{
				custom_math::vector_3 reflected;
				custom_math::vector_3 N(0, 0, 1);
				reflected = -(N * curr_vel.dot(N)*2.0 - curr_vel);
				curr_vel = reflected;
			}
			// Check to see if the ball collides with any of the net's triangles
			else if (is_colliding(n.tris, curr_pos, ball_radius))
			{
				double net_height_at_collision_location = get_net_height(curr_pos.x);

				custom_math::vector_3 up(0, curr_vel.length(), 0);

				if (curr_pos.y == net_height_at_collision_location)
				{
					cout << "up" << endl;
					curr_vel = up;
				}
				else if(curr_pos.y > net_height_at_collision_location)
				{
					// go from 0 to 1 and lerp(curr_vel, up)
					double t = (curr_pos.y - net_height_at_collision_location) / (ball_radius*2) + 0.5;

					cout << "lerp curr_vel up" << endl;
					cout << t << endl;

					double curr_vel_len = curr_vel.length();
					curr_vel = lerp(curr_vel, up, t);
					curr_vel.normalize();
					curr_vel *= curr_vel_len;
				}
				else if(curr_pos.y < net_height_at_collision_location)
				{
					if (curr_pos.y < (net_height_at_collision_location - ball_radius))
					{
						cout << "reflected" << endl;

						custom_math::vector_3 reflected;
						custom_math::vector_3 N(0, 0, 1);
						reflected = -(N * curr_vel.dot(N)*2.0 - curr_vel);

						curr_vel = reflected;
					}
					else
					{
						custom_math::vector_3 reflected;
						custom_math::vector_3 N(0, 0, 1);
						reflected = -(N * curr_vel.dot(N)*2.0 - curr_vel);

						// go from 0 to 1 and lerp(up, reflected)
						double t = (curr_pos.y - net_height_at_collision_location) / (ball_radius * 2) + 1;

						cout << "lerp up reflected" << endl;
						cout << t << endl;

						double curr_vel_len = curr_vel.length();
						curr_vel = lerp(up, reflected, t);
						curr_vel.normalize();
						curr_vel *= curr_vel_len;
					}
				}
			}
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
		target_position,
		0);

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
			target_position,
			0);
	}

	// rotate vectors on y axis to get closer to the target position
	custom_math::vector_3 end_position = p[p.size() - 1];
	custom_math::vector_3 v1 = server_position - target_position;
	custom_math::vector_3 v2 = server_position - end_position;
	v1.normalize();
	v2.normalize();

	double angle = acos(v1.dot(v2));

	server_velocity.rotate_y(-angle);

	get_path(
		p,
		server_position,
		server_velocity,
		server_angular_velocity,
		target_position,
		0);

	return 0;
}

void get_targets(
	custom_math::vector_3 in_server_pos,
	custom_math::vector_3 in_server_vel,
	custom_math::vector_3 in_server_ang_vel,
	custom_math::vector_3 &in_out_target_pos,
	custom_math::vector_3 &out_server_vel,
	custom_math::vector_3 &out_server_ang_vel,
	vector<custom_math::vector_3> &path)
{
	vector< vector<custom_math::vector_3> > paths;
	paths.resize(num_vectors);

	in_server_vel = in_out_target_pos - in_server_pos;
	const double server_vel_len = in_server_vel.length();
	const custom_math::vector_3 up(0, server_vel_len, 0);
	double step_size = 1.0 / (double)(num_vectors);

	vector<custom_math::vector_3> server_vels;
	vector<custom_math::vector_3> server_ang_vels;

	for (size_t i = 0; i < num_vectors; i++)
	{
		in_server_vel = lerp(in_out_target_pos - in_server_pos, up, i*step_size);
		in_server_vel.normalize();
		in_server_vel *= server_vel_len;

		//cout << "temp sv " << in_server_vel.x << " " << in_server_vel.y << " " << in_server_vel.z << endl;

		server_vels.push_back(in_server_vel);
		server_ang_vels.push_back(in_server_ang_vel);

		get_path(
			paths[i], 
			in_server_pos, 
			in_server_vel, 
			in_server_ang_vel, 
			in_out_target_pos,
			0);
	}

	// find the closest path end that lands in the opponent's court
	vector<d> index_double;

	for (size_t i = 0; i < num_vectors; i++)
	{
		custom_math::vector_3 end_point = paths[i][paths[i].size() - 1];

//		cout << "count " << paths[i].size() << endl;

//		cout << "end point " << end_point.x << " " << end_point.y << " " << end_point.z << endl;

		custom_math::vector_3 diff = end_point - in_out_target_pos;

		if (pro_mode || REGION_OPPONENT_IN_BOUNDS == get_ball_region(end_point.x, end_point.z))
		{
			double val = diff.length();

			d dval;
			dval.index = i;
			dval.val = val;

			index_double.push_back(dval);
		}
	}
	
	if (0 == index_double.size())
	{
		in_out_target_pos.z -= 1.0;

		get_targets(
			in_server_pos, in_server_vel, in_server_ang_vel, in_out_target_pos,
			out_server_vel,
			out_server_ang_vel,
			out_path);

		return;
	}

	sort(index_double.begin(), index_double.end());

	hone_path(
		paths[index_double[0].index],
		in_server_pos,
		server_vels[index_double[0].index],
		server_ang_vels[index_double[0].index],
		in_out_target_pos,
		num_length_adjustment_iterations);

	get_path(
		paths[index_double[0].index],
		in_server_pos,
		server_vels[index_double[0].index],
		server_ang_vels[index_double[0].index],
		in_out_target_pos,
		max_bounce_count);

	out_server_vel = server_vels[index_double[0].index];
	out_server_ang_vel = server_ang_vels[index_double[0].index];
	path = paths[index_double[0].index];
}



custom_math::vector_3 background_colour(0.25f, 0.25f, 0.25f);
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
float u_spacer = 0.01f;
float v_spacer = 0.5f*u_spacer;
float w_spacer = 0.1f;
float camera_near = 0.1f;
float camera_far = 10000.0f;

bool lmb_down = false;
bool mmb_down = false;
bool rmb_down = false;
int mouse_x = 0;
int mouse_y = 0;

float last_click_float_x = 0;
float last_click_float_y = 0;
float last_click_float_z = 0;

#endif
