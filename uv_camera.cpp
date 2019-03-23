// Shawn Halayka
// This code and data is in the public domain.


#include "uv_camera.h"

const float pi = 4.0f*atan(1.0f);
const float pi_half = 0.5f*pi;
const float pi_2 = 2.0f*pi;
const float epsilon = 1e-6f;



uv_camera::uv_camera(void)
{
	u = v = 0;
	w = 4;
	fov = 45;
	near_plane = 1.0;
	far_plane = 4.0;
	win_x = win_y = 0;
}

void uv_camera::Set(const float u_rad, const float v_rad, const float w_metres, const float fov_deg, const int width_px, const int height_px, float src_near, float src_far)
{
	u = u_rad;
	v = v_rad;
	w = w_metres;
	near_plane = src_near;
	far_plane = src_far;

	static const float lock = epsilon * 1000.0f;

	if(u < -pi_half + lock)
		u = -pi_half + lock;
	else if(u > pi_half - lock)
		u = pi_half - lock;

	while(v < 0)
		v += pi_2;

	while(v > pi_2)
		v -= pi_2;

	if(w < 0)
		w = 0;
//	else if(w > 10000)
//		w = 10000;

	fov = fov_deg;
	win_x = width_px;
	win_y = height_px;

	Transform();
}


void uv_camera::Transform(void)
{
	Reset();
	Rotate();
	Translate();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(
		fov, 
		static_cast<GLfloat>(win_x)/static_cast<GLfloat>(win_y), 
		near_plane, far_plane);

	gluLookAt(
		eye.x, eye.y, eye.z, // Eye position.
		eye.x + look_at.x, eye.y + look_at.y, eye.z + look_at.z, // Look at position (not direction).
		up.x, up.y, up.z); // Up direction vector.
}

void uv_camera::Set(void)
{
	// Force a recalculation of the camera vectors and frustum.
	Set(u, v, w, fov, win_x, win_y, near_plane, far_plane);
}

void uv_camera::Set_Large_Screenshot(size_t num_cams, size_t cam_index_x, size_t cam_index_y)
{
	// No guarantees about the behaviour of this functionality. It wasn't tested a lot.

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Image plane reference:
	// http://www.songho.ca/opengl/gl_transform.html
    const float deg_to_rad = (1.0/360.0)*2*pi;
	float aspect = win_x/win_y;
    float tangent = tan((fov/2.0)*deg_to_rad);
    float height = near_plane * tangent; // Half height of near_plane plane.
    float width = height * aspect; // Half width of near_plane plane.

	float cam_width = 2*width/num_cams;
	float cam_height = 2*height/num_cams;

	float left = -width + cam_index_x*cam_width;
	float right = -width + (cam_index_x + 1)*cam_width;
	float bottom = -height + cam_index_y*cam_height;
	float top = -height + (cam_index_y + 1)*cam_height;

	// Instead of gluPerspective...
    glFrustum(left, right, bottom, top, near_plane, far_plane);

	gluLookAt(
		eye.x, eye.y, eye.z, // Eye position.
		eye.x + look_at.x, eye.y + look_at.y, eye.z + look_at.z, // Look at position (not direction).
		up.x, up.y, up.z); // Up direction vector.
}

void uv_camera::Reset(void)
{
	eye.zero();
	look_at.zero();
	up.zero();
	right.zero();

	look_at.z = -1;
	up.y = 1;
	right.x = 1;
}

void uv_camera::Rotate(void)
{
	// Rotate about the world x axis
	look_at.rotate_x(u);
	up.rotate_x(u);
	// Right only rotates on the x axis

	// Rotate about the world y axis
	look_at.rotate_y(v);
	up.rotate_y(v);
	right.rotate_y(v);
}

void uv_camera::Translate(void)
{
	// Place the eye directly across the sphere from the look-at vector's "tip",
	// Then scale the sphere radius by w
	eye.x = -look_at.x*w;
	eye.y = -look_at.y*w;
	eye.z = -look_at.z*w;
}

custom_math::vector_3 uv_camera::GetScreenRay(const int x, const int y, const int screen_width, const int screen_height)
{
	custom_math::vector_3 E(eye.x, eye.y, eye.z);
	custom_math::vector_3 T(look_at.x, look_at.y, look_at.z);
	custom_math::vector_3 w(up.x, up.y, up.z);
	w.normalize();

	custom_math::vector_3 t = T - E;
	custom_math::vector_3 b = w.cross(t);

	custom_math::vector_3 t_n = t;
	t_n.normalize();

	custom_math::vector_3 b_n = b;
	b_n.normalize();

	custom_math::vector_3 v_n = t_n.cross(b_n);

	double theta = fov * custom_math::pi / 180.0;
	double d = 1.0;
	double aspect = static_cast<double>(screen_width) / static_cast<double>(screen_height);
	double g_y = -d * tan(theta / 2.0);
	double g_x = g_y * aspect;

	custom_math::vector_3 q_x = b_n * (2.0*g_x) / (static_cast<double>(win_x) - 1.0);
	custom_math::vector_3 q_y = v_n * (2.0*g_y) / (static_cast<double>(win_y) - 1.0);

	custom_math::vector_3 p_1m = t_n * d - b_n * g_x - v_n * g_y;
	custom_math::vector_3 p_ij = p_1m + q_x * x + q_y * y;

	return p_ij;
}