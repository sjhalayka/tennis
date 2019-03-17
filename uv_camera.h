#ifndef uv_camera_h
#define uv_camera_h

#include <cstdlib>
#include <GLUT/glut.h>       //GLUT Library

#include "custom_math.h"




// UV camera
//
// latitude:     | longitude:    | radius:       |
//       *_*_    |        ___    |        ___    |
//      */   \   |       /   \   |       /   \   |
// u:  *|  x  |  |  v:  |**x**|  |  w:  |  x**|  |
//      *\___/   |       \___/   |       \___/   |
//       * *     |               |               |
// 

class uv_camera
{
public:
	// Use as read-only
	float u, v, w, fov;
	float win_x, win_y;
    custom_math::vector_3 eye, look_at, up, right;
	float near_plane;
	float far_plane;

public:
	uv_camera(void);

	// Must initialize or change camera settings through these two functions
	void Set(const float u_rad, const float v_rad, const float w_metres, const float fov_deg, const int width_px, const int height_px, float src_near, float src_far);
	void Set(void);
	void Set_Large_Screenshot(size_t num_cams, size_t cam_num_x, size_t cam_num_y);
protected:
	void Transform(void);
	void Reset(void);
	void Rotate(void);
	void Translate(void);
};


#endif
