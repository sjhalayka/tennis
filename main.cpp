#include "main.h"

#pragma comment(lib, "freeglut.lib")

int main(int argc, char **argv)
{
	cout << setprecision(20) << endl;

	size_t num_vectors = 10;

	paths.resize(num_vectors);

	server_vel = target_pos - server_pos;
	const double server_vel_len = server_vel.length();
	const custom_math::vector_3 up(0, server_vel_len, 0);
	double step_size = 1.0 / num_vectors;

	for (size_t i = 0; i < num_vectors; i++)
	{
		server_vel = lerp(target_pos - server_pos, up, i*step_size);
		server_vel.normalize();
		server_vel *= server_vel_len;

		get_path(paths[i], server_pos, server_vel, server_ang_vel, target_pos);
	}


	// find two closest path ends

	size_t largest_index = 0;
	size_t second_largest_index = 0;

	for (size_t i = 0; i < num_vectors; i++)
	{


	}



	glutInit(&argc, argv);
	init_opengl(win_x, win_y);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
	glutKeyboardFunc(keyboard_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutPassiveMotionFunc(passive_motion_func);
	//glutIgnoreKeyRepeat(1);
	glutMainLoop();
	glutDestroyWindow(win_id);

	return 0;
}

void idle_func(void)
{
    glutPostRedisplay();
}

void init_opengl(const int &width, const int &height)
{
	win_x = width;
	win_y = height;

	if(win_x < 1)
		win_x = 1;

	if(win_y < 1)
		win_y = 1;

	glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE|GLUT_DEPTH);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("orbit");

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDepthMask(GL_TRUE);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glClearColor(background_colour.x, background_colour.y, background_colour.z, 1);
	glClearDepth(1.0f);

	main_camera.Set(0, 0, camera_w, camera_fov, win_x, win_y, camera_near, camera_far);
	main_camera.u += custom_math::pi/6.0;
	main_camera.v += custom_math::pi / 4.0;
	main_camera.Set();
}

void reshape_func(int width, int height)
{
	win_x = width;
	win_y = height;

	if(win_x < 1)
		win_x = 1;

	if(win_y < 1)
		win_y = 1;

	glutSetWindow(win_id);
	glutReshapeWindow(win_x, win_y);
	glViewport(0, 0, win_x, win_y);

	main_camera.Set(main_camera.u, main_camera.v, main_camera.w, main_camera.fov, win_x, win_y, camera_near, camera_far);
}

// Text drawing code originally from "GLUT Tutorial -- Bitmap Fonts and Orthogonal Projections" by A R Fernandes
void render_string(int x, const int y, void *font, const string &text)
{
	for(size_t i = 0; i < text.length(); i++)
	{
		glRasterPos2i(x, y);
		glutBitmapCharacter(font, text[i]);
		x += glutBitmapWidth(font, text[i]) + 1;
	}
}
// End text drawing code.

void draw_objects(void)
{
	glDisable(GL_LIGHTING);

	glPushMatrix();

	glDisable(GL_CULL_FACE);

	glBegin(GL_QUADS);

	glColor3f(1.0f, 0.5, 0.0f);

	glVertex3f(-half_court_width, 0, -half_court_length);
	glVertex3f(-half_court_width, 0, half_court_length);
	glVertex3f(half_court_width, 0, half_court_length);
	glVertex3f(half_court_width, 0, -half_court_length);

	glColor3f(0.0f, 0.5, 1.0f);

	glVertex3f(-half_court_width, 0, 0);
	glVertex3f(half_court_width, 0, 0);
	glVertex3f(half_court_width, net_height, 0);
	glVertex3f(-half_court_width, net_height, 0);

	glEnd();

	glEnable(GL_CULL_FACE);



	glLineWidth(2.0);

	glBegin(GL_LINES);

	glColor3f(1, 1, 1);

	glVertex3f(-half_court_width, 0, half_court_length);
	glVertex3f(half_court_width, 0, half_court_length);

	glVertex3f(-half_court_width, 0, -half_court_length);
	glVertex3f(half_court_width, 0, -half_court_length);

	glVertex3f(-half_court_width, 0, half_court_length);
	glVertex3f(-half_court_width, 0, -half_court_length);

	glVertex3f(half_court_width, 0, half_court_length);
	glVertex3f(half_court_width, 0, -half_court_length);

	//glVertex3f(-half_court_width, 0, 0);
	//glVertex3f(half_court_width, 0, 0);

	glEnd();


	glPointSize(10.0f);

	glBegin(GL_POINTS);

	glColor3f(1, 1, 1);
	glVertex3f(server_pos.x, server_pos.y, server_pos.z);

	glColor3f(0, 0, 0);
	glVertex3f(target_pos.x, target_pos.y, target_pos.z);

	glEnd();



	glLineWidth(1.0f);

	glBegin(GL_LINES);

	glColor3f(0.5, 0.5, 0.5);
	glVertex3f(server_pos.x, server_pos.y, server_pos.z);
	glVertex3f(server_pos.x, 0, server_pos.z);

	glColor3f(1, 1, 1);
	glVertex3f(server_pos.x, server_pos.y, server_pos.z);
	glVertex3f(server_pos.x + server_vel.x, server_pos.y + server_vel.y, server_pos.z + server_vel.z);

	glColor3f(0, 0, 0);
	glVertex3f(server_pos.x, server_pos.y, server_pos.z);
	glVertex3f(server_pos.x + server_ang_vel.x, server_pos.y + server_ang_vel.y, server_pos.z + server_ang_vel.z);

	glEnd();


	glColor3f(1.0, 0.0, 0.0);

	for (size_t i = 0; i < paths.size(); i++)
	{
		glBegin(GL_LINE_STRIP);

		for (size_t j = 0; j < paths[i].size(); j++)
			glVertex3f(paths[i][j].x, paths[i][j].y, paths[i][j].z);

		glEnd();
	}
    
	// If we do draw the axis at all, make sure not to draw its outline.
	if(true == draw_axis)
	{
		//glBegin(GL_LINES);

		//glColor3f(1, 0, 0);
		//glVertex3f(0, 0, 0);
		//glVertex3f(1, 0, 0);
		//glColor3f(0, 1, 0);
		//glVertex3f(0, 0, 0);
		//glVertex3f(0, 1, 0);
		//glColor3f(0, 0, 1);
		//glVertex3f(0, 0, 0);
		//glVertex3f(0, 0, 1);

		//glColor3f(0.5, 0.5, 0.5);
		//glVertex3f(0, 0, 0);
		//glVertex3f(-1, 0, 0);
		//glVertex3f(0, 0, 0);
		//glVertex3f(0, -1, 0);
		//glVertex3f(0, 0, 0);
		//glVertex3f(0, 0, -1);

		//glEnd();
	}

	glPopMatrix();
}




void display_func(void)
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	// Draw the model's components using OpenGL/GLUT primitives.
	draw_objects();

	if(true == draw_control_list)
	{
		// Text drawing code originally from "GLUT Tutorial -- Bitmap Fonts and Orthogonal Projections" by A R Fernandes
		// http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, win_x, 0, win_y);
		glScalef(1, -1, 1); // Neat. :)
		glTranslatef(0, -win_y, 0); // Neat. :)
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glColor3f(control_list_colour.x, control_list_colour.y, control_list_colour.z);

		size_t break_size = 22;
		size_t start = 20;
		ostringstream oss;

		render_string(10, start + 1 * break_size, GLUT_BITMAP_HELVETICA_18, string("Keyboard controls:"));

        render_string(10, start + 3 * break_size, GLUT_BITMAP_HELVETICA_18, string("  q: Server pos.x++"));
		render_string(10, start + 4 * break_size, GLUT_BITMAP_HELVETICA_18, string("  w: Server pos.x--"));
		render_string(10, start + 5 * break_size, GLUT_BITMAP_HELVETICA_18, string("  a: Server pos.z++"));
		render_string(10, start + 6 * break_size, GLUT_BITMAP_HELVETICA_18, string("  s: Server pos.z--"));

		render_string(10, start + 8 * break_size, GLUT_BITMAP_HELVETICA_18, string("  e: Target pos.x++"));
		render_string(10, start + 9 * break_size, GLUT_BITMAP_HELVETICA_18, string("  r: Target pos.x--"));
		render_string(10, start + 10 * break_size, GLUT_BITMAP_HELVETICA_18, string("  d: Target pos.z++"));
		render_string(10, start + 11 * break_size, GLUT_BITMAP_HELVETICA_18, string("  f: Target pos.z--"));

								
        custom_math::vector_3 eye = main_camera.eye;
		custom_math::vector_3 eye_norm = eye;
		eye_norm.normalize();

		oss.clear();
		oss.str("");		
		oss << "Camera position: " << eye.x << ' ' << eye.y << ' ' << eye.z;
		render_string(10, win_y - 2*break_size, GLUT_BITMAP_HELVETICA_18, oss.str());

		oss.clear();
		oss.str("");
		oss << "Camera position (normalized): " << eye_norm.x << ' ' << eye_norm.y << ' ' << eye_norm.z;
		render_string(10, win_y - break_size, GLUT_BITMAP_HELVETICA_18, oss.str());

		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		// End text drawing code.
	}

	glFlush();
	glutSwapBuffers();
}

void keyboard_func(unsigned char key, int x, int y)
{
	switch(tolower(key))
	{
	case 'q':
	{
		server_pos.x += 1;
		get_path(paths[0], server_pos, server_vel, server_ang_vel, target_pos);
		break;
	}
	case 'w':
	{
		server_pos.x -= 1;
		get_path(paths[0], server_pos, server_vel, server_ang_vel, target_pos);
		break;
	}
	case 'a':
	{
		server_pos.z += 1;
		get_path(paths[0], server_pos, server_vel, server_ang_vel, target_pos);
		break;
	}
	case 's':
	{
		server_pos.z -= 1;

		if(server_pos.z < 0)
			server_pos.z = 0;

		get_path(paths[0], server_pos, server_vel, server_ang_vel, target_pos);
		break;
	}
	case 'e':
	{
		target_pos.x += 1;
		get_path(paths[0], server_pos, server_vel, server_ang_vel, target_pos);
		break;
	}
	case 'r':
	{
		target_pos.x -= 1;
		get_path(paths[0], server_pos, server_vel, server_ang_vel, target_pos);
		break;
	}
	case 'd':
	{
		target_pos.z += 1;

		if (target_pos.z > 0)
			target_pos.z = 0;

		get_path(paths[0], server_pos, server_vel, server_ang_vel, target_pos);
		break;
	}
	case 'f':
	{
		target_pos.z -= 1;
		get_path(paths[0], server_pos, server_vel, server_ang_vel, target_pos);
		break;
	}


	default:
		break;
	}
}

void mouse_func(int button, int state, int x, int y)
{
	if(GLUT_LEFT_BUTTON == button)
	{
		if(GLUT_DOWN == state)
			lmb_down = true;
		else
			lmb_down = false;
	}
	else if(GLUT_MIDDLE_BUTTON == button)
	{
		if(GLUT_DOWN == state)
			mmb_down = true;
		else
			mmb_down = false;
	}
	else if(GLUT_RIGHT_BUTTON == button)
	{
		if(GLUT_DOWN == state)
			rmb_down = true;
		else
			rmb_down = false;
	}
}

void motion_func(int x, int y)
{
	int prev_mouse_x = mouse_x;
	int prev_mouse_y = mouse_y;

	mouse_x = x;
	mouse_y = y;

	int mouse_delta_x = mouse_x - prev_mouse_x;
	int mouse_delta_y = prev_mouse_y - mouse_y;

	if(true == lmb_down && (0 != mouse_delta_x || 0 != mouse_delta_y))
	{
		main_camera.u -= static_cast<float>(mouse_delta_y)*u_spacer;
		main_camera.v += static_cast<float>(mouse_delta_x)*v_spacer;
	}
	else if(true == rmb_down && (0 != mouse_delta_y))
	{
		main_camera.w -= static_cast<float>(mouse_delta_y)*w_spacer;

		if(main_camera.w < 1.1f)
			main_camera.w = 1.1f;

	}

	main_camera.Set(); // Calculate new camera vectors.
}

void passive_motion_func(int x, int y)
{
	mouse_x = x;
	mouse_y = y;
}




