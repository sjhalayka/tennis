#include "main.h"

#pragma comment(lib, "freeglut.lib")



int main(int argc, char **argv)
{
	integrator_func_pointer = &proceed_symplectic4;

	get_targets(
		in_server_pos, in_server_vel, in_server_ang_vel, in_out_target_pos,
		out_server_vel,
		out_server_ang_vel,
		out_path);

	cout << out_path[out_path.size() - 1].x << " " << out_path[out_path.size() - 1].y << " " << out_path[out_path.size() - 1].z << endl;

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

	if (win_x < 1)
		win_x = 1;

	if (win_y < 1)
		win_y = 1;

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
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

	glClearColor(static_cast<float>(background_colour.x), static_cast<float>(background_colour.y), static_cast<float>(background_colour.z), 1.0f);
	glClearDepth(1.0f);

	main_camera.Set(0, 0, camera_w, camera_fov, win_x, win_y, camera_near, camera_far);
	main_camera.u += static_cast<float>(custom_math::pi) / 6.0f;
	main_camera.v += static_cast<float>(custom_math::pi) / 4.0f;
	main_camera.Set();
}

void reshape_func(int width, int height)
{
	win_x = width;
	win_y = height;

	if (win_x < 1)
		win_x = 1;

	if (win_y < 1)
		win_y = 1;

	glutSetWindow(win_id);
	glutReshapeWindow(win_x, win_y);
	glViewport(0, 0, win_x, win_y);

	main_camera.Set(main_camera.u, main_camera.v, main_camera.w, main_camera.fov, win_x, win_y, camera_near, camera_far);
}

// Text drawing code originally from "GLUT Tutorial -- Bitmap Fonts and Orthogonal Projections" by A R Fernandes
void render_string(int x, const int y, void *font, const string &text)
{
	for (size_t i = 0; i < text.length(); i++)
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

	glVertex3d(-half_court_width, 0, -half_court_length);
	glVertex3d(-half_court_width, 0, half_court_length);
	glVertex3d(half_court_width, 0, half_court_length);
	glVertex3d(half_court_width, 0, -half_court_length);

	glEnd();

	glEnable(GL_CULL_FACE);

	glLineWidth(2.0);

	glBegin(GL_LINES);

	glColor3f(1, 1, 1);

	glVertex3d(-half_court_width, 0, half_court_length);
	glVertex3d(half_court_width, 0, half_court_length);

	glVertex3d(-half_court_width, 0, -half_court_length);
	glVertex3d(half_court_width, 0, -half_court_length);

	glVertex3d(-half_court_width, 0, half_court_length);
	glVertex3d(-half_court_width, 0, -half_court_length);

	glVertex3d(half_court_width, 0, half_court_length);
	glVertex3d(half_court_width, 0, -half_court_length);

	glEnd();


	glPointSize(10.0f);

	glBegin(GL_POINTS);

	glColor3f(1, 1, 1);
	glVertex3d(in_server_pos.x, in_server_pos.y, in_server_pos.z);

	glColor3f(0, 0, 0);
	glVertex3d(in_out_target_pos.x, in_out_target_pos.y, in_out_target_pos.z);

	glEnd();







	glLineWidth(1.0f);

	glBegin(GL_LINES);

	glColor3f(0.5, 0.5, 0.5);
	glVertex3d(in_server_pos.x, in_server_pos.y, in_server_pos.z);
	glVertex3d(in_server_pos.x, 0, in_server_pos.z);

	glColor3f(1, 1, 1);
	glVertex3d(in_server_pos.x, in_server_pos.y, in_server_pos.z);
	glVertex3d(in_server_pos.x + out_server_vel.x, in_server_pos.y + out_server_vel.y, in_server_pos.z + out_server_vel.z);

	glVertex3d(in_server_pos.x, in_server_pos.y, in_server_pos.z);
	glVertex3d(in_server_pos.x + out_server_ang_vel.x, in_server_pos.y + out_server_ang_vel.y, in_server_pos.z + out_server_ang_vel.z);

	glEnd();


	//glColor3f(0.0, 0.0, 1.0);

	//for (size_t i = 0; i < all_paths.size(); i++)
	//{
	//	glBegin(GL_LINE_STRIP);

	//	for (size_t j = 0; j < all_paths[i].size(); j++)
	//	{
	//		glVertex3d(all_paths[i][j].x, all_paths[i][j].y, all_paths[i][j].z);
	//	}

	//	glEnd();
	//}


	glColor3f(0.0, 1.0, 0.0);

	glBegin(GL_LINE_STRIP);

	for (size_t i = 0; i < out_path.size(); i++)
	{
		glVertex3d(out_path[i].x, out_path[i].y, out_path[i].z);
	}

	glEnd();





	glDisable(GL_CULL_FACE);

	glBegin(GL_QUADS);

	glColor4f(0.0f, 0.5, 1.0f, 0.5f);

	glVertex3d(-half_court_width, 0, 0);
	glVertex3d(half_court_width, 0, 0);
	glVertex3d(half_court_width, net_height, 0);
	glVertex3d(-half_court_width, net_height, 0);

	glEnd();

	glEnable(GL_CULL_FACE);

	glPopMatrix();
}




void display_func(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw the model's components using OpenGL/GLUT primitives.
	draw_objects();

	if (true == draw_control_list)
	{
		// Text drawing code originally from "GLUT Tutorial -- Bitmap Fonts and Orthogonal Projections" by A R Fernandes
		// http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, win_x, 0, win_y);
		glScalef(1, -1, 1); // Neat. :)
		glTranslatef(0, -static_cast<float>(win_y), 0); // Neat. :)
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glColor3d(control_list_colour.x, control_list_colour.y, control_list_colour.z);

		size_t break_size = 22;
		size_t start = 20;
		ostringstream oss;

		render_string(10, start + 1 * break_size, GLUT_BITMAP_HELVETICA_18, string("Keyboard controls:"));

		render_string(10, start + 3 * break_size, GLUT_BITMAP_HELVETICA_18, string("  w: Server pos"));
		render_string(10, start + 4 * break_size, GLUT_BITMAP_HELVETICA_18, string("  a: Server pos"));
		render_string(10, start + 5 * break_size, GLUT_BITMAP_HELVETICA_18, string("  s: Server pos"));
		render_string(10, start + 6 * break_size, GLUT_BITMAP_HELVETICA_18, string("  d: Server pos"));

		render_string(10, start + 8 * break_size, GLUT_BITMAP_HELVETICA_18, string("  Click LMB: Target pos"));

		render_string(10, start + 10 * break_size, GLUT_BITMAP_HELVETICA_18, string("  1: Euler"));
		render_string(10, start + 11 * break_size, GLUT_BITMAP_HELVETICA_18, string("  2: Symplectic 2"));
		render_string(10, start + 12 * break_size, GLUT_BITMAP_HELVETICA_18, string("  3: Symplectic 4"));
		render_string(10, start + 13 * break_size, GLUT_BITMAP_HELVETICA_18, string("  4: RK2"));
		render_string(10, start + 14 * break_size, GLUT_BITMAP_HELVETICA_18, string("  5: RK4"));


		custom_math::vector_3 eye = main_camera.eye;
		custom_math::vector_3 eye_norm = eye;
		eye_norm.normalize();

		oss.clear();
		oss.str("");
		oss << "Integrator: ";

		if (integrator_func_pointer == proceed_Euler)
		{
			oss << "Euler";
		}
		else if (integrator_func_pointer == proceed_symplectic2)
		{
			oss << "Symplectic 2";
		}
		else if (integrator_func_pointer == proceed_symplectic4)
		{
			oss << "Symplectic 4";
		}
		else if (integrator_func_pointer == proceed_RK2)
		{
			oss << "RK2";
		}
		else if (integrator_func_pointer == proceed_RK4)
		{
			oss << "RK4";
		}



		render_string(10, win_y - 3 * break_size, GLUT_BITMAP_HELVETICA_18, oss.str());


		oss.clear();
		oss.str("");
		oss << "Camera position: " << eye.x << ' ' << eye.y << ' ' << eye.z;
		render_string(10, win_y - 2 * break_size, GLUT_BITMAP_HELVETICA_18, oss.str());

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
	switch (tolower(key))
	{
	case 'd':
	{
		in_server_pos.x += 1;
		break;
	}
	case 'a':
	{
		in_server_pos.x -= 1;
		break;
	}
	case 's':
	{
		in_server_pos.z += 1;
		break;
	}
	case 'w':
	{
		in_server_pos.z -= 1;

		if (in_server_pos.z < 0)
			in_server_pos.z = 0;

		break;
	}
	case 'o':
	{
		double len = in_server_ang_vel.length();

		in_server_ang_vel.normalize();
		in_server_ang_vel *= len + 1;

		break;
	}
	case 'p':
	{
		double len = in_server_ang_vel.length();

		in_server_ang_vel.normalize();

		if (len > 1)
			in_server_ang_vel *= len - 1;

		break;
	}
	case '1':
	{
		integrator_func_pointer = &proceed_Euler;
		break;
	}
	case '2':
	{
		integrator_func_pointer = &proceed_symplectic2;
		break;
	}

	case '3':
	{
		integrator_func_pointer = &proceed_symplectic4;
		break;
	}
	case '4':
	{
		integrator_func_pointer = &proceed_RK2;
		break;
	}
	case '5':
	{
		integrator_func_pointer = &proceed_RK4;
		break;
	}
	default:
		break;
	}

	get_targets(
		in_server_pos, in_server_vel, in_server_ang_vel, in_out_target_pos,
		out_server_vel,
		out_server_ang_vel,
		out_path);
}

void mouse_func(int button, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == button)
	{
		if (GLUT_DOWN == state)
			lmb_down = true;
		else
			lmb_down = false;
	}
	else if (GLUT_MIDDLE_BUTTON == button)
	{
		if (GLUT_DOWN == state)
			mmb_down = true;
		else
			mmb_down = false;
	}
	else if (GLUT_RIGHT_BUTTON == button)
	{
		if (GLUT_DOWN == state)
			rmb_down = true;
		else
			rmb_down = false;
	}


	if (lmb_down)
	{
		// Intersect screen ray with tennis court plane
		custom_math::vector_3 p_ij = main_camera.Get_Screen_Ray(x, y, win_x, win_y);

		custom_math::vector_3 N(0, 1, 0);
		custom_math::vector_3 D(p_ij.x, p_ij.y, p_ij.z);
		D.normalize();
		custom_math::vector_3 O(main_camera.eye.x, main_camera.eye.y, main_camera.eye.z);
		double u = -(N.dot(O) + 0.0) / N.dot(D);
		custom_math::vector_3 P = O + D * u;

		in_out_target_pos = P;

		if (false == pro_mode)
		{
			if (in_out_target_pos.x < -half_court_width)
				in_out_target_pos.x = -half_court_width;
			else if (in_out_target_pos.x > half_court_width)
				in_out_target_pos.x = half_court_width;

			if (in_out_target_pos.z < -half_court_length)
				in_out_target_pos.z = -half_court_length;
			else if (in_out_target_pos.z > 0)
				in_out_target_pos.z = 0;
		}

		get_targets(
			in_server_pos, in_server_vel, in_server_ang_vel, in_out_target_pos,
			out_server_vel,
			out_server_ang_vel,
			out_path);
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

	if (true == lmb_down && (0 != mouse_delta_x || 0 != mouse_delta_y))
	{
		main_camera.u -= static_cast<float>(mouse_delta_y)*u_spacer;
		main_camera.v += static_cast<float>(mouse_delta_x)*v_spacer;
	}
	else if (true == rmb_down && (0 != mouse_delta_y))
	{
		main_camera.w -= static_cast<float>(mouse_delta_y)*w_spacer;

		if (main_camera.w < 1.1f)
			main_camera.w = 1.1f;

	}

	main_camera.Set(); // Calculate new camera vectors.
}

void passive_motion_func(int x, int y)
{
	mouse_x = x;
	mouse_y = y;
}




