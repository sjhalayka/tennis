

#ifndef custom_math_h
#define custom_math_h

#include <algorithm>
using std::sort;

#include <limits>
using std::numeric_limits;

#include <vector>
using std::vector;

#include <set>
using std::set;

#include <map>
using std::map;


#include <iostream>
using std::cout;
using std::endl;

#include <cmath>
#include <cstdlib>
#include <GL/glut.h>       //GLUT Library


namespace custom_math
{
	class vector_3;
	class line_segment_3;
	class circle_3;

	class indexed_triangle;
	class sorted_indexed_triangle;
	class sorted_indexed_edge;
	class indexed_ngon;

	class indexed_curved_triangle;

	class vector_4;
	class circle_4;
	class line_segment_4;

	const double pi = 3.14159265358979323846;
	const double pi_half = pi/2;
	const double pi_2 = 2*pi;
	const double epsilon = 1e-6;

	double d(const double &a, const double &b);
	double d_3(const vector_3 &a, const vector_3 &b);
	double d_3_sq(const vector_3 &a, const vector_3 &b);
	double d_4(const vector_4 &a, const vector_4 &b);
};

class custom_math::vector_3
{
public:
	double x, y, z;

	vector_3(const double &src_x = 0, const double &src_y = 0, const double &src_z = 0);
	bool operator==(const vector_3 &rhs);
	bool operator!=(const vector_3 &rhs);
	void zero(void);
	void rotate_x(const double &radians);
	void rotate_y(const double &radians);
    void rotate_z(const double &radians);
    vector_3 operator+(const vector_3 &rhs);
	vector_3 operator-(const vector_3 &rhs);
	vector_3 operator*(const vector_3 &rhs);
	vector_3 operator*(const double &rhs);
	vector_3 operator/(const double &rhs);
	vector_3 &operator=(const vector_3 &rhs);
	vector_3 &operator+=(const vector_3 &rhs);
	vector_3 &operator*=(const vector_3 &rhs);
	vector_3 &operator*=(const double &rhs);

	vector_3 &operator/=(const double &rhs);

	vector_3 operator-(void);
	double length(void) const;
	vector_3 &normalize(void);
	double dot(const vector_3 &rhs) const;
	double self_dot(void) const;
	vector_3 cross(const vector_3 &rhs) const;
};


class custom_math::vector_4
{
public:
	double x, y, z, w;

	vector_4(const double &src_x = 0, const double &src_y = 0, const double &src_z = 0, const double &src_w = 0);
	void zero(void);
	vector_4 operator+(const vector_4 &rhs);
	vector_4 operator-(const vector_4 &rhs);
	vector_4 operator*(const vector_4 &rhs);
	vector_4 operator*(const double &rhs);
	vector_4 operator/(const double &rhs);
	vector_4 &operator=(const vector_4 &rhs);
	vector_4 &operator+=(const vector_4 &rhs);
	vector_4 &operator*=(const vector_4 &rhs);
	vector_4 &operator*=(const double &rhs);
	vector_4 operator-(void);
	double length(void) const;
	vector_4 &normalize(void);
	double dot(const vector_4 &rhs) const;
	double self_dot(void) const;
};


class custom_math::line_segment_3
{
public:
	vector_3 start, end;

	double length(void)
	{
		return d_3(start, end);
	}

	bool operator<(line_segment_3 &rhs)
	{
		return length() < rhs.length();
	}
};

class custom_math::line_segment_4
{
public:
	vector_4 start, end;

	double length(void)
	{
		return d_4(start, end);
	}

	bool operator<(line_segment_4 &rhs)
	{
		return length() < rhs.length();
	}
};


class custom_math::circle_3
{
public:
	vector_3 U, V;

	void get_vertices(size_t num_steps, double radius, vector<vector_3> &vertices)
	{
		vertices.clear();

		for(size_t step = 0; step < num_steps; step++)
		{
			const double circumference_arc = 2*pi/static_cast<double>(num_steps);
			double t = step*circumference_arc;
			static vector_3 v;
			v = U*cos(t) + V*sin(t);

			double vlen = v.length();

			if(vlen > 1.0 + epsilon || vlen < 1.0 - epsilon)
				cout << "circle_3 parameterization error: " << vlen << endl;

			v = v*radius;

			vertices.push_back(v);
		}
	}

	void make_Vy_zero(void)
	{
		if(U.y > 0.0 - epsilon && U.y < 0.0 + epsilon)
		{
			static vector_3 temp_v;
			temp_v = U;
			U = V;
			V = temp_v;
		}
	}

	// Note: These reparameterization functions work with vectors of arbitrary dimension.
	// e.g., No 3D cross product operations were used.
	void reparameterize_U(void)
	{
		// TODO: Handle special case where U is close to 0 or 1 (rectangle goes to zero area).
		// TODO: Make north pole the U vector?

		// Make a rectangle.
		static vector_3 U1, U2, U3, U4;

		U.normalize();

		U1 = U;

		// Invert all, and then revert y back to original sign.
		U2 = -U;
		U2.y = -U2.y;

		// Invert all.
		U3 = -U;

		// Invert y.
		U4 = U;
		U4.y = -U4.y;

		U = ((U1 - U3) + (U4 - U2)).normalize();
		V = ((U1 - U3) + (U2 - U4)).normalize();

		make_Vy_zero();

		if(U.y < 0)
			U.y = -U.y;
	}

	void reparameterize_UV(void)
	{
		// Make a rectangle.
		static vector_3 U1, U2, U3, U4;

		U.normalize();
		V.normalize();

		// Handle special case where the vertices are antipodal.
		double dot = U.dot(V);

		if(dot < -1.0 + epsilon)
		{
			cout << "circle_3 -- handling antipodal reparameterization." << endl;
			V = vector_3(0, 1, 0);
		}

		U1 = U*cos(0.0) + V*sin(0.0);
		U2 = U*cos(0.5*pi) + V*sin(0.5*pi);
		U3 = U*cos(pi) + V*sin(pi);
		U4 = U*cos(1.5*pi) + V*sin(1.5*pi);

		U = ((U1 - U3) + (U4 - U2)).normalize();
		V = ((U1 - U3) + (U2 - U4)).normalize();

		make_Vy_zero();
	}
};


class custom_math::circle_4
{
public:
	vector_4 U, V;

	void get_vertices(size_t num_steps, double radius, vector<vector_4> &vertices)
	{
		vertices.clear();

		for(size_t step = 0; step < num_steps; step++)
		{
			const double circumference_arc = 2*pi/static_cast<double>(num_steps);
			double t = step*circumference_arc;
			static vector_4 v;
			v = U*cos(t) + V*sin(t);

			double vlen = v.length();

			if(vlen > 1.0 + epsilon || vlen < 1.0 - epsilon)
				cout << "circle_4 parameterization error: " << vlen << endl;

			v = v*radius;

			vertices.push_back(v);
		}
	}

	void make_Vy_zero(void)
	{
		if(U.y > 0.0 - epsilon && U.y < 0.0 + epsilon)
		{
			static vector_4 temp_v;
			temp_v = U;
			U = V;
			V = temp_v;
		}
	}

	// Note: These reparameterization functions work with vectors of arbitrary dimension.
	// e.g., No 3D cross product operations were used.
	void reparameterize_U(void)
	{
		// TODO: Handle special case where U is close to 0 or 1 (rectangle goes to zero area).
		// TODO: Make north pole the U vector?

		// Make a rectangle.
		static vector_4 U1, U2, U3, U4;

		U.normalize();

		U1 = U;

		// Invert all, and then revert y back to original sign.
		U2 = -U;
		U2.y = -U2.y;

		// Invert all.
		U3 = -U;

		// Invert y.
		U4 = U;
		U4.y = -U4.y;

		U = ((U1 - U3) + (U4 - U2)).normalize();
		V = ((U1 - U3) + (U2 - U4)).normalize();

		make_Vy_zero();

		if(U.y < 0)
			U.y = -U.y;
	}

	void reparameterize_UV(void)
	{
		// Make a rectangle.
		static vector_4 U1, U2, U3, U4;

		U.normalize();
		V.normalize();

		// Handle special case where the vertices are antipodal.
		double dot = U.dot(V);

		if(dot < -1.0 + epsilon)
		{
			cout << "circle_4 -- handling antipodal reparameterization." << endl;
			V = vector_4(0, 1, 0, 0);
		}

		U1 = U*cos(0.0) + V*sin(0.0);
		U2 = U*cos(0.5*pi) + V*sin(0.5*pi);
		U3 = U*cos(pi) + V*sin(pi);
		U4 = U*cos(1.5*pi) + V*sin(1.5*pi);

		U = ((U1 - U3) + (U4 - U2)).normalize();
		V = ((U1 - U3) + (U2 - U4)).normalize();

		make_Vy_zero();
	}
};



class custom_math::sorted_indexed_edge
{
public:
	sorted_indexed_edge(const size_t src_v0, const size_t src_v1)
	{
		if(src_v0 < src_v1)
		{
			v0 = src_v0;
			v1 = src_v1;
		}
		else
		{
			v1 = src_v0;
			v0 = src_v1;
		}
	}

	bool operator==(const sorted_indexed_edge &rhs) const
	{
		// vertices should be pre-sorted
		if(v0 == rhs.v0 && v1 == rhs.v1)
			return true;

		return false;
	}

	bool operator<(const sorted_indexed_edge &rhs) const
	{
		// vertices should be pre-sorted
		if(v0 < rhs.v0)
			return true;
		else if(v0 > rhs.v0)
			return false;
		// else first components are equal

		if(v1 < rhs.v1)
			return true;

		return false;
	}

	size_t v0, v1;
};

class custom_math::indexed_triangle
{
public:
	size_t i0, i1, i2; // vertices
};

class custom_math::indexed_curved_triangle
{
protected:
	vector<vector_3> local_vertices; // first three vertices shoul be equal to vertices[i0], [i1], [i2], respectively.
	set<sorted_indexed_edge> local_curved_outline_edges;
	vector<indexed_triangle> local_triangles;

	vector<vector<float> > local_materials;
	vector<indexed_triangle> mat_blend_ops;

public:
	size_t seed_i0, seed_i1, seed_i2; // vertices indices from main mesh, for helping get seed mat data
	vector_3 circumcentre_normal;

	void init_geometry(const size_t src_i0, const vector_3 &v0, 
					const size_t src_i1, const vector_3 &v1, 
					const size_t src_i2, const vector_3 &v2,
					const size_t num_subdivisions)
	{
		local_vertices.clear();
		local_curved_outline_edges.clear();
		local_triangles.clear();
		local_materials.clear();
		mat_blend_ops.clear();


		circumcentre_normal = v0;
		circumcentre_normal = circumcentre_normal + v1;
		circumcentre_normal = circumcentre_normal + v2;
		circumcentre_normal = circumcentre_normal/3.0;
		circumcentre_normal.normalize();


		seed_i0 = src_i0;
		seed_i1 = src_i1;
		seed_i2 = src_i2;

		local_vertices.push_back(v0);
		local_vertices.push_back(v1);
		local_vertices.push_back(v2);

		indexed_triangle tri;
		tri.i0 = 0;
		tri.i1 = 1;
		tri.i2 = 2;
		local_triangles.push_back(tri);

		local_curved_outline_edges.insert(sorted_indexed_edge(0, 1));
		local_curved_outline_edges.insert(sorted_indexed_edge(1, 2));
		local_curved_outline_edges.insert(sorted_indexed_edge(2, 0));


		for(size_t i = 0; i < num_subdivisions; i++)
		{
			set<sorted_indexed_edge> distinct_edges;
			set<sorted_indexed_edge> new_local_curved_outline_edges;
			vector<indexed_triangle> new_local_triangles;

			for(size_t j = 0; j < local_triangles.size(); j++)
			{
				distinct_edges.insert(sorted_indexed_edge(local_triangles[j].i0, local_triangles[j].i1));
				distinct_edges.insert(sorted_indexed_edge(local_triangles[j].i1, local_triangles[j].i2));
				distinct_edges.insert(sorted_indexed_edge(local_triangles[j].i2, local_triangles[j].i0));
			}

			map<sorted_indexed_edge, size_t> edge_to_new_vertex_map;

			size_t new_local_vertex_index = local_vertices.size();

			for(set<sorted_indexed_edge>::const_iterator ci = distinct_edges.begin(); ci != distinct_edges.end(); ci++)
			{
				edge_to_new_vertex_map[*ci] = new_local_vertex_index;

				vector_3 mid_point = local_vertices[ci->v0] + local_vertices[ci->v1];
				mid_point *= 0.5;
				mid_point.normalize(); // Project onto 2-sphere
				local_vertices.push_back(mid_point);

				indexed_triangle blend_op;
				blend_op.i0 = new_local_vertex_index;
				blend_op.i1 = ci->v0;
				blend_op.i2 = ci->v1;
				mat_blend_ops.push_back(blend_op);

				// If edge is part of the curved triangle edge, split it into two subedges
				if(local_curved_outline_edges.end() != local_curved_outline_edges.find(*ci))
				{
					new_local_curved_outline_edges.insert(sorted_indexed_edge(ci->v0, new_local_vertex_index));
					new_local_curved_outline_edges.insert(sorted_indexed_edge(new_local_vertex_index, ci->v1));
				}

				new_local_vertex_index++;
			}

			new_local_curved_outline_edges.swap(local_curved_outline_edges);

			for(size_t j = 0; j < local_triangles.size(); j++)
			{
				sorted_indexed_edge edge0(local_triangles[j].i0, local_triangles[j].i1);
				sorted_indexed_edge edge1(local_triangles[j].i1, local_triangles[j].i2);
				sorted_indexed_edge edge2(local_triangles[j].i2, local_triangles[j].i0);

				size_t new_vertex0 = edge_to_new_vertex_map[edge0];
				size_t new_vertex1 = edge_to_new_vertex_map[edge1];
				size_t new_vertex2 = edge_to_new_vertex_map[edge2];

				indexed_triangle tri;

				tri.i0 = local_triangles[j].i0;
				tri.i1 = new_vertex0;
				tri.i2 = new_vertex2;
				new_local_triangles.push_back(tri);

				tri.i0 = new_vertex0;
				tri.i1 = local_triangles[j].i1;
				tri.i2 = new_vertex1;
				new_local_triangles.push_back(tri);

				tri.i0 = new_vertex0;
				tri.i1 = new_vertex1;
				tri.i2 = new_vertex2;
				new_local_triangles.push_back(tri);

				tri.i0 = new_vertex2;
				tri.i1 = new_vertex1;
				tri.i2 = local_triangles[j].i2;
				new_local_triangles.push_back(tri);


				// add 4 new triangles to new_local_triangles

			}

			new_local_triangles.swap(local_triangles);

			// for each triangle
			// - for each edge in CCW order, get corresponding new vertices
			// - use these new vertices and existing vertices to make 4 subtriangles
		}

		vector<float> empty_mat(4, 0.0f);
		local_materials.resize(local_vertices.size(), empty_mat);
	}

	// Input are materials for local_vertices[0], [1], [2]
	void init_mats(const vector<float> &mat0, const vector<float> &mat1, const vector<float> &mat2)
	{
		if(4 != mat0.size() || 4 != mat1.size() || 4 != mat2.size())
			return;

		local_materials[0] = mat0;
		local_materials[1] = mat1;
		local_materials[2] = mat2;

		for(size_t i = 0; i < mat_blend_ops.size(); i++)
		{
			local_materials[mat_blend_ops[i].i0][0] = 0.5f*(local_materials[mat_blend_ops[i].i1][0] + local_materials[mat_blend_ops[i].i2][0]);
			local_materials[mat_blend_ops[i].i0][1] = 0.5f*(local_materials[mat_blend_ops[i].i1][1] + local_materials[mat_blend_ops[i].i2][1]);
			local_materials[mat_blend_ops[i].i0][2] = 0.5f*(local_materials[mat_blend_ops[i].i1][2] + local_materials[mat_blend_ops[i].i2][2]);
			local_materials[mat_blend_ops[i].i0][3] = 0.5f*(local_materials[mat_blend_ops[i].i1][3] + local_materials[mat_blend_ops[i].i2][3]);
		}
	}

	void draw_mat4(void)
	{
		for(size_t i = 0; i < local_triangles.size(); i++)
		{
			glMaterialfv(GL_FRONT, GL_DIFFUSE, &local_materials[local_triangles[i].i0][0]);
			glNormal3f(local_vertices[local_triangles[i].i0].x, local_vertices[local_triangles[i].i0].y, local_vertices[local_triangles[i].i0].z);
			glVertex3f(local_vertices[local_triangles[i].i0].x, local_vertices[local_triangles[i].i0].y, local_vertices[local_triangles[i].i0].z);

			glMaterialfv(GL_FRONT, GL_DIFFUSE, &local_materials[local_triangles[i].i1][0]);
			glNormal3f(local_vertices[local_triangles[i].i1].x, local_vertices[local_triangles[i].i1].y, local_vertices[local_triangles[i].i1].z);
			glVertex3f(local_vertices[local_triangles[i].i1].x, local_vertices[local_triangles[i].i1].y, local_vertices[local_triangles[i].i1].z);

			glMaterialfv(GL_FRONT, GL_DIFFUSE, &local_materials[local_triangles[i].i2][0]);
			glNormal3f(local_vertices[local_triangles[i].i2].x, local_vertices[local_triangles[i].i2].y, local_vertices[local_triangles[i].i2].z);
			glVertex3f(local_vertices[local_triangles[i].i2].x, local_vertices[local_triangles[i].i2].y, local_vertices[local_triangles[i].i2].z);
		}
	}

	void draw_colour3(void)
	{
		for(size_t i = 0; i < local_triangles.size(); i++)
		{
			glColor3f(local_materials[local_triangles[i].i0][0], local_materials[local_triangles[i].i0][1], local_materials[local_triangles[i].i0][2]);
			glNormal3f(local_vertices[local_triangles[i].i0].x, local_vertices[local_triangles[i].i0].y, local_vertices[local_triangles[i].i0].z);
			glVertex3f(local_vertices[local_triangles[i].i0].x, local_vertices[local_triangles[i].i0].y, local_vertices[local_triangles[i].i0].z);

			glColor3f(local_materials[local_triangles[i].i1][0], local_materials[local_triangles[i].i1][1], local_materials[local_triangles[i].i1][2]);
			glNormal3f(local_vertices[local_triangles[i].i1].x, local_vertices[local_triangles[i].i1].y, local_vertices[local_triangles[i].i1].z);
			glVertex3f(local_vertices[local_triangles[i].i1].x, local_vertices[local_triangles[i].i1].y, local_vertices[local_triangles[i].i1].z);

			glColor3f(local_materials[local_triangles[i].i2][0], local_materials[local_triangles[i].i2][1], local_materials[local_triangles[i].i2][2]);
			glNormal3f(local_vertices[local_triangles[i].i2].x, local_vertices[local_triangles[i].i2].y, local_vertices[local_triangles[i].i2].z);
			glVertex3f(local_vertices[local_triangles[i].i2].x, local_vertices[local_triangles[i].i2].y, local_vertices[local_triangles[i].i2].z);
		}
	}

	void draw_outline(void)
	{
		for(set<sorted_indexed_edge>::const_iterator ci = local_curved_outline_edges.begin(); ci != local_curved_outline_edges.end(); ci++)
		{
			glVertex3f(local_vertices[ci->v0].x, local_vertices[ci->v0].y, local_vertices[ci->v0].z);
			glVertex3f(local_vertices[ci->v1].x, local_vertices[ci->v1].y, local_vertices[ci->v1].z);
		}
	}

};



class custom_math::sorted_indexed_triangle
{
public:
	sorted_indexed_triangle(const size_t src_i0, const size_t src_i1, const size_t src_i2)
	{
		i0 = src_i0;
		i1 = src_i1;
		i2 = src_i2;
	}

	sorted_indexed_triangle(void)
	{

	}

	bool operator==(const sorted_indexed_triangle &rhs) const
	{
		vector<size_t> indices;
		indices.push_back(i0);
		indices.push_back(i1);
		indices.push_back(i2);
		sort(indices.begin(), indices.end());

		vector<size_t> rhs_indices;
		rhs_indices.push_back(rhs.i0);
		rhs_indices.push_back(rhs.i1);
		rhs_indices.push_back(rhs.i2);
		sort(rhs_indices.begin(), rhs_indices.end());

		// vertices should be pre-sorted
		if(indices[0] == rhs_indices[0] && indices[1] == rhs_indices[1] && indices[2] == rhs_indices[2])
			return true;

		return false;
	}

	bool operator<(const sorted_indexed_triangle &rhs) const
	{
		vector<size_t> indices;
		indices.push_back(i0);
		indices.push_back(i1);
		indices.push_back(i2);
		sort(indices.begin(), indices.end());

		vector<size_t> rhs_indices;
		rhs_indices.push_back(rhs.i0);
		rhs_indices.push_back(rhs.i1);
		rhs_indices.push_back(rhs.i2);
		sort(rhs_indices.begin(), rhs_indices.end());

		// vertices should be pre-sorted
		if(indices[0] < rhs_indices[0])
			return true;
		else if(indices[0] > rhs_indices[0])
			return false;
		// else first components are equal

		if(indices[1] < rhs_indices[1])
			return true;
		else if(indices[1] > rhs_indices[1])
			return false;
		// else second components are equal

		if(indices[2] < rhs_indices[2])
			return true;

		return false;
	}

	size_t i0, i1, i2;
};




class custom_math::indexed_ngon
{
public:
	vector<size_t> v; // vertices
};

#endif

