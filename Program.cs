using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace tennis_cs
{
    class d : IComparable<d>
    {
        public int index;
        public double val;

        public d()
        {
            index = 0;
            val = 0;
        }

        public int CompareTo(d other)
        {
            return other.val.CompareTo(this.val);
        }

        public static bool operator <(d d1, d d2)
        {
            return d1.val < d2.val;
        }

        public static bool operator >(d d1, d d2)
        {
            return d1.val > d2.val;
        }
    }

    class vector_3
    {
        public double x, y, z;

        public vector_3(double src_x, double src_y, double src_z)
        {
            x = src_x;
            y = src_y;
            z = src_z;
        }


        public void rotate_y(double radians)
        {
            double t_x = x;

            x = t_x * Math.Cos(radians) + z * -Math.Sin(radians);
            z = t_x * Math.Sin(radians) + z * Math.Cos(radians);
        }

        public double self_dot()
        {
            return x * x + y * y + z * z;
        }

        public double length()
        {
            return Math.Sqrt(self_dot());
        }

        public static vector_3 operator +(vector_3 lhs, vector_3 rhs)
        {
            return new vector_3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
        }

        public static vector_3 operator -(vector_3 lhs, vector_3 rhs)
        {
            return new vector_3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
        }

        public static vector_3 operator *(vector_3 lhs, double rhs)
        {
            return new vector_3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
        }

        public static vector_3 operator /(vector_3 lhs, double rhs)
        {
            return new vector_3(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
        }

        public vector_3 cross(vector_3 rhs)
        {
            vector_3 c = new vector_3(0, 0, 0);
            c.x = y * rhs.z - rhs.y * z;
            c.y = z * rhs.x - rhs.z * x;
            c.z = x * rhs.y - rhs.x * y;

            return c;
        }

        public double dot(vector_3 rhs)
        {
            return x * rhs.x + y * rhs.y + z * rhs.z;
        }

        public vector_3 normalize()
        {
            double len = length();

            if (len != 1)
            {
                x /= len;
                y /= len;
                z /= len;
            }

            return this;
        }
    }

    class tennis_parameters
    {
        public tennis_parameters()
        {
            court_width = 10.9728; // 36 feet
            half_court_width = court_width / 2.0;
            court_length = 22.86; // 75 feet
            half_court_length = court_length / 2.0;
            net_height = 0.9144; // 3 feet

            num_vectors = 10;
            num_hone_iterations = 1;
            num_length_adjustment_iterations = 20;

            in_server_pos = new vector_3(3, 1, 3);
            in_server_vel = new vector_3(-3, 1, -5);
            in_server_ang_vel = new vector_3(10, 5, 0);
            in_target_pos = new vector_3(5, 0, -5);

            out_server_vel_1 = new vector_3(0, 0, 0);
            out_server_ang_vel_1 = new vector_3(0, 0, 0);
            out_server_vel_2 = new vector_3(0, 0, 0);
            out_server_ang_vel_2 = new vector_3(0, 0, 0);

            out_p_1 = new List<vector_3>();
            out_p_2 = new List<vector_3>();
        }

        public double court_width;
        public double half_court_width;
        public double court_length;
        public double half_court_length;

        public double net_height;

        public int num_vectors;
        public int num_hone_iterations;
        public int num_length_adjustment_iterations;

        public vector_3 in_server_pos;
        public vector_3 in_server_vel;
        public vector_3 in_server_ang_vel;
        public vector_3 in_target_pos;

        public vector_3 out_server_vel_1;
        public vector_3 out_server_ang_vel_1;
        public vector_3 out_server_vel_2;
        public vector_3 out_server_ang_vel_2;

        public List<vector_3> out_p_1;
        public List<vector_3> out_p_2;
    }

    class tennis_functions
    {
        vector_3 lerp(vector_3 A, vector_3 B, double t)
        {
            vector_3 a = A * t;
            vector_3 b = B * (1.0 - t);

            return a + b;
        }

        vector_3 acceleration(vector_3 pos, vector_3 vel, vector_3 ang_vel)
        {
            // Gravitation, in metres per second, per second
            vector_3 grav_accel = new vector_3(0, -9.81, 0);

            // Magnus effect, in metres per second, per second
            // angular velocity x velocity * 0.5*fluid_density*drag_coeff*ball_cross_section_area / ball_mass
            vector_3 magnus_accel = ang_vel.cross(vel) * 0.001;

            // Wind and drag, in metres per second, per second
            vector_3 wind_vel = new vector_3(5, 0, 0); // Set this to 0, 0, 0 for plain drag
            vector_3 drag_vel = wind_vel - vel;
            double drag_speed = drag_vel.length();

            // velocity * (velocity length) * 0.5*fluid_density*drag_coeff*ball_cross_section_area / ball mass
            vector_3 drag_accel = drag_vel * drag_speed * 0.001;

            return grav_accel + magnus_accel + drag_accel;
        }

        void proceed_rk4(ref vector_3 pos, ref vector_3 vel, vector_3 ang_vel)
        {
            const double one_sixth = 1.0 / 6.0;
            const double dt = 0.0001;

            vector_3 k1_velocity = vel;
            vector_3 k1_acceleration = acceleration(pos, k1_velocity, ang_vel);
            vector_3 k2_velocity = vel + k1_acceleration * dt * 0.5;
            vector_3 k2_acceleration = acceleration(pos + k1_velocity * dt * 0.5, k2_velocity, ang_vel);
            vector_3 k3_velocity = vel + k2_acceleration * dt * 0.5;
            vector_3 k3_acceleration = acceleration(pos + k2_velocity * dt * 0.5, k3_velocity, ang_vel);
            vector_3 k4_velocity = vel + k3_acceleration * dt;
            vector_3 k4_acceleration = acceleration(pos + k3_velocity * dt, k4_velocity, ang_vel);

            vel += (k1_acceleration + (k2_acceleration + k3_acceleration) * 2.0 + k4_acceleration) * one_sixth * dt;
            pos += (k1_velocity + (k2_velocity + k3_velocity) * 2.0 + k4_velocity) * one_sixth * dt;
        }

        int get_path(
            ref List<vector_3> p,
            vector_3 server_position,
            vector_3 server_velocity,
            vector_3 server_angular_velocity,
            vector_3 target_position,
            double net_height,
            double half_court_width)
        {
            p.Clear();

            p.Add(server_position);

            vector_3 last_pos = server_position;
            vector_3 last_vel = server_velocity;

            while (true)
            {
                vector_3 curr_pos = last_pos;
                vector_3 curr_vel = last_vel;
                proceed_rk4(ref curr_pos, ref curr_vel, server_angular_velocity);

                p.Add(curr_pos);

                // if collides with surface
                if (curr_pos.y <= 0)
                    break;

                bool is_near_net = (curr_pos.z < 0 && last_pos.z >= 0);

                // if collides with net, reflect vector
                if (is_near_net &&
                    curr_pos.y >= 0 && curr_pos.y <= net_height &&
                    curr_pos.x >= -half_court_width && curr_pos.x <= half_court_width)
                {
                    vector_3 N = new vector_3(0, 0, 1);
                    curr_vel = N * curr_vel.dot(N) * 2.0 - curr_vel;

                    curr_vel.x = -curr_vel.x;
                    curr_vel.y = -curr_vel.y;
                    curr_vel.z = -curr_vel.z;
                }

                last_pos = curr_pos;
                last_vel = curr_vel;
            }

            return 0;
        }


        public int hone_path(
            ref List<vector_3> p,
            vector_3 server_position,
            ref vector_3 server_velocity,
            ref vector_3 server_angular_velocity,
            vector_3 target_position,
            int num_length_adjustment_iterations,
            double net_height,
            double half_court_width)
        {

            get_path(
                ref p,
                server_position,
                server_velocity,
                server_angular_velocity,
                target_position,
                net_height,
                half_court_width);

            for (int i = 0; i < num_length_adjustment_iterations; i++)
            {
                // adjust velocity length to get closer to the target position
                vector_3 begin_pos = p[0];
                vector_3 end_pos = p[p.Count() - 1];
                vector_3 diff_a = end_pos - begin_pos;
                vector_3 diff_b = target_position - begin_pos;
                double len_a = diff_a.length();
                double len_b = diff_b.length();
                double slope = len_a / len_b;

                server_velocity = server_velocity / slope;

                get_path(
                    ref p,
                    server_position,
                    server_velocity,
                    server_angular_velocity,
                    target_position,
                    net_height,
                    half_court_width);
            }

            // rotate vectors on y axis to get closer to the target position
            vector_3 end_position = p[p.Count() - 1];
            vector_3 v1 = server_position - target_position;
            vector_3 v2 = server_position - end_position;
            v1.normalize();
            v2.normalize();

            double angle = Math.Acos(v1.dot(v2));

            server_velocity.rotate_y(-angle);
            server_angular_velocity.rotate_y(-angle);

            get_path(
                ref p,
                server_position,
                server_velocity,
                server_angular_velocity,
                target_position,
                net_height,
                half_court_width);

            return 0;
        }

        public void get_targets(
            vector_3 in_server_pos,
            vector_3 in_server_vel,
            vector_3 in_server_ang_vel,
            vector_3 in_target_pos,
            ref vector_3 out_server_vel_1,
            ref vector_3 out_server_ang_vel_1,
            ref vector_3 out_server_vel_2,
            ref vector_3 out_server_ang_vel_2,
            ref List<vector_3> p_1,
            ref List<vector_3> p_2,
            int num_vectors,
            int num_hone_iterations,
            int num_length_adjustment_iterations,
            double net_height,
            double half_court_width)
        {
            List<List<vector_3>> paths = new List<List<vector_3>>();
            List<vector_3> path = new List<vector_3>();

            for (int i = 0; i < num_vectors; i++)
                paths.Add(path);

            in_server_vel = in_target_pos - in_server_pos;
            double server_vel_len = in_server_vel.length();
            vector_3 up = new vector_3(0, server_vel_len, 0);
            double step_size = 1.0 / num_vectors;

            List<vector_3> server_vels = new List<vector_3>();
            List<vector_3> server_ang_vels = new List<vector_3>();

            for (int i = 0; i < num_vectors; i++)
            {
                in_server_vel = lerp(in_target_pos - in_server_pos, up, i * step_size);
                in_server_vel.normalize();
                in_server_vel *= server_vel_len;

                server_vels.Add(in_server_vel);
                server_ang_vels.Add(in_server_ang_vel);

                List<vector_3> p = paths[i];

                get_path(
                     ref p,
                      in_server_pos,
                      in_server_vel,
                      in_server_ang_vel,
                      in_target_pos,
                      net_height,
                      half_court_width);

                paths[i] = p;
            }


            // find two closest path ends
            List<d> index_double = new List<d>();

            for (int i = 0; i < num_vectors; i++)
            {
                vector_3 end_point = paths[i][paths[i].Count() - 1];
                vector_3 diff = end_point - in_target_pos;

                double val = diff.length();

                d dval = new d();
                dval.index = i;
                dval.val = val;

                index_double.Add(dval);
            }

            index_double.Sort();

            int smallest_index = index_double[0].index;
            int second_smallest_index = index_double[1].index;

            for (int i = 0; i < num_vectors; i++)
            {
                if (i == smallest_index)
                {
                    for (int j = 0; j < num_hone_iterations; j++)
                    {
                        List<vector_3> p = paths[i];
                        vector_3 sv = server_vels[i];
                        vector_3 sav = server_ang_vels[i];

                        hone_path(ref p, in_server_pos, ref sv, ref sav, in_target_pos, num_length_adjustment_iterations, net_height, half_court_width);

                        paths[i] = p;
                        server_vels[i] = sv;
                        server_ang_vels[i] = sav;
                    }

                    out_server_vel_1 = server_vels[i];
                    out_server_ang_vel_1 = server_ang_vels[i];
                    p_1 = paths[i];
                }

                if (i == second_smallest_index)
                {
                    for (int j = 0; j < num_hone_iterations; j++)
                    {
                        List<vector_3> p = paths[i];
                        vector_3 sv = server_vels[i];
                        vector_3 sav = server_ang_vels[i];

                        hone_path(ref p, in_server_pos, ref sv, ref sav, in_target_pos, num_length_adjustment_iterations, net_height, half_court_width);

                        paths[i] = p;
                        server_vels[i] = sv;
                        server_ang_vels[i] = sav;
                    }

                    out_server_vel_2 = server_vels[i];
                    out_server_ang_vel_2 = server_ang_vels[i];
                    p_2 = paths[i];
                }
            }
        }






    }

    class Program
    {
        static void Main(string[] args)
        {
            tennis_parameters t = new tennis_parameters();
            tennis_functions f = new tennis_functions();

            f.get_targets(
                 t.in_server_pos,
                 t.in_server_vel,
                 t.in_server_ang_vel,
                 t.in_target_pos,
                 ref t.out_server_vel_1,
                 ref t.out_server_ang_vel_1,
                 ref t.out_server_vel_2,
                 ref t.out_server_ang_vel_2,
                 ref t.out_p_1,
                 ref t.out_p_2,
                 t.num_vectors,
                 t.num_hone_iterations,
                 t.num_length_adjustment_iterations,
                 t.net_height,
                 t.half_court_width);

             
            Console.WriteLine(t.out_p_1[t.out_p_1.Count() - 1].x + " " + t.out_p_1[t.out_p_1.Count() - 1].y + " " + t.out_p_1[t.out_p_1.Count() - 1].z);
            Console.WriteLine(t.out_p_2[t.out_p_2.Count() - 1].x + " " + t.out_p_2[t.out_p_2.Count() - 1].y + " " + t.out_p_2[t.out_p_2.Count() - 1].z);






        }
    }
}
