#include <catch2/catch.hpp>

#include <iostream>
#include <filesystem>

#include <igl/edges.h>
#include <SimState.hpp>
#include <io/serialize_json.hpp>
#include <problems/distance_barrier_rb_problem.hpp>
#include <igl/PI.h>
#include "matplotlibcpp.h"

using namespace ipc;
using namespace ipc::rigid;
using namespace nlohmann;

namespace plt = matplotlibcpp;

//------------------
// Helpers

void rectangular_prism_3DB(double width, double depth, double height, Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, Eigen::MatrixXi& faces){
    int num_vertices = 8; // Rectangular prism
    int dim = 3;

    double dh = height/2.0;
    double dw = width/2.0;
    double dd = depth/2.0;

    vertices.resize(num_vertices, dim);
    vertices << -dw, -dd,  dh, 
                -dw,  dd,  dh,
                 dw,  dd,  dh,
                 dw, -dd,  dh,
                -dw, -dd, -dh,
                -dw,  dd, -dh,
                 dw,  dd, -dh,
                 dw, -dd, -dh;

    faces.resize(2 * 6, dim);
    faces <<    4, 3, 0,
                4, 0, 5,
                5, 0, 1,
                5, 1, 6,
                6, 1, 2,
                6, 2, 7,
                7, 2, 3,
                7, 3, 4,
                4, 6, 7,
                4, 5, 6,
                0, 3, 2,
                0, 2, 1;
    
    igl::edges(faces, edges);

    return;
}

std::string rectangular_prism_STL(double width, double depth, double height){
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;

    rectangular_prism_3DB(width, depth, height, vertices, edges, faces);

    Eigen::MatrixXd normals;
    normals.resizeLike(faces);

    for (int i = 0; i < faces.rows(); i++){
        Eigen::Vector3i v_ids = faces.row(i);
        Eigen::Vector3d e0 = vertices.row(v_ids[1]) - vertices.row(v_ids[0]);
        Eigen::Vector3d e2 = vertices.row(v_ids[2]) - vertices.row(v_ids[0]);
        normals.row(i) = e0.cross(e2).transpose().normalized();
    }

    Eigen::IOFormat vec_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", " ", "\n");

    std::stringstream ss;
    ss << "solid ASCII\n";
    for (int i = 0; i < faces.rows(); i++){
        ss << "\tfacet normal " << normals.row(i).format(vec_fmt);
        ss << "\t\touter loop\n";
        for (int v_i = 0; v_i < faces.cols(); v_i++){
            ss << "\t\t\tvertex " << vertices.row(faces(i,v_i)).format(vec_fmt);
        }
        ss << "\t\tendloop\n";
        ss << "\tendfacet\n";
    }
    ss << "endsolid\n";

    return ss.str();
}

json single_body_json_template(){
    json jsonInput = R"({
        "scene_type": "distance_barrier_rb_problem",
        "solver": "ipc_solver",
        "timestep": 0.001,
        "max_time": 0.5,
        "distance_barrier_constraint": {
            "barrier_type": "ipc",
            "initial_barrier_activation_distance": 1e-4
        },
        "ipc_solver": {
            "velocity_conv_tol": 1e-5
        },
        "friction_constraints": {
            "iterations": -1
        },
        "rigid_body_problem": {
            "coefficient_restitution": 0.3,
            "coefficient_friction": 0.3,
            "gravity": [0, 0, 0],
            "rigid_bodies": [{
                "mesh": "",
                "position": [0, 0, 0],
                "force": [0, 0, 0],
                "rotation": [0, 0, 0],
                "density": 1e-6
            }]
        }
        })"_json;
    return jsonInput;
}


class TempFile {
public:
    TempFile(const std::string& filename) : filename(filename){
        this->fpath = std::filesystem::temp_directory_path().string() + filename;
    }
    void write(const std::string& str){
        std::ofstream out(fpath);
        out << str; out.close();
    }
    std::string path(){return fpath;}

    ~TempFile() {std::remove(fpath.c_str());}
private:
    std::string filename;
    std::string fpath;
};

template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

//------------------

//------------------
// Tests

TEST_CASE("Test Z Force", "[validation][linear_forces]")
{
    // Box dimensions
    double width = 100.0; // mm
    double depth = 100.0; // mm
    double height = 100.0; // mm

    // Creating the input geometry as stl file
    TempFile tmpSTL("box.stl");
    std::string stl_content = rectangular_prism_STL(width, depth, height);
    tmpSTL.write(stl_content);

    // Creating the input json file
    json jsonInput = single_body_json_template();

    // First body json reference
    auto& body_json = jsonInput["rigid_body_problem"]["rigid_bodies"][0];

    // Setting the mesh filepath
    body_json["mesh"] = tmpSTL.path();

    // -----------------------------------
    // Settings

    // Force vector
    Eigen::Vector3d force(0.0, 0.0, -500.0);

    auto& force_json = body_json["force"];
    for (int i = 0; i < 3; i++){
        force_json[i] = force[i];
    }

    // Density
    // Calculate required density to get mass of 1 tonne
    double vol = width * depth * height;
    double rho = 1.0/vol;

    body_json["density"] = rho;

    // Max time
    double max_time = 10.0;
    jsonInput["max_time"] = max_time;

    // Write out the json file
    TempFile tmpJson("input.json");
    tmpJson.write(jsonInput.dump());

    // Run the simulation
    SimState sim;

    sim.load_scene(tmpJson.path(), "");

    DistanceBarrierRBProblem* problem_ptr = dynamic_cast<DistanceBarrierRBProblem*>(sim.problem_ptr.get());
    if (!problem_ptr) throw "Could not cast problem to DistanceBarrierRBProblem";

    std::shared_ptr<DistanceBarrierRBProblem> distRBProblem_ptr = std::shared_ptr<DistanceBarrierRBProblem>(sim.problem_ptr, problem_ptr);

    // ---------------------------------------------------------
    // Checking state before simulation
    // VectorMax3d linear_pos = problem_ptr->m_assembler.m_rbs[0].pose.position;
    // std::cout << "Start Linear Position: \n" << linear_pos << std::endl;

    // MatrixMax3d trans_pos = problem_ptr->m_assembler.m_rbs[0].pose.transform;
    // std::cout << "Start Transform Position: \n" << trans_pos << std::endl;

    // VectorMax3d linear_vel = problem_ptr->m_assembler.m_rbs[0].velocity.position;
    // std::cout << "Start Linear Velocity: \n" << linear_vel << std::endl;

    // MatrixMax3d trans_vel = problem_ptr->m_assembler.m_rbs[0].velocity.transform;
    // std::cout << "Start Transform Velocity: \n" << trans_vel << std::endl;


    // ---------------------------------------------------------
    // Simulation
    TempFile tmpOut("output.txt");
    sim.run_simulation(tmpOut.path());

    // ---------------------------------------------------------
    // Checking state after simulation

    double mass = problem_ptr->m_assembler.m_rbs[0].mass;
    // std::cout << "Mass of rigid body: " << mass << std::endl;

    // MatrixMax12d mass_mat = problem_ptr->m_assembler.m_rbs[0].mass_matrix;
    // std::cout << "Generalized Mass Matrix: \n" << mass_mat << std::endl;

    // VectorMax3d e_linear_pos = problem_ptr->m_assembler.m_rbs[0].pose.position;
    // std::cout << "Final Linear Position: \n" << e_linear_pos << std::endl;

    // MatrixMax3d e_trans_pos = problem_ptr->m_assembler.m_rbs[0].pose.transform;
    // std::cout << "Final Transform Position: \n" << e_trans_pos << std::endl;

    // VectorMax3d e_linear_vel = problem_ptr->m_assembler.m_rbs[0].velocity.position;
    // std::cout << "Final Linear Velocity: \n" << e_linear_vel << std::endl;

    // MatrixMax3d e_trans_vel = problem_ptr->m_assembler.m_rbs[0].velocity.transform;
    // std::cout << "Final Transform Velocity: \n" << e_trans_vel << std::endl;

    // Collecting simulation results
    int num_states = sim.state_sequence.size();
    Eigen::MatrixXd positions;
    positions.resize(num_states, 3);
    Eigen::MatrixXd velocities;
    velocities.resize(num_states, 3);

    std::vector<Eigen::Matrix3d> transforms;
    transforms.reserve(num_states);

    for (int i = 0; i < num_states; i++){
        auto& rb_json = sim.state_sequence[i]["rigid_bodies"][0];
        auto& pos_json = rb_json["position"];
        Eigen::VectorXd pos(3);
        ipc::rigid::from_json(pos_json, pos);
        positions.row(i) = pos.transpose();

        auto& vel_json = rb_json["linear_velocity"];
        Eigen::VectorXd vel(3);
        ipc::rigid::from_json(vel_json, vel);
        velocities.row(i) = vel.transpose();

        Eigen::Matrix3d trans;
        auto& trans_json = rb_json["transform"];
        ipc::rigid::from_json(trans_json, trans);
        transforms.push_back(trans);
    }

    // Analytically calculated solution
    int num_timesteps = num_states - 1;
    double final_time = problem_ptr->timestep() * num_timesteps;
    std::vector<double> times = linspace(0.0, final_time, num_timesteps + 1);

    Eigen::Vector3d linear_acc = force / mass;
    double z_acc = linear_acc[2];

    std::vector<double> z_vel_expected(times.size());
    std::vector<double> z_pos_expected(times.size());

    std::transform(times.begin(), times.end(), z_vel_expected.begin(),
        [z_acc](double t) { return z_acc*t; }
    );

    std::transform(times.begin(), times.end(), z_pos_expected.begin(),
        [z_acc](double t) { return 0.5 * z_acc * t*t; }
    );

    // ---------------------------------------
    // Checking solutions

    // Displacement in X and Y should be zero
    CHECK(positions.col(0).squaredNorm() <= 1E-8);
    CHECK(positions.col(1).squaredNorm() <= 1E-8);

    // Velocity in X and Y should be zero
    CHECK(velocities.col(0).squaredNorm() <= 1E-8);
    CHECK(velocities.col(1).squaredNorm() <= 1E-8);

    // Deformations should be identity
    double deform_error = 0.0;
    for (Eigen::Matrix3d& trans : transforms){
        deform_error += (trans - Eigen::Matrix3d::Identity()).squaredNorm();
    }
    CHECK(deform_error <= 1E-8);

    // ---------------------------------------
    // Plotting solutions

    // Extracting out z_position and z_velocity
    std::vector<double> z_pos(positions.rows());
    Eigen::VectorXd::Map(&z_pos[0], positions.rows()) = positions.col(2);

    std::vector<double> z_vel(velocities.rows());
    Eigen::VectorXd::Map(&z_vel[0], velocities.rows()) = velocities.col(2);


    // NOTE: Environment variable PYTHONHOME may need to be set for plotting to work
    // PYTHONHOME can be set to the directory containing the python interpreter
    plt::plot(times, z_pos, {{"label", "Position Z - ABD"}});
    plt::plot(times, z_pos_expected, {{"label", "Position Z - Analytical"}});
    plt::legend();
    plt::xlabel("time (s)");
    plt::ylabel("displacement (mm)");
    plt::show();

    plt::figure();
    plt::plot(times, z_vel, {{"label", "Velocity Z - ABD"}});
    plt::plot(times, z_vel_expected, {{"label", "Velocity Z - Analytical"}});
    plt::legend();
    plt::xlabel("time (s)");
    plt::ylabel("Velocity (mm/s)");
    plt::show();
    std::cout << "";
}


TEST_CASE("Test Torque", "[validation][torque]")
{
    // Box dimensions
    double width = 100.0; // mm
    double depth = 100.0; // mm
    double height = 100.0; // mm

    // Creating the input geometry as stl file
    TempFile tmpSTL("box.stl");
    std::string stl_content = rectangular_prism_STL(width, depth, height);
    tmpSTL.write(stl_content);

    // Creating the input json file
    json jsonInput = single_body_json_template();

    // First body json reference
    auto& body_json = jsonInput["rigid_body_problem"]["rigid_bodies"][0];

    // -----------------------------------
    // Settings

    // Mesh filepath
    body_json["mesh"] = tmpSTL.path();

    // Torque vector
    Eigen::Vector3d torque(0.0, 0.0, 50000.0);

    auto& torque_json = body_json["torque"];
    for (int i = 0; i < 3; i++){
        torque_json[i] = torque[i];
    }

    // Density
    // Calculate required density to get mass of 1 tonne
    double vol = width * depth * height;
    double rho = 1.0/vol;

    body_json["density"] = rho;

    // Max time
    double max_time = 1.0;
    jsonInput["max_time"] = max_time;

    // Write out the json file
    TempFile tmpJson("input.json");
    tmpJson.write(jsonInput.dump());

    // -----------------------------------
    // Init simulation

    SimState sim;

    sim.load_scene(tmpJson.path(), "");

    DistanceBarrierRBProblem* problem_ptr = dynamic_cast<DistanceBarrierRBProblem*>(sim.problem_ptr.get());
    if (!problem_ptr) throw "Could not cast problem to DistanceBarrierRBProblem";

    std::shared_ptr<DistanceBarrierRBProblem> distRBProblem_ptr = std::shared_ptr<DistanceBarrierRBProblem>(sim.problem_ptr, problem_ptr);

    // -----------------------------------
    // Checking state before simulation
    VectorMax3d linear_pos = problem_ptr->m_assembler.m_rbs[0].pose.position;
    std::cout << "Start Linear Position: \n" << linear_pos << std::endl;

    MatrixMax3d trans_pos = problem_ptr->m_assembler.m_rbs[0].pose.transform;
    std::cout << "Start Transform Position: \n" << trans_pos << std::endl;

    VectorMax3d linear_vel = problem_ptr->m_assembler.m_rbs[0].velocity.position;
    std::cout << "Start Linear Velocity: \n" << linear_vel << std::endl;

    MatrixMax3d trans_vel = problem_ptr->m_assembler.m_rbs[0].velocity.transform;
    std::cout << "Start Transform Velocity: \n" << trans_vel << std::endl;


    // -----------------------------------
    // Run Simulation
    TempFile tmpOut("output.txt");
    sim.run_simulation(tmpOut.path());

    // -----------------------------------
    // Checking state after simulation

    double mass = problem_ptr->m_assembler.m_rbs[0].mass;
    std::cout << "Mass of rigid body: " << mass << std::endl;

    MatrixMax12d mass_mat = problem_ptr->m_assembler.m_rbs[0].mass_matrix;
    std::cout << "Generalized Mass Matrix: \n" << mass_mat << std::endl;

    VectorMax3d e_linear_pos = problem_ptr->m_assembler.m_rbs[0].pose.position;
    std::cout << "Final Linear Position: \n" << e_linear_pos << std::endl;

    MatrixMax3d e_trans_pos = problem_ptr->m_assembler.m_rbs[0].pose.transform;
    std::cout << "Final Transform Position: \n" << e_trans_pos << std::endl;

    VectorMax3d e_linear_vel = problem_ptr->m_assembler.m_rbs[0].velocity.position;
    std::cout << "Final Linear Velocity: \n" << e_linear_vel << std::endl;

    MatrixMax3d e_trans_vel = problem_ptr->m_assembler.m_rbs[0].velocity.transform;
    std::cout << "Final Transform Velocity: \n" << e_trans_vel << std::endl;

    // Grabbing the state history to check accuracy of solution
    int num_states = sim.state_sequence.size();
    Eigen::MatrixXd positions;
    positions.resize(num_states, 3);
    Eigen::MatrixXd velocities;
    velocities.resize(num_states, 3);

    std::vector<MatrixMax3d> transforms;
    transforms.reserve(num_states);

    std::cout << sim.state_sequence[0].dump() << std::endl;

    for (int i = 0; i < num_states; i++){
        auto& rb_json = sim.state_sequence[i]["rigid_bodies"][0];
        auto& pos_json = rb_json["position"];
        Eigen::VectorXd pos(3);
        ipc::rigid::from_json(pos_json, pos);
        positions.row(i) = pos.transpose();

        auto& vel_json = rb_json["linear_velocity"];
        Eigen::VectorXd vel(3);
        ipc::rigid::from_json(vel_json, vel);
        velocities.row(i) = vel.transpose();

        MatrixMax3d trans(3,3);
        auto& trans_json = rb_json["transform"];
        ipc::rigid::from_json(trans_json, trans);
        transforms.push_back(trans);
    }

    // Converting transformation matrix to closest rotation vector
    Eigen::MatrixXd rot_vecs(num_states, 3);

    int i = 0;
    for (const MatrixMax3d& transform : transforms){
        Eigen::AngleAxisd a(construct_quaternion_from_transform(transform));
        rot_vecs.row(i) = a.angle() * a.axis();
        i++;
    }

    // -----------------------------------
    // Analytically calculated solution
    int num_timesteps = num_states - 1;
    double final_time = problem_ptr->timestep() * num_timesteps;
    std::vector<double> times = linspace(0.0, final_time, num_timesteps + 1);

    // Moment of inertia is diagonal
    // I = mass * edge_len^2 / 6 * Identity 
    double moment_inertia = mass * width * width / 6.0;
    // α = I^-1 * 𝜏
    Eigen::Vector3d angular_acc = torque / moment_inertia;

    double z_angular_acc = angular_acc[2];

    std::vector<double> z_angle_expected(times.size());
    std::vector<double> angular_vel_expected(times.size());

    std::transform(times.begin(), times.end(), angular_vel_expected.begin(),
        [z_angular_acc](double t) { return z_angular_acc*t; }
    );

    std::transform(times.begin(), times.end(), z_angle_expected.begin(),
        [z_angular_acc](double t) { return 0.5 * z_angular_acc * t*t; }
    );

    // Clipping angle to be between -π and +π
    std::transform(z_angle_expected.begin(), z_angle_expected.end(), z_angle_expected.begin(),
        [](const double angle) { return angle + 2 * floor((igl::PI - angle)/(2*igl::PI)) * igl::PI; }
    );


    // ---------------------------------------
    // Checking solutions

    // Displacement in X and Y should be zero
    CHECK(positions.col(0).squaredNorm() <= 1E-8);
    CHECK(positions.col(1).squaredNorm() <= 1E-8);

    // Velocity in X and Y should be zero
    CHECK(velocities.col(0).squaredNorm() <= 1E-8);
    CHECK(velocities.col(1).squaredNorm() <= 1E-8);

    // Angular displacements along X and Y should be zero
    CHECK(rot_vecs.col(0).squaredNorm() <= 1E-8);
    CHECK(rot_vecs.col(1).squaredNorm() <= 1E-8);

    // ---------------------------------------
    // Plotting solutions

    // Extracting out z_angle
    std::vector<double> z_angle(rot_vecs.rows());
    Eigen::VectorXd::Map(&z_angle[0], rot_vecs.rows()) = rot_vecs.col(2);

    // NOTE: Environment variable PYTHONHOME may need to be set for plotting to work
    // PYTHONHOME can be set to the directory containing the python interpreter
    plt::plot(times, z_angle, {{"label", "Z Angle - ABD"}});
    plt::plot(times, z_angle_expected, {{"label", "Z Angle - Analytical"}});
    plt::legend();
    plt::xlabel("time (s)");
    plt::ylabel("Angular displacement about Z-axis (rad)");
    plt::show();
}


TEST_CASE("Test Pin World", "[validation][linear_constraint][pin_world]")
{
    // Box dimensions
    double width = 10.0; // mm
    double depth = 10.0; // mm
    double height = 10.0; // mm

    // Creating the input geometry as stl file
    TempFile tmpSTL("box.stl");
    std::string stl_content = rectangular_prism_STL(width, depth, height);
    tmpSTL.write(stl_content);

    // Creating the input json file
    json jsonInput = single_body_json_template();

    // First body json reference
    auto& body_json = jsonInput["rigid_body_problem"]["rigid_bodies"][0];

    // -----------------------------------
    // Settings

    // Mesh filepath
    body_json["mesh"] = tmpSTL.path();

    // Force vector
    Eigen::Vector3d force(0.0, 0.0, -50000.0);

    auto& force_json = body_json["force"];
    for (int i = 0; i < 3; i++){
        force_json[i] = force[i];
    }

    // Density
    // Calculate required density to get mass of 1 tonne
    double vol = width * depth * height;
    double rho = 1.0/vol;

    body_json["density"] = rho;

    // Max time
    double max_time = 1.0;
    jsonInput["max_time"] = max_time;

    // Linear constraint
    auto cons = R"([
                    {
                        "type": "pin_world",
                        "body_name": "box",
                        "body_point": [-500.0, 0.0, 0.0]
                    }
                ])"_json;

    Eigen::Vector3d body_point(-500, 0.0, 0.0);

    jsonInput["rigid_body_problem"]["linear_constraints"] = cons;


    // Write out the json file
    TempFile tmpJson("input.json");
    tmpJson.write(jsonInput.dump());

    // -----------------------------------
    // Init simulation

    SimState sim;

    sim.load_scene(tmpJson.path(), "");

    DistanceBarrierRBProblem* problem_ptr = dynamic_cast<DistanceBarrierRBProblem*>(sim.problem_ptr.get());
    if (!problem_ptr) throw "Could not cast problem to DistanceBarrierRBProblem";

    std::shared_ptr<DistanceBarrierRBProblem> distRBProblem_ptr = std::shared_ptr<DistanceBarrierRBProblem>(sim.problem_ptr, problem_ptr);

    // -----------------------------------
    // Checking state before simulation
    VectorMax3d linear_pos = problem_ptr->m_assembler.m_rbs[0].pose.position;
    std::cout << "Start Linear Position: \n" << linear_pos << std::endl;

    MatrixMax3d trans_pos = problem_ptr->m_assembler.m_rbs[0].pose.transform;
    std::cout << "Start Transform Position: \n" << trans_pos << std::endl;

    VectorMax3d linear_vel = problem_ptr->m_assembler.m_rbs[0].velocity.position;
    std::cout << "Start Linear Velocity: \n" << linear_vel << std::endl;

    MatrixMax3d trans_vel = problem_ptr->m_assembler.m_rbs[0].velocity.transform;
    std::cout << "Start Transform Velocity: \n" << trans_vel << std::endl;


    // -----------------------------------
    // Run Simulation
    TempFile tmpOut("output.txt");
    sim.run_simulation(tmpOut.path());

    // -----------------------------------
    // Checking state after simulation

    double mass = problem_ptr->m_assembler.m_rbs[0].mass;
    std::cout << "Mass of rigid body: " << mass << std::endl;

    MatrixMax12d mass_mat = problem_ptr->m_assembler.m_rbs[0].mass_matrix;
    std::cout << "Generalized Mass Matrix: \n" << mass_mat << std::endl;

    VectorMax3d e_linear_pos = problem_ptr->m_assembler.m_rbs[0].pose.position;
    std::cout << "Final Linear Position: \n" << e_linear_pos << std::endl;

    MatrixMax3d e_trans_pos = problem_ptr->m_assembler.m_rbs[0].pose.transform;
    std::cout << "Final Transform Position: \n" << e_trans_pos << std::endl;

    VectorMax3d e_linear_vel = problem_ptr->m_assembler.m_rbs[0].velocity.position;
    std::cout << "Final Linear Velocity: \n" << e_linear_vel << std::endl;

    MatrixMax3d e_trans_vel = problem_ptr->m_assembler.m_rbs[0].velocity.transform;
    std::cout << "Final Transform Velocity: \n" << e_trans_vel << std::endl;

    // Grabbing the state history to check accuracy of solution
    int num_states = sim.state_sequence.size();
    Eigen::MatrixXd positions;
    positions.resize(num_states, 3);
    Eigen::MatrixXd velocities;
    velocities.resize(num_states, 3);

    std::vector<MatrixMax3d> transforms;
    transforms.reserve(num_states);

    std::cout << sim.state_sequence[0].dump() << std::endl;

    for (int i = 0; i < num_states; i++){
        auto& rb_json = sim.state_sequence[i]["rigid_bodies"][0];
        auto& pos_json = rb_json["position"];
        Eigen::VectorXd pos(3);
        ipc::rigid::from_json(pos_json, pos);
        positions.row(i) = pos.transpose();

        auto& vel_json = rb_json["linear_velocity"];
        Eigen::VectorXd vel(3);
        ipc::rigid::from_json(vel_json, vel);
        velocities.row(i) = vel.transpose();

        MatrixMax3d trans(3,3);
        auto& trans_json = rb_json["transform"];
        ipc::rigid::from_json(trans_json, trans);
        transforms.push_back(trans);
    }

    // Converting transformation matrix to closest rotation vector
    Eigen::MatrixXd rot_vecs(num_states, 3);

    int i = 0;
    for (const MatrixMax3d& transform : transforms){
        Eigen::AngleAxisd a(construct_quaternion_from_transform(transform));
        rot_vecs.row(i) = a.angle() * a.axis();
        i++;
    }

    // -----------------------------------
    // Numerically calculated solution
    int num_timesteps = num_states - 1;
    double final_time = problem_ptr->timestep() * num_timesteps;
    std::vector<double> times = linspace(0.0, final_time, num_timesteps + 1);

    // Moment of inertia is diagonal
    // I_c = mass * edge_len^2 / 6 * Identity 
    double moment_inertia = mass * width * width / 6.0;
    // I = I_c + mass * d^2
    moment_inertia = moment_inertia + mass * body_point[0] * body_point[0];
    double moi_inv = 1.0 / moment_inertia;

    // Using forward euler to integrate numerically
    std::vector<double> thetas = {0.0};
    std::vector<double> theta_dots = {0.0};
    double theta = thetas[0];
    double theta_dot = theta_dots[0];
    double dt = 0.001;
    for (int i = 0; i < times.size() - 1; i++){
        double tau = force[2] * cos(theta) * body_point[0];
        theta = theta + theta_dot * dt;
        theta_dot = theta_dot + moi_inv * tau * dt;
        thetas.push_back(theta);        
        theta_dots.push_back(theta_dot);
    }

    // double z_angular_acc = angular_acc[2];

    // std::vector<double> z_angle_expected(times.size());
    // std::vector<double> angular_vel_expected(times.size());

    // std::transform(times.begin(), times.end(), angular_vel_expected.begin(),
    //     [z_angular_acc](double t) { return z_angular_acc*t; }
    // );

    // std::transform(times.begin(), times.end(), z_angle_expected.begin(),
    //     [z_angular_acc](double t) { return 0.5 * z_angular_acc * t*t; }
    // );

    // // Clipping angle to be between -π and +π
    // std::transform(z_angle_expected.begin(), z_angle_expected.end(), z_angle_expected.begin(),
    //     [](const double angle) { return angle + 2 * floor((igl::PI - angle)/(2*igl::PI)) * igl::PI; }
    // );


    // ---------------------------------------
    // Checking solutions

    // // Displacement in X and Y should be zero
    // CHECK(positions.col(0).squaredNorm() <= 1E-8);
    // CHECK(positions.col(1).squaredNorm() <= 1E-8);

    // // Velocity in X and Y should be zero
    // CHECK(velocities.col(0).squaredNorm() <= 1E-8);
    // CHECK(velocities.col(1).squaredNorm() <= 1E-8);

    // // Angular displacements along X and Y should be zero
    // CHECK(rot_vecs.col(0).squaredNorm() <= 1E-8);
    // CHECK(rot_vecs.col(1).squaredNorm() <= 1E-8);

    // ---------------------------------------
    // Plotting solutions

    // Extracting out y_angle
    std::vector<double> y_angle(rot_vecs.rows());
    Eigen::VectorXd::Map(&y_angle[0], rot_vecs.rows()) = rot_vecs.col(1);

    // NOTE: Environment variable PYTHONHOME may need to be set for plotting to work
    // PYTHONHOME can be set to the directory containing the python interpreter
    plt::plot(times, y_angle, {{"label", "Y Angle - ABD"}});
    plt::plot(times, thetas, {{"label", "Y Angle - Simple Numerical"}});
    plt::legend();
    plt::xlabel("time (s)");
    plt::ylabel("Angular displacement about Y-axis (rad)");
    plt::show();

    // Extracting out z_position
    std::vector<double> z_pos(positions.rows());
    Eigen::VectorXd::Map(&z_pos[0], positions.rows()) = positions.col(2);

    plt::figure();
    plt::plot(times, z_pos, {{"label", "Z Position - ABD"}});
    plt::legend();
    plt::xlabel("time (s)");
    plt::ylabel("Z Position (mm)");
    plt::show();
}

TEST_CASE("Test Newmark", "[validation][torque][newmark]")
{
    // Box dimensions
    double width = 100.0; // mm
    double depth = 100.0; // mm
    double height = 100.0; // mm

    // Creating the input geometry as stl file
    TempFile tmpSTL("box.stl");
    std::string stl_content = rectangular_prism_STL(width, depth, height);
    tmpSTL.write(stl_content);

    // Creating the input json file
    json jsonInput = single_body_json_template();

    // First body json reference
    auto& body_json = jsonInput["rigid_body_problem"]["rigid_bodies"][0];

    // -----------------------------------
    // Settings

    // Mesh filepath
    body_json["mesh"] = tmpSTL.path();

    // Torque vector
    Eigen::Vector3d torque(0.0, 0.0, 50000.0);

    auto& torque_json = body_json["torque"];
    for (int i = 0; i < 3; i++){
        torque_json[i] = torque[i];
    }

    // Density
    // Calculate required density to get mass of 1 tonne
    double vol = width * depth * height;
    double rho = 1.0/vol;

    body_json["density"] = rho;

    // Max time
    double max_time = 1.0;
    jsonInput["max_time"] = max_time;

    // Time stepper
    jsonInput["rigid_body_problem"]["time_stepper"] = "implicit_newmark";

    // Write out the json file
    TempFile tmpJson("input.json");
    tmpJson.write(jsonInput.dump());

    // -----------------------------------
    // Init simulation

    SimState sim;

    sim.load_scene(tmpJson.path(), "");

    DistanceBarrierRBProblem* problem_ptr = dynamic_cast<DistanceBarrierRBProblem*>(sim.problem_ptr.get());
    if (!problem_ptr) throw "Could not cast problem to DistanceBarrierRBProblem";

    std::shared_ptr<DistanceBarrierRBProblem> distRBProblem_ptr = std::shared_ptr<DistanceBarrierRBProblem>(sim.problem_ptr, problem_ptr);

    // -----------------------------------
    // Checking state before simulation
    VectorMax3d linear_pos = problem_ptr->m_assembler.m_rbs[0].pose.position;
    std::cout << "Start Linear Position: \n" << linear_pos << std::endl;

    MatrixMax3d trans_pos = problem_ptr->m_assembler.m_rbs[0].pose.transform;
    std::cout << "Start Transform Position: \n" << trans_pos << std::endl;

    VectorMax3d linear_vel = problem_ptr->m_assembler.m_rbs[0].velocity.position;
    std::cout << "Start Linear Velocity: \n" << linear_vel << std::endl;

    MatrixMax3d trans_vel = problem_ptr->m_assembler.m_rbs[0].velocity.transform;
    std::cout << "Start Transform Velocity: \n" << trans_vel << std::endl;


    // -----------------------------------
    // Run Simulation
    TempFile tmpOut("output.txt");
    sim.run_simulation(tmpOut.path());

    // -----------------------------------
    // Checking state after simulation

    double mass = problem_ptr->m_assembler.m_rbs[0].mass;
    std::cout << "Mass of rigid body: " << mass << std::endl;

    MatrixMax12d mass_mat = problem_ptr->m_assembler.m_rbs[0].mass_matrix;
    std::cout << "Generalized Mass Matrix: \n" << mass_mat << std::endl;

    VectorMax3d e_linear_pos = problem_ptr->m_assembler.m_rbs[0].pose.position;
    std::cout << "Final Linear Position: \n" << e_linear_pos << std::endl;

    MatrixMax3d e_trans_pos = problem_ptr->m_assembler.m_rbs[0].pose.transform;
    std::cout << "Final Transform Position: \n" << e_trans_pos << std::endl;

    VectorMax3d e_linear_vel = problem_ptr->m_assembler.m_rbs[0].velocity.position;
    std::cout << "Final Linear Velocity: \n" << e_linear_vel << std::endl;

    MatrixMax3d e_trans_vel = problem_ptr->m_assembler.m_rbs[0].velocity.transform;
    std::cout << "Final Transform Velocity: \n" << e_trans_vel << std::endl;

    // Grabbing the state history to check accuracy of solution
    int num_states = sim.state_sequence.size();
    Eigen::MatrixXd positions;
    positions.resize(num_states, 3);
    Eigen::MatrixXd velocities;
    velocities.resize(num_states, 3);

    std::vector<MatrixMax3d> transforms;
    transforms.reserve(num_states);

    std::cout << sim.state_sequence[0].dump() << std::endl;

    for (int i = 0; i < num_states; i++){
        auto& rb_json = sim.state_sequence[i]["rigid_bodies"][0];
        auto& pos_json = rb_json["position"];
        Eigen::VectorXd pos(3);
        ipc::rigid::from_json(pos_json, pos);
        positions.row(i) = pos.transpose();

        auto& vel_json = rb_json["linear_velocity"];
        Eigen::VectorXd vel(3);
        ipc::rigid::from_json(vel_json, vel);
        velocities.row(i) = vel.transpose();

        MatrixMax3d trans(3,3);
        auto& trans_json = rb_json["transform"];
        ipc::rigid::from_json(trans_json, trans);
        transforms.push_back(trans);
    }

    // Converting transformation matrix to closest rotation vector
    Eigen::MatrixXd rot_vecs(num_states, 3);

    int i = 0;
    for (const MatrixMax3d& transform : transforms){
        Eigen::AngleAxisd a(construct_quaternion_from_transform(transform));
        rot_vecs.row(i) = a.angle() * a.axis();
        i++;
    }

    // -----------------------------------
    // Analytically calculated solution
    int num_timesteps = num_states - 1;
    double final_time = problem_ptr->timestep() * num_timesteps;
    std::vector<double> times = linspace(0.0, final_time, num_timesteps + 1);

    // Moment of inertia is diagonal
    // I = mass * edge_len^2 / 6 * Identity 
    double moment_inertia = mass * width * width / 6.0;
    // α = I^-1 * 𝜏
    Eigen::Vector3d angular_acc = torque / moment_inertia;

    double z_angular_acc = angular_acc[2];

    std::vector<double> z_angle_expected(times.size());
    std::vector<double> angular_vel_expected(times.size());

    std::transform(times.begin(), times.end(), angular_vel_expected.begin(),
        [z_angular_acc](double t) { return z_angular_acc*t; }
    );

    std::transform(times.begin(), times.end(), z_angle_expected.begin(),
        [z_angular_acc](double t) { return 0.5 * z_angular_acc * t*t; }
    );

    // Clipping angle to be between -π and +π
    std::transform(z_angle_expected.begin(), z_angle_expected.end(), z_angle_expected.begin(),
        [](const double angle) { return angle + 2 * floor((igl::PI - angle)/(2*igl::PI)) * igl::PI; }
    );


    // ---------------------------------------
    // Checking solutions

    // Displacement in X and Y should be zero
    CHECK(positions.col(0).squaredNorm() <= 1E-8);
    CHECK(positions.col(1).squaredNorm() <= 1E-8);

    // Velocity in X and Y should be zero
    CHECK(velocities.col(0).squaredNorm() <= 1E-8);
    CHECK(velocities.col(1).squaredNorm() <= 1E-8);

    // Angular displacements along X and Y should be zero
    CHECK(rot_vecs.col(0).squaredNorm() <= 1E-8);
    CHECK(rot_vecs.col(1).squaredNorm() <= 1E-8);

    // ---------------------------------------
    // Plotting solutions

    // Extracting out z_angle
    std::vector<double> z_angle(rot_vecs.rows());
    Eigen::VectorXd::Map(&z_angle[0], rot_vecs.rows()) = rot_vecs.col(2);

    // Estimating angular_vel
    std::vector<double> ang_vel(rot_vecs.rows());
    // Starting ang_vel = 0
    ang_vel[0] = 0.0;
    for (int i = 1; i < ang_vel.size(); i++){
        ang_vel[i] = (rot_vecs(i,2) - rot_vecs(i-1,2)) / problem_ptr->timestep();
    }

    // NOTE: Environment variable PYTHONHOME may need to be set for plotting to work
    // PYTHONHOME can be set to the directory containing the python interpreter
    plt::plot(times, z_angle, {{"label", "Z Angle - ABD"}});
    plt::plot(times, z_angle_expected, {{"label", "Z Angle - Analytical"}});
    plt::legend();
    plt::xlabel("time (s)");
    plt::ylabel("Angular displacement about Z-axis (rad)");
    plt::show();

    plt::plot(times, ang_vel, {{"label", "Angular Velocity - ABD"}});
    plt::plot(times, angular_vel_expected, {{"label", "Angular Velocity - Analytical"}});
    plt::legend();
    plt::xlabel("time (s)");
    plt::ylabel("Angular velocity about Z-axis (rad/s)");
    plt::ylim(angular_vel_expected[0], angular_vel_expected.back());
    plt::show();
}