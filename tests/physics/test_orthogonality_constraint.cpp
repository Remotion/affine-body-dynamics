#include <array>
#include <iomanip>
#include <iostream>

#include <catch2/catch.hpp>
#include <finitediff.hpp>
#include <igl/PI.h>
#include <igl/edges.h>

#include <physics/mass.hpp>
#include <problems/distance_barrier_rb_problem.hpp>
#include <utils/not_implemented_error.hpp>

using namespace ipc;
using namespace ipc::rigid;

RigidBody create_rigid_body_3DA(
    Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, Eigen::MatrixXi& faces,
    PoseD pose,
    PoseD velocity,
    double density)
{
    static int id = 0;
    int ndof = Pose<double>::dim_to_ndof(vertices.cols());
    return RigidBody(
        vertices, edges, faces,
        /*pose=*/pose,
        /*velcoity=*/velocity,
        /*force=*/Pose<double>::Zero(vertices.cols()),
        /*density=*/1.0,
        /*oriented=*/false,
        /*group=*/id++);
}

void rectangular_prism_3DA(double width, double depth, double height,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& edges,
    Eigen::MatrixXi& faces)
    {
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

TEST_CASE("ABD Potential", "[Distance-Barrier-RB-Problem][Body-Energy-Term]")
{
    const size_t dim = 3;
    int ndof = PoseD::dim_to_ndof(dim);
    const double width = 1.0;
    const double depth = 1.0;
    const double height = 1.0;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;
    rectangular_prism_3DA(width, depth, height, vertices, edges, faces);
    PoseD pose = PoseD::Eye(dim);
    SECTION("Translation Case")
    {
        pose.position << 1.0, 1.0, 1.0;
    }
    SECTION("90 Deg Rotation along X Case")
    {
        pose.transform << 0.0, 1.0, 0.0, 0.0, 1.0, 0.5, 1.0, 0.5, 0.3; // 0.5 * igl::PI
    }
    // pose.set_zero();
    PoseD& velocity = PoseD::Zero(dim);
    const double density = 1.0;
    RigidBody cube = create_rigid_body_3DA(vertices, edges, faces, pose, velocity, density);
    std::vector<RigidBody> rbs{cube};
    DistanceBarrierRBProblem rbp;
    rbp.init(rbs);
    // std::cout << "num vars : " << rbp.num_vars() << std::endl;
    Eigen::VectorXd x = 
        rbp.poses_to_dofs<double>({ pose });
    
    double energy = rbp.compute_energy_term(x);
    RigidBody body = rbp.m_assembler.m_rbs.at(0);
    MatrixMax12d mass_mat = body.mass_matrix;
    // Eigen::VectorXd q_t0 =  rbp.poses_to_dofs<double>({ rbp.poses_t0 });
    VectorX<double> x_bar = x;  //q_t0 + h*qdot_t0 + h*h*qddot_t0;
    double exp_energy = 0.0;

    exp_energy += (x-x_bar).dot(mass_mat*(x-x_bar));
    exp_energy /= 2.0;
    double k = rbp.orthogonal_stiffness();
    double h = rbp.timestep();
    double vol = rbs.at(0).volume;
    MatrixMax3d eye = Matrix3<double>::Identity();
    MatrixMax3d Q = pose.transform;
    exp_energy += h*h * (vol*k*(Q*Q.transpose() - eye).squaredNorm());

    CHECK(energy == Approx(exp_energy).margin(0.01));
}

TEST_CASE("ABD Gradient", "[Distance-Barrier-RB-Problem][Body-Energy-Diff]")
{
    const size_t dim = 3;
    int ndof = PoseD::dim_to_ndof(dim);
    const double width = 1.0;
    const double depth = 1.0;
    const double height = 1.0;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;
    rectangular_prism_3DA(width, depth, height, vertices, edges, faces);
    PoseD pose = PoseD::Eye(dim);
    SECTION("Translation Case")
    {
        pose.position << 1.0, 1.0, 1.0;
    }
    SECTION("90 Deg Rotation along X Case")
    {
        pose.transform << 0.0, 1.0, 0.0, 0.0, 1.0, 0.5, 1.0, 0.5, 0.3; // 0.5 * igl::PI
    }
    SECTION("Translation and Rotation Case")
    {
        pose.position << 1.0, 1.0, 1.0;
        pose.transform << 0.0, 1.0, 0.0, 0.0, 1.0, 0.5, 1.0, 0.5, 0.3; // 0.5 * igl::PI
    }
    // pose.set_zero();
    PoseD& velocity = PoseD::Zero(dim);
    const double density = 1.0;
    RigidBody cube = create_rigid_body_3DA(vertices, edges, faces, pose, velocity, density);
    std::vector<RigidBody> rbs{cube};
    DistanceBarrierRBProblem rbp;
    rbp.init(rbs);
    // std::cout << "num vars : " << rbp.num_vars() << std::endl;
    Eigen::VectorXd x = 
        rbp.poses_to_dofs<double>({ pose });
    Eigen::VectorXd grad_fx;
    double energy = rbp.compute_energy_term(x, grad_fx);
    Eigen::VectorXd grad_fx_approx = eval_grad_energy_approx(rbp, x);
    CHECK(fd::compare_gradient(grad_fx, grad_fx_approx));
}


TEST_CASE("ABD Hessian", "[Distance-Barrier-RB-Problem][Body-Energy-Hess]")
{
    const size_t dim = 3;
    int ndof = PoseD::dim_to_ndof(dim);
    const double width = 1.0;
    const double depth = 1.0;
    const double height = 1.0;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;
    rectangular_prism_3DA(width, depth, height, vertices, edges, faces);
    PoseD pose = PoseD::Eye(dim);
    SECTION("Translation Case")
    {
        pose.position << 1.0, 1.0, 1.0;
    }
    SECTION("90 Deg Rotation along X Case")
    {
        pose.transform << 0.0, 1.0, 0.0, 0.0, 1.0, 0.5, 1.0, 0.5, 0.3; // 0.5 * igl::PI
    }
    SECTION("Translation and Rotation Case")
    {
        pose.position << 1.0, 1.0, 1.0;
        pose.transform << 0.0, 1.0, 0.0, 0.0, 1.0, 0.5, 1.0, 0.5, 0.3; // 0.5 * igl::PI
    }
    // pose.set_zero();
    PoseD& velocity = PoseD::Zero(dim);
    const double density = 1.0;
    RigidBody cube = create_rigid_body_3DA(vertices, edges, faces, pose, velocity, density);
    std::vector<RigidBody> rbs{cube};
    DistanceBarrierRBProblem rbp;
    rbp.init(rbs);
    // std::cout << "num vars : " << rbp.num_vars() << std::endl;
    Eigen::VectorXd x = 
        rbp.poses_to_dofs<double>({ pose });
    Eigen::VectorXd grad_fx;
    Eigen::SparseMatrix<double> hess_fx;
    double energy = rbp.compute_energy_term(x, grad_fx, hess_fx);
   Eigen::MatrixXd hess_fx_approx = eval_hess_energy_approx(rbp, x);
   
    CHECK(fd::compare_jacobian(hess_fx.toDense(), hess_fx_approx));
}