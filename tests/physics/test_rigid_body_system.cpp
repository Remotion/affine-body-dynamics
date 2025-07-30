#include <array>
#include <iomanip>
#include <iostream>

#include <catch2/catch.hpp>

#include <igl/PI.h>
#include <igl/edges.h>

#include <physics/rigid_body_assembler.hpp>

// ---------------------------------------------------
// Tests
// ---------------------------------------------------
using namespace ipc;
using namespace ipc::rigid;

RigidBody simple_rigid_body(
    Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, Pose<double> velocity)
{
    static int id = 0;
    int ndof = Pose<double>::dim_to_ndof(vertices.cols());
    return RigidBody(
        vertices, edges, /*pose=*/Pose<double>::Eye(vertices.cols()), velocity,
        /*force=*/Pose<double>::Zero(vertices.cols()),
        /*density=*/1.0,
        /*oriented=*/false,
        /*group=*/id++);
}

RigidBody simple_rigid_body_3D(
    Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, Eigen::MatrixXi& faces, Pose<double> velocity)
{
    static int id = 0;
    int ndof = Pose<double>::dim_to_ndof(vertices.cols());
    return RigidBody(
        vertices, edges, faces,
        /*pose=*/Pose<double>::Eye(vertices.cols()),
        /*velcoity=*/velocity,
        /*force=*/Pose<double>::Zero(vertices.cols()),
        /*density=*/1.0,
        /*oriented=*/false,
        /*group=*/id++);
}

void rectangular_prism_3D(double width, double depth, double height, Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, Eigen::MatrixXi& faces){
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

// TODO: 2D Rigid Body not yet implemented
// TEST_CASE("Rigid Body System Transform", "[RB][RB-System][RB-System-transform]")
// {
//     Eigen::MatrixXd vertices(4, 2);
//     Eigen::MatrixXi edges(4, 2);
//     Pose<double> velocity = Pose<double>::Zero(vertices.cols());
//     Pose<double> rb1_pose_t1 = Pose<double>::Eye(vertices.cols());
//     Pose<double> rb2_pose_t1 = Pose<double>::Eye(vertices.cols());

//     Eigen::MatrixXd expected(4, 2);

//     vertices << -0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, 0.5;
//     edges << 0, 1, 1, 2, 2, 3, 3, 0;

//     SECTION("Translation Case")
//     {
//         rb1_pose_t1.position << 0.5, 0.5;
//         rb2_pose_t1.position << 1.0, 1.0;

//         // expected displacements
//         expected.resize(8, 2);
//         expected.block(0, 0, 4, 2) =
//             rb1_pose_t1.position.transpose().replicate(4, 1);
//         expected.block(4, 0, 4, 2) =
//             rb2_pose_t1.position.transpose().replicate(4, 1);
//     }

//     SECTION("90 Deg Rotation Case")
//     {
//         rb1_pose_t1.rotation << 0.5 * igl::PI;
//         rb2_pose_t1.rotation << igl::PI;
//         expected.resize(8, 2);
//         expected.block(0, 0, 4, 2) << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;
//         expected.block(4, 0, 4, 2) << 1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0,
//             -1.0;
//     }

//     std::vector<RigidBody> rbs;
//     rbs.push_back(simple_rigid_body(vertices, edges, velocity));
//     rbs.push_back(simple_rigid_body(vertices, edges, velocity));
//     RigidBodyAssembler assembler;
//     assembler.init(rbs);

//     Poses<double> poses = { { rb1_pose_t1, rb2_pose_t1 } };

//     /// compute displacements between current and given positions
//     /// TODO: update test to not need displacements
//     Eigen::MatrixXd actual =
//         assembler.world_vertices(poses) - assembler.world_vertices();
//     CHECK((expected - actual).squaredNorm() < 1E-6);
// }


TEST_CASE("Rigid Body System Transform 3D", "[RB][RB-System][RB-System-transform]")
{
    double width = 100; //mm-x
    double depth = 50; //mm-x
    double height = 25; //mm-x
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;
    rectangular_prism_3D(width, depth, height, vertices, edges, faces);
    Pose<double> velocity = Pose<double>::Zero(vertices.cols());
    Pose<double> rb1_pose_t1 = Pose<double>::Eye(vertices.cols());
    Pose<double> rb2_pose_t1 = Pose<double>::Eye(vertices.cols());

    int nverts = vertices.rows();
    int dim = vertices.cols();

    Eigen::MatrixXd expected(nverts, dim);

    SECTION("Translation Case")
    {
        rb1_pose_t1.position << 10, 10, 10;
        rb2_pose_t1.position << -20, -20, -20;

        // expected vertices displacements
        expected.resize(2 * nverts, dim);
        expected.block(0, 0, nverts, dim) =
            rb1_pose_t1.position.transpose().replicate(nverts, 1);
        expected.block(nverts, 0, nverts, dim) =
            rb2_pose_t1.position.transpose().replicate(nverts, 1);
    }

    SECTION("90 Deg Rotation Case")
    {
        Eigen::Vector3d axis = {1.0, 0.0, 0.0};
        double rb1_angle = 0.5 * igl::PI;
        double rb2_angle = igl::PI;

        // Set new pose of rbs
        rb1_pose_t1.set_rotation(rb1_angle * axis);
        rb2_pose_t1.set_rotation(rb2_angle * axis);

        // expected vertices displacements
        expected.resize(2 * nverts, dim);

        auto rb1_R = Eigen::AngleAxisd(rb1_angle, axis).toRotationMatrix();
        auto rb2_R = Eigen::AngleAxisd(rb2_angle, axis).toRotationMatrix();
        expected.block(0, 0, nverts, dim) = vertices * rb1_R.transpose() - vertices;
        expected.block(nverts, 0, nverts, dim) = vertices * rb2_R.transpose() - vertices;
    }

    std::vector<RigidBody> rbs;
    rbs.push_back(simple_rigid_body_3D(vertices, edges, faces, velocity));
    rbs.push_back(simple_rigid_body_3D(vertices, edges, faces, velocity));
    RigidBodyAssembler assembler;
    assembler.init(rbs);

    Poses<double> poses = { { rb1_pose_t1, rb2_pose_t1 } };

    /// compute displacements between current and given positions
    Eigen::MatrixXd actual =
        assembler.world_vertices(poses) - assembler.world_vertices();

    // Uncomment to help with debugging
    // std::cout << "Actual Displacments: \n" << actual << std::endl;
    // std::cout << "Expected Displacments: \n" << expected << std::endl;

    CHECK((expected - actual).squaredNorm() < 1E-6);
}


TEST_CASE("Rigid Body System World Vertices Jacobian", "[RB][RB-System][ABD][Derivatives]")
{
    double width = 100; //mm-x
    double depth = 50; //mm-x
    double height = 25; //mm-x
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;
    rectangular_prism_3D(width, depth, height, vertices, edges, faces);
    Pose<double> velocity = Pose<double>::Zero(vertices.cols());

    int nverts = vertices.rows();
    int dim = vertices.cols();

    // Setup assembler
    std::vector<RigidBody> rbs;
    rbs.push_back(simple_rigid_body_3D(vertices, edges, faces, velocity));
    rbs.push_back(simple_rigid_body_3D(vertices, edges, faces, velocity));
    RigidBodyAssembler assembler;
    assembler.init(rbs);

    // Set poses
    Eigen::Vector3d axis_1 = {3.0, 4.0, 1.0};
    axis_1.normalize();
    double rb1_angle = 0.5 * igl::PI;
    VectorMax3d pos_1(3);
    pos_1 << 10.0, 8.0, 5.0;

    Eigen::Vector3d axis_2 = {0.0, 1.2, 1.0};
    axis_2.normalize();
    double rb2_angle = igl::PI;
    VectorMax3d pos_2(3);
    pos_2 << 2.0, 4.0, 2.0;

    PosesD poses = {PoseD::FromRigidPose(pos_1, axis_1 * rb1_angle), PoseD::FromRigidPose(pos_2, axis_2 * rb2_angle)};

    // Get global jacobian and world vertices from assembler
    Eigen::MatrixXd jac(2 * nverts * dim, PoseD::dim_to_ndof(dim));
    Eigen::MatrixXd V = assembler.world_vertices_diff(poses, jac, true /*compute_jac*/);

    // Get expected jacobian and world vertices
    Eigen::MatrixXd expected_jac(2 * nverts * dim, PoseD::dim_to_ndof(dim));
    rbs[0].world_vertices_jacobian(0, expected_jac);
    rbs[1].world_vertices_jacobian(nverts, expected_jac);

    Eigen::MatrixXd expected_V(2 * nverts, dim);
    expected_V.block(0, 0, nverts, dim) = rbs[0].world_vertices(poses[0]);
    expected_V.block(nverts, 0, nverts, dim) = rbs[1].world_vertices(poses[1]);

    // Uncomment to help with debugging
    // std::cout << "Jacobian: \n" << jac << std::endl;
    // std::cout << "Expected Jacobian: \n" << expected_jac << std::endl;
    // std::cout << "V: \n" << V << std::endl;
    // std::cout << "Expected V: \n" << expected_V << std::endl;

    CHECK((expected_jac - jac).squaredNorm() < 1E-6);
    CHECK((expected_V - V).squaredNorm() < 1E-6);
}
