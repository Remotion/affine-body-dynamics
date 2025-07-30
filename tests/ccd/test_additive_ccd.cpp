#include <catch2/catch.hpp>

#include <string>

#include <ghc/fs_std.hpp> // filesystem
#include <igl/edges.h>
#include <igl/read_triangle_mesh.h>

#include <ccd/ccd.hpp>
#include <logger.hpp>


// #include <ghc/fs_std.hpp> // filesystem
#include <igl/PI.h>
// #include <igl/edges.h>

#include <ipc/distance/edge_edge.hpp>
// #include <ccd.hpp>
#include <ccd/impact.hpp>
// #include <ccd/piecewise_linear/time_of_impact.hpp>
#include <ccd/rigid/time_of_impact.hpp>
#include <constants.hpp>
#include <io/serialize_json.hpp>
#include <physics/rigid_body_assembler.hpp>

using namespace ipc;
using namespace ipc::rigid;
const double TESTING_TOI_TOLERANCE = 1e-6;

RigidBody create_bodyA(
    const Eigen::MatrixXd& vertices,
    const Eigen::MatrixXi& edges,
    const Eigen::MatrixXi& faces)
{
    static int id = 0;
    int dim = vertices.cols();
    Pose<double> pose = Pose<double>::Eye(dim);
    // pose.set_zero();
    RigidBody rb = RigidBody(
        vertices, edges, faces, pose,
        /*velocity=*/Pose<double>::Zero(dim),
        /*force=*/Pose<double>::Zero(dim),
        /*denisty=*/1.0,
        /*oriented=*/false,
        /*group_id=*/id++);
    rb.vertices = vertices; // Cancel out the inertial rotation for testing
    rb.pose.position.setZero();
    rb.pose.transform.setZero();
    return rb;
}

RigidBody
create_bodyA(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& edges)
{
    return create_bodyA(vertices, edges, Eigen::MatrixXi());
}

TEST_CASE("Additive edge-edge CCD", "[accd][edge_edge]")
{
    int dim = 3;
    int ndof = Pose<double>::dim_to_ndof(dim);
    int pos_ndof = Pose<double>::dim_to_pos_ndof(dim);
    int rot_ndof = Pose<double>::dim_to_trans_ndof(dim);

    Eigen::MatrixXd bodyA_vertices(2, dim);
    bodyA_vertices.row(0) << -1, 0, 0;
    bodyA_vertices.row(1) << 1, 0, 0;
    Eigen::MatrixXd bodyB_vertices(2, dim);
    bodyB_vertices.row(0) << 0, 1, -2;
    bodyB_vertices.row(1) << 0, 1, 2;

    Eigen::MatrixXi bodyA_edges(1, 2);
    bodyA_edges.row(0) << 0, 1;
    Eigen::MatrixXi bodyB_edges(1, 2);
    bodyB_edges.row(0) << 0, 1;
    // std::cout << "Body A in edge-edge \n";
    RigidBody bodyA = create_bodyA(bodyA_vertices, bodyA_edges);
    // std::cout << "Body B in edge-edge \n";
    RigidBody bodyB = create_bodyA(bodyB_vertices, bodyB_edges);


    RigidBodyAssembler assembler;
    assembler.init(std::vector<RigidBody>({bodyA, bodyB}));
    PosesD poses_t0(assembler.num_bodies());
    PosesD poses_t1(assembler.num_bodies());
    poses_t0[0] = Pose<double>::Eye(dim);
    poses_t0[1] = Pose<double>::Eye(dim);
    poses_t1[0] = Pose<double>::Eye(dim);
    poses_t1[1] = Pose<double>::Eye(dim);

    poses_t1[0].position << 0.0, 0.75, -0.0;
    poses_t1[1].position << 0.0, -0.75, 0.0;

    TrajectoryType traj = TrajectoryType::ACCD;
    Impacts impacts;
    DetectionMethod mthd = DetectionMethod::IAABB;

    int collision_types = 6;
    // std::cout << "collision type: " << collision_types << " num bodies : " << assembler.num_bodies() << std::endl;
    detect_collisions(assembler, poses_t0, poses_t1, collision_types, impacts, mthd, traj);
    double expected_toi = 2.0/3.0;
    double expected_impacted_alpha = 0.5;
    double expected_impacting_alpha = 0.5;
    // std::cout << "impact time: " << impacts.ee_impacts.at(0).time << " impacted alpha : " << impacts.ee_impacts.at(0).impacted_alpha << " impacting alpha : " << impacts.ee_impacts.at(0).impacting_alpha << std::endl;
    CHECK(expected_toi == Approx(impacts.ee_impacts.at(0).time).margin(0.01));
    CHECK(expected_impacted_alpha == Approx(impacts.ee_impacts.at(0).impacted_alpha).margin(0.01));
    CHECK(expected_impacting_alpha == Approx(impacts.ee_impacts.at(0).impacting_alpha).margin(0.01));
}

TEST_CASE("Additive vertex-face CCD", "[accd][vertex_face]")
{
    int dim = 3;
    int ndof = Pose<double>::dim_to_ndof(dim);
    int pos_ndof = Pose<double>::dim_to_pos_ndof(dim);
    int rot_ndof = Pose<double>::dim_to_trans_ndof(dim);

    Eigen::MatrixXd bodyA_vertices(4, dim);
    bodyA_vertices.row(0) << -1, 0, 0;
    bodyA_vertices.row(1) << 1, 0, 0;
    bodyA_vertices.row(2) << 0, 1, 0;
    bodyA_vertices.row(3) << 0, 0, -1;
    // double y = GENERATE(1.0 + 1e-8, 1.0, 1.0 - 1e-8, 0.5, 1e-8, 0.0, 1e-8);
    Eigen::MatrixXd bodyB_vertices(2, dim);
    bodyB_vertices.row(0) << -0.5, 0.5, 1;
    bodyB_vertices.row(1) << 0, 0.5, 2.0;

    Eigen::MatrixXi bodyA_faces(4, 3);
    bodyA_faces.row(0) << 0, 1, 2;
    bodyA_faces.row(1) << 3, 1, 0;
    bodyA_faces.row(2) << 0, 2, 3;
    bodyA_faces.row(3) << 1, 3, 2;
    // Eigen::MatrixXi bodyB_faces(1, 3);
    // bodyB_faces.row(0) << 0, 1, 2;

    Eigen::MatrixXi bodyA_edges;
    igl::edges(bodyA_faces, bodyA_edges);
    Eigen::MatrixXi bodyB_edges(1, 2);
    bodyB_edges.row(0) << 0, 1;
    // std::cout << "Body A in face-vertex \n";
    RigidBody bodyA = create_bodyA(bodyA_vertices, bodyA_edges, bodyA_faces);
    // std::cout << "Body B in face-vertex \n";
    RigidBody bodyB = create_bodyA(bodyB_vertices, bodyB_edges);

    RigidBodyAssembler assembler;
    assembler.init(std::vector<RigidBody>({bodyA, bodyB}));
    PosesD poses_t0(assembler.num_bodies());
    PosesD poses_t1(assembler.num_bodies());
    poses_t0[0] = Pose<double>::Eye(dim);
    poses_t0[1] = Pose<double>::Eye(dim);
    poses_t1[0] = Pose<double>::Eye(dim);
    poses_t1[1] = Pose<double>::Eye(dim);

    poses_t1[0].position << 0.0, 0.0, 1.0;
    poses_t1[1].position << 0.0, 0.0, -1.0;

    TrajectoryType traj = TrajectoryType::ACCD;
    Impacts impacts;
    DetectionMethod mthd = DetectionMethod::IAABB;

    int collision_types = 6;
    // std::cout << "collision type: " << collision_types << " num bodies : " << assembler.num_bodies() << std::endl;
    detect_collisions(assembler, poses_t0, poses_t1, collision_types, impacts, mthd, traj);
    double expected_toi = 0.5;
    double expected_u = 0.5;
    double expected_v = 0.0;
    // std::cout << "barycentric coords: " << impacts.fv_impacts.at(0).u << impacts.fv_impacts.at(0).v << std::endl;
    CHECK(expected_toi == Approx(impacts.fv_impacts.at(0).time).margin(0.01));
    CHECK(expected_u == Approx(impacts.fv_impacts.at(0).u).margin(0.01));
    CHECK(expected_v == Approx(impacts.fv_impacts.at(0).v).margin(0.01));
}
