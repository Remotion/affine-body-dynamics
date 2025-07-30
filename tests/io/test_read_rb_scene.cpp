#include <catch2/catch.hpp>

#include <iostream>

#include <io/read_rb_scene.hpp>
#include <physics/mass.hpp>
#include <utils/eigen_ext.hpp>

//-----------------
// Tests
//-----------------

TEST_CASE("Read 2D RB Scene", "[io][json][rigid-body]")
{
    std::vector<ipc::rigid::RigidBody> rbs;
    using namespace nlohmann;
    auto j = R"({"rigid_bodies": [
             {
               "vertices": [
                 [0, 0],
                 [2, 0],
                 [2, 3],
                 [5, 3],
                 [5, 5],
                 [0, 5]
               ],
               "edges": [
                 [0, 1],
                 [1, 2],
                 [2, 3],
                 [3, 4],
                 [4, 5],
                 [5, 0]
               ],
               "linear_velocity": [0.0, 0.0],
               "angular_velocity": [0.0]
             }
           ]})"_json;

    ipc::rigid::read_rb_scene(j, rbs);

    CHECK(rbs.size() == 1);

    Eigen::MatrixXd positions(6, 2);
    positions << 0, 0, 2, 0, 2, 3, 5, 3, 5, 5, 0, 5;

    // std::cout << "World vertices:\n" << rbs[0].world_vertices() << std::endl;

    CHECK(
        (positions - rbs[0].world_vertices()).squaredNorm()
        == Approx(0.0).margin(1e-12));

    Eigen::MatrixXi edges(6, 2);
    edges << 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 0;
    CHECK((edges - rbs[0].edges).squaredNorm() == Approx(0.0).margin(1e-12));

    Eigen::VectorXd velocity(6);
    velocity << 0, 0, 0, 0, 0, 0;

    // std::cout << "RB Velocity:\n" << rbs[0].velocity.dof() << std::endl;
    CHECK(
        (velocity - rbs[0].velocity.dof()).squaredNorm()
        == Approx(0.0).margin(1e-12));
}

// TODO: Test reading 3D RB scenes
