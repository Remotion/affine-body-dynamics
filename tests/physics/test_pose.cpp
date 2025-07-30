// Test Pose
#include <catch2/catch.hpp>

#include <Eigen/Geometry>
#include <igl/PI.h>

#include <autodiff/autodiff_types.hpp>
#include <interval/interval.hpp>
#include <physics/pose.hpp>

TEST_CASE("Poses to dofs", "[physics][pose]")
{
    using namespace ipc::rigid;
    int dim = GENERATE(2, 3);
    int num_bodies = GENERATE(0, 1, 2, 3, 10, 1000);
    Eigen::VectorXd dofs =
        Eigen::VectorXd::Random(num_bodies * Pose<double>::dim_to_ndof(dim));
    Poses<double> poses = Pose<double>::dofs_to_poses(dofs, dim);
    Eigen::VectorXd returned_dofs = Pose<double>::poses_to_dofs(poses);
    CHECK((dofs - returned_dofs).squaredNorm() == Approx(0));
}

TEST_CASE("Cast poses", "[physics][pose]")
{
    using namespace ipc::rigid;
    int dim = GENERATE(2, 3);
    int num_bodies = GENERATE(0, 1, 2, 3, 10, 1000);

    Eigen::VectorXf dof =
        Eigen::VectorXf::Random(num_bodies * Pose<float>::dim_to_ndof(dim));
    Poses<float> expected_posesf = Pose<float>::dofs_to_poses(dof, dim);

    Poses<float> actual_posesf = cast<float>(cast<double>(expected_posesf));

    CHECK(
        (Pose<float>::poses_to_dofs(expected_posesf)
         - Pose<float>::poses_to_dofs(actual_posesf))
            .squaredNorm()
        == Approx(0.0));
}

TEST_CASE("SE(3) ↦ SO(3)", "[physics][pose]")
{
    using namespace ipc::rigid;
    double angle;
    Eigen::Vector3d axis;

    SECTION("zero")
    {
        angle = 0;
        axis = Eigen::Vector3d::Random();
    }
    SECTION("random")
    {
        angle = GENERATE(take(100, random(0.0, 2 * igl::PI)));
        axis = Eigen::Vector3d::Random();
    }
    axis.normalize();

    Pose<double> p = Pose<double>::FromRigidPose(Eigen::Vector3d::Zero(), angle * axis);
    Eigen::Matrix3d R_actual = p.construct_transformation_matrix();
    Eigen::Matrix3d R_expected =
        Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    CHECK((R_actual - R_expected).norm() == Approx(0).margin(1e-12));
}

TEST_CASE("Pose set to zero", "[physics][pose]")
{
    using namespace ipc::rigid;
    double angle;
    Eigen::Vector3d axis;

    angle = GENERATE(take(100, random(0.0, 2 * igl::PI)));
    axis = Eigen::Vector3d::Random();
    axis.normalize();

    Pose<double> p = Pose<double>::FromRigidPose(Eigen::Vector3d::Zero(), angle * axis);
    p.set_eye();

    Eigen::Matrix3d R_actual = p.construct_transformation_matrix();
    Eigen::Matrix3d R_expected = Eigen::Matrix3d::Identity();

    Eigen::Vector3d pos_actual = p.position;
    Eigen::Vector3d pos_expected = Eigen::Vector3d::Zero();

    CHECK((R_actual - R_expected).norm() == Approx(0).margin(1e-12));
    CHECK((pos_actual - pos_expected).norm() == Approx(0).margin(1e-12));
}

TEST_CASE("Interval SE(3) ↦ SO(3)", "[!benchmark][physics][pose]")
{
    using namespace ipc::rigid;
    double angle;
    Eigen::Vector3d axis;

    SECTION("zero")
    {
        angle = 0;
        axis = Eigen::Vector3d::Random();
    }
    SECTION("random")
    {
        angle = GENERATE(take(1, random(0.0, 2 * igl::PI)));
        axis = Eigen::Vector3d::Random();
    }
    axis.normalize();

    Pose<double> p = Pose<double>::Eye(3);
    p.set_rotation(angle * axis);
    BENCHMARK("Double SE(3) ↦ SO(3)")
    {
        Eigen::Matrix3d R = p.construct_transformation_matrix();
        return R;
    };
    Pose<ipc::rigid::Interval> pI = p.cast<ipc::rigid::Interval>();
    BENCHMARK("Interval SE(3) ↦ SO(3)")
    {
        Matrix3I R = pI.construct_transformation_matrix();
        return R;
    };
}
