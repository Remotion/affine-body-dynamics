// Test the mass utilities.

#include <catch2/catch.hpp>

#include <iostream>
#include <physics/mass.hpp>

using namespace ipc;
using namespace ipc::rigid;

TEST_CASE("Center of Mass", "[physics][mass]")
{
    int dim = GENERATE(2);
    int num_vertices = 6 * GENERATE(10, 50, 100);

    // Test vertices positions for given rb position
    Eigen::MatrixXd vertices = Eigen::MatrixXd::Random(num_vertices, dim);

    Eigen::MatrixXi edges =
        Eigen::VectorXd::LinSpaced(num_vertices, 0, num_vertices - 1)
            .cast<int>();
    edges = Eigen::MatrixXi(
        Eigen::Map<Eigen::MatrixXi>(edges.data(), num_vertices / dim, dim));

    double total_mass1;
    VectorMax3d center_of_mass1;
    MatrixMax3d moment_of_inertia1;
    compute_mass_properties(
        vertices, edges, total_mass1, center_of_mass1, moment_of_inertia1);

    double total_mass2 = compute_total_mass(vertices, edges);
    VectorMax3d center_of_mass2 = compute_center_of_mass(vertices, edges);
    MatrixMax3d moment_of_inertia2 = compute_moment_of_inertia(vertices, edges);

    CAPTURE(dim, num_vertices);
    CHECK(total_mass1 == Approx(total_mass2));
    CHECK((center_of_mass1 - center_of_mass2).norm() < 1e-12);
    CHECK((moment_of_inertia1 - moment_of_inertia2).norm() < 1e-12);
}

TEST_CASE("Moments of Density 2D", "[physics][mass]")
{
    int dim = GENERATE(2);
    int num_vertices = 6 * GENERATE(10, 50, 100);

    // Test vertices positions for given rb position
    Eigen::MatrixXd vertices = Eigen::MatrixXd::Random(num_vertices, dim);

    Eigen::MatrixXi edges =
        Eigen::VectorXd::LinSpaced(num_vertices, 0, num_vertices - 1)
            .cast<int>();
    edges = Eigen::MatrixXi(
        Eigen::Map<Eigen::MatrixXi>(edges.data(), num_vertices / dim, dim));

    MatrixMax3d moment_of_inertia = compute_moment_of_inertia(vertices, edges);
    double moment_of_inertia_expected = moment_of_inertia(2,2);

    // std::cout << "Moment of inertia:\n" << moment_of_inertia << std::endl;

    Eigen::Vector3d moments;
    compute_mass_moments_2D(vertices, edges, moments);
    double moment_of_inertia_calculated = moments[0] + moments[1];

    // std::cout << "Integrals of (x^2 dm, y^2 dm, xy dm):\n" << moments << std::endl;


    CHECK(moment_of_inertia_calculated == Approx(moment_of_inertia_expected).margin(1e-7));
}

TEST_CASE("Principal Axes 2D", "[physics][mass]")
{
    // Check principal axes of rectangle with code vs analytically calculated
    // Note: we assume density = 1.0 (so we ignore it in calcs)
    double height = 100; //mm - z
    double width = 50; //mm - x
    int num_vertices = 4; // Rectangular prism
    int dim = 2;

    double dh = height/2.0;
    double dw = width/2.0;

    Eigen::MatrixXd vertices(num_vertices, dim);
    vertices << -dw,  dh, 
                 dw,  dh,
                 dw, -dh,
                -dw, -dh;

    Eigen::MatrixXi edges(4, 2);
    edges <<    0, 1,
                1, 2,
                2, 3,
                3, 0;

    // 1. Rotate Body
    // Now we rotate the body to see if the calculated principal axes and principal moments
    // are correct
    double angle = 1.6 * EIGEN_PI;
    Eigen::Rotation2Dd rot(angle);
    Eigen::Matrix2d R_applied = rot.toRotationMatrix();

    // std::cout << "Body rotated using rotation matrix: \n" << R_applied << std::endl;

    // Rotating body (right multiply by R.T because vertices are row vectors)
    vertices = vertices * R_applied.transpose();


    // 2. Calculate inertia tensor then get principal axes
    double mass;
    VectorMax3d com;
    MatrixMax3d I;
    compute_mass_properties_2D(vertices, edges, mass, com, I);
    MatrixMax3d R0;
    compute_principle_axes_2D(I, R0);

    // std::cout << "Principal axes basis to world basis rotation R0: \n" << R0 << std::endl;

    // The principal axes should be parallel or anti-parallel to the initial rotation matrix we applied R
    Eigen::Matrix2d dot_matrix = R0.transpose() * R_applied;
    CHECK(dot_matrix.isUnitary());
    CHECK((dot_matrix.determinant() - 1.0) <= 1.0e-9);

    // Check all components are either 1, -1 or 0
    auto almostEqual = [](double a, double b, double epsilon = 1e-9) {
        return std::abs(a - b) <= epsilon;
    };
    Eigen::Matrix2<bool> bool_mat = dot_matrix.unaryExpr([&](double v) { return almostEqual(v,1.0) || almostEqual(v,0.0) || almostEqual(v,-1.0); });
    CHECK(bool_mat.all());

    // 3. Recalculate moments of density in the principal frame
    // By rotating vertices into the principal frame
    vertices = vertices * R0;

    // Then calculating moments in principal frame
    Eigen::Matrix<double, 3, 1> princ_moments;
    compute_mass_moments_2D(vertices, edges, princ_moments);

    // std::cout << "Principal moments: \n" << princ_moments << std::endl;

    // Check that in the principal frame integral of xy dm == 0
    CHECK(abs(princ_moments[2]) < 1E-9);
}

TEST_CASE("Moments of Density 3D", "[physics][mass]")
{
    // Check moments of mass of rectangular prism with code vs analytically calculated
    double height = 100; //mm - z
    double width = 50; //mm - x
    double depth = 25; //mm - y
    int num_vertices = 8; // Rectangular prism
    int dim = 3;

    double dh = height/2.0;
    double dw = width/2.0;
    double dd = depth/2.0;

    Eigen::Vector3d center{0.0, 0.0, 0.0};
    Eigen::MatrixXd vertices(num_vertices, dim);
    vertices << -dw, -dd,  dh, 
                -dw,  dd,  dh,
                 dw,  dd,  dh,
                 dw, -dd,  dh,
                -dw, -dd, -dh,
                -dw,  dd, -dh,
                 dw,  dd, -dh,
                 dw, -dd, -dh;

    Eigen::MatrixXi faces(2 * 6, dim);
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

    Eigen::Matrix<double, 6, 1> moments;
    compute_mass_moments_3D(vertices, faces, moments);

    // std::cout << "Moments calculated: " << moments << std::endl;

    // Moments expected
    double v = height * width * depth;
    double pxx = 1.0/12.0 * v * width * width;
    double pyy = 1.0/12.0 * v * depth * depth;
    double pzz = 1.0/12.0 * v * height * height;
    double pxy = 0.0;
    double pyz = 0.0;
    double pzx = 0.0;

    Eigen::Matrix<double, 6, 1> moments_expected;
    moments_expected << pxx, pyy, pzz, pxy, pyz, pzx;

    // std::cout << "Moments calculated: " << moments_expected << std::endl;

    CHECK((moments - moments_expected).norm() < 1e-7);
}

TEST_CASE("Principal Axes 3D", "[physics][mass]")
{
    // Check moments of mass of rectangular prism with code vs analytically calculated
    // Note: we assume density = 1.0 (so we ignore it in calcs)
    double height = 100; //mm - z
    double width = 50; //mm - x
    double depth = 25; //mm - y
    int num_vertices = 8; // Rectangular prism
    int dim = 3;

    double dh = height/2.0;
    double dw = width/2.0;
    double dd = depth/2.0;

    Eigen::Vector3d center{0.0, 0.0, 0.0};
    Eigen::MatrixXd vertices(num_vertices, dim);
    vertices << -dw, -dd,  dh, 
                -dw,  dd,  dh,
                 dw,  dd,  dh,
                 dw, -dd,  dh,
                -dw, -dd, -dh,
                -dw,  dd, -dh,
                 dw,  dd, -dh,
                 dw, -dd, -dh;

    Eigen::MatrixXi faces(2 * 6, dim);
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


    // 1. Rotate Body
    // Now we rotate the body to see if the calculated principal axes and principal moments
    // are correct
    Eigen::Vector3d axis;
    axis << 1.0, 2.0, 3.0;
    axis.normalize();
    double angle = 1.6 * EIGEN_PI;

    Eigen::AngleAxisd rot_vec = Eigen::AngleAxisd(angle, axis);
    Eigen::Matrix3d R_applied = rot_vec.toRotationMatrix();

    // std::cout << "Body rotated using rotation matrix: \n" << R_applied << std::endl;

    // Rotating body (right multiply by R.T because vertices are row vectors)
    vertices = vertices * R_applied.transpose();


    // 2. Calculate principal axes and their respective moments of inertia

    // Calculate moments in world coordinates first
    Eigen::Matrix<double, 6, 1> wm0;
    compute_mass_moments_3D(vertices, faces, wm0);

    // Construct inertia tensor
    Eigen::Matrix3d I;
    I  <<    wm0[1] + wm0[2],   // Ixx: y^2 + z^2
            -wm0[3],            // -Ixy: -xy
            -wm0[5],            // -Ixz: -xz

            -wm0[3],            // -Iyx: -xy
             wm0[0] + wm0[2],   // Iyy: x^2 + z^2
            -wm0[4],            // -Iyz: -yz

            -wm0[5],            // -Izx: -xz
            -wm0[4],            // -Izy: -yz
             wm0[0] + wm0[1];   // Izz: x^2 + y^2

    MatrixMax3d R0;
    compute_principle_axes_3D(I, R0);

    // std::cout << "Principal axes basis to world basis rotation R0: \n" << R0 << std::endl;

    // The principal axes should be parallel or anti-parallel to the initial rotation matrix we applied R
    Eigen::Matrix3d dot_matrix = R0.transpose() * R_applied;
    CHECK(dot_matrix.isUnitary());
    CHECK((dot_matrix.determinant() - 1.0) <= 1.0e-9);

    // Check all components are either 1, -1 or 0
    auto almostEqual = [](double a, double b, double epsilon = 1e-9) {
        return std::abs(a - b) <= epsilon;
    };
    Eigen::Matrix3<bool> bool_mat = dot_matrix.unaryExpr([&](double v) { return almostEqual(v,1.0) || almostEqual(v,0.0) || almostEqual(v,-1.0); });
    CHECK(bool_mat.all());

    // 3. Recalculate moments of density in the principal frame
    // By rotating vertices into the principal frame
    vertices = vertices * R0;

    // Then calculating moments in principal frame
    Eigen::Matrix<double, 6, 1> princ_moments;
    compute_mass_moments_3D(vertices, faces, princ_moments);

    // std::cout << "Principal moments: \n" << princ_moments << std::endl;


    // Moments expected
    double v = height * width * depth;
    double pxx = 1.0/12.0 * v * width * width;
    double pyy = 1.0/12.0 * v * depth * depth;
    double pzz = 1.0/12.0 * v * height * height;
    double pxy = 0.0;
    double pyz = 0.0;
    double pzx = 0.0;

    Eigen::Matrix<double, 6, 1> moments_expected;
    moments_expected << pxx, pyy, pzz, pxy, pyz, pzx;


    // std::cout << "Analytical moments: \n" << moments_expected << std::endl;

    std::sort(princ_moments.data(), princ_moments.data() + princ_moments.size());
    std::sort(moments_expected.data(), moments_expected.data() + moments_expected.size());

    CHECK((princ_moments - moments_expected).norm() < 1e-6);
}

// TODO: Add 3D RB test
