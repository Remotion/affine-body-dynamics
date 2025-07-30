// Test the rigid body class.

#include <array>
#include <iomanip>
#include <iostream>

#include <catch2/catch.hpp>

#include <igl/PI.h>
#include <igl/edges.h>

#include <physics/rigid_body.hpp>

// ---------------------------------------------------
// Tests
// ---------------------------------------------------
using namespace ipc;
using namespace ipc::rigid;

RigidBody
simple(Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, Pose<double> velocity)
{
    static int id = 0;
    int ndof = Pose<double>::dim_to_ndof(vertices.cols());
    return RigidBody(
        vertices, edges, Pose<double>::Eye(vertices.cols()), velocity,
        /*force=*/Pose<double>::Zero(vertices.cols()), /*density=*/1.0,
        /*oriented=*/false,
        /*group=*/id++);
}

RigidBody simple_3D(
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

void rectangular_prism(double width, double depth, double height, Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, Eigen::MatrixXi& faces){
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

template<typename DerivedA, typename DerivedB>
bool allclose(const Eigen::DenseBase<DerivedA>& a,
              const Eigen::DenseBase<DerivedB>& b,
              const typename DerivedA::RealScalar& rtol
                  = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
              const typename DerivedA::RealScalar& atol
                  = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs()
          <= (atol + rtol * b.derived().array().abs())).all();
}

// TODO: 2D Rigid Body not yet implemented
TEST_CASE("2D Rigid Body Transform", "[RB][RB-transform]")
{
    // Test vertices positions for given rb position
    Eigen::MatrixXd vertices_t0(4, 2);
    Eigen::MatrixXi edges(4, 2);
    Pose<double> velocity = Pose<double>::Zero(vertices_t0.cols());
    Pose<double> rb_step = Pose<double>::Eye(vertices_t0.cols());

    Eigen::MatrixXd vertices_step(4, 2), expected(4, 2);

    vertices_t0 << -0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, 0.5;
    edges << 0, 1, 1, 2, 2, 3, 3, 0;

    SECTION("Translation Case")
    {
        rb_step.position << 0.5, 0.5;
        rb_step.set_rotation(0.0);
        vertices_step = rb_step.position.transpose().replicate(4, 1);
    }

    SECTION("90 Deg Rotation Case")
    {
        rb_step.position << 0.0, 0.0;
        rb_step.set_rotation(0.5 * igl::PI);
        vertices_step << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;
    }

    SECTION("Translation and Rotation Case")
    {
        rb_step.position << 0.5, 0.5;
        rb_step.set_rotation(0.5 * igl::PI);
        vertices_step << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;
        vertices_step += rb_step.position.transpose().replicate(4, 1);
    }
    expected = vertices_t0 + vertices_step;

    auto rb = simple(vertices_t0, edges, velocity);
    Pose<double> gamma_t1(
        rb_step.position + rb.pose.position,
        rb_step.transform * rb.pose.transform);
    Eigen::MatrixXd actual = rb.world_vertices<double>(gamma_t1);
    CHECK((expected - actual).squaredNorm() < 1E-6);
}

TEST_CASE("3D Rigid Body Transform", "[RB][RB-transform]")
{
    double width = 100; //mm-x
    double depth = 50; //mm-x
    double height = 25; //mm-x
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;
    rectangular_prism(width, depth, height, vertices, edges, faces);
    Pose<double> velocity = Pose<double>::Zero(vertices.cols());
    Pose<double> rb_pose_new = Pose<double>::Eye(vertices.cols());

    int nverts = vertices.rows();
    int dim = vertices.cols();

    Eigen::MatrixXd expected(nverts, dim);

    SECTION("Translation Case")
    {
        Eigen::Vector3d displacement = {10, 10, 10};

        // Set new rb pose
        rb_pose_new.position += displacement;

        // expected vertices displacements
        expected = displacement.transpose().replicate(nverts, 1);
    }

    SECTION("90 Deg Rotation Case")
    {
        Eigen::Vector3d axis = {1.0, 0.0, 0.0};
        double angle = 0.5 * igl::PI;

        // Set new rb pose
        auto R = construct_rotation_matrix(VectorMax3d(angle * axis));
        rb_pose_new.transform = R * rb_pose_new.transform;

        // expected vertices displacements
        auto expected_R = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        expected = vertices * expected_R.transpose() - vertices;
    }

    SECTION("Translation and Rotation Case")
    {
        Eigen::Vector3d displacement = {10, 10, 10};

        Eigen::Vector3d axis = {1.0, 0.0, 0.0};
        double angle = 0.5 * igl::PI;

        // Set new rb pose
        auto R = construct_rotation_matrix(VectorMax3d(angle * axis));
        rb_pose_new.transform = R * rb_pose_new.transform;
        rb_pose_new.position += displacement;

        // expected vertices displacements
        auto expected_R = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        expected = (vertices * R.transpose()).rowwise() + displacement.transpose() - vertices;
    }

    RigidBody rb = simple_3D(vertices, edges, faces, velocity);


    /// compute displacements between current and start vertices positions
    Eigen::MatrixXd actual = rb.world_vertices(rb_pose_new) - rb.world_vertices();
    CHECK((expected - actual).squaredNorm() < 1E-6);
}

TEST_CASE("J Matrix In-Place", "[RB][ABD][Mass-Properties]")
{
    Eigen::MatrixXd mat(6,12);
    mat.setZero();

    VectorMax3d x_bar;
    x_bar.resize(3);
    x_bar << 1.0, 2.0, 3.0;

    RigidBody::J_matrix(x_bar, mat.block<3,12>(0,0));

    x_bar << 4.0, 5.0, 6.0;
    RigidBody::J_matrix(x_bar, mat.block<3,12>(3,0));

    Eigen::MatrixXd J = RigidBody::J_matrix(x_bar);

    Eigen::MatrixXd mat_expected(6,12);
    mat_expected << 1.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0,
                    1.0, 0.0, 0.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0, 6.0; 

    // std::cout << mat;
    
    CHECK((mat - mat_expected).squaredNorm() < 1E-6);
}


TEST_CASE("Generalized Mass Matrix", "[RB][ABD]")
{
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;
    double w = 100;
    double d = 50;
    double h = 25;
    double density = 1.0;
    rectangular_prism(w, d, h, vertices, edges, faces);

    double volume = w * d * h;
    double mass = density * volume;
    
    // Rotating to check code is able to find principal axes
    double angle = 1.6 * EIGEN_PI;
    Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();
    Eigen::Matrix3d R_applied = Eigen::AngleAxisd(angle, axis).toRotationMatrix();;
    vertices = vertices * R_applied.transpose();


    // Create rigid body
    RigidBody rb(
        vertices, edges, faces, Pose<double>::Eye(vertices.cols()),
        Pose<double>::Zero(vertices.cols()),
        /*force=*/Pose<double>::Zero(vertices.cols()), /*density=*/1.0,
        /*oriented=*/false,
        /*group=*/0);


    Eigen::VectorXd expected_diagonals(12);
    expected_diagonals <<             mass,           mass,           mass, 1.04166667e+08,
                            2.60416667e+07, 6.51041667e+06, 1.04166667e+08, 2.60416667e+07,
                            6.51041667e+06, 1.04166667e+08, 2.60416667e+07, 6.51041667e+06;
    

    auto actual_diagonals = rb.mass_matrix.diagonal();

    // Uncomment to help debugging
    // std::cout << "Applied rotation: \n" << R_applied << std::endl;
    // std::cout << "Principal axes: \n" << rb.R0 << std::endl;
    // std::cout << "Generalized mass matrix \n" << Eigen::MatrixXd(rb.mass_matrix) << std::endl;
    // std::cout << "Expected diagonals: \n" << expected_diagonals << std::endl;
    // std::cout << "Actual diagonals: \n" << actual_diagonals << std::endl;


    CHECK(allclose(expected_diagonals, actual_diagonals, /*atol*/1e-6, /*rtol*/1e-6));
}

TEST_CASE("World Vertices Jacobian", "[RB][ABD][Derivatives]")
{
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;
    rectangular_prism(100, 50, 25, vertices, edges, faces);
    
    // Create rigid body
    RigidBody rb(
        vertices, edges, faces, Pose<double>::Eye(vertices.cols()),
        Pose<double>::Zero(vertices.cols()),
        /*force=*/Pose<double>::Zero(vertices.cols()), /*density=*/1.0,
        /*oriented=*/false,
        /*group=*/0);

    // World vertices (making it bigger than necessary, rb should only fill the corresponding entries)
    Eigen::MatrixXd V;
    V.resize(vertices.rows() + 4, vertices.cols());
    V.setZero();

    // Jacobian (making it bigger than necessary, rb should only fill the corresponding entries)
    Eigen::MatrixXd jac;
    jac.resize((vertices.rows() + 1) * vertices.cols(), PoseD::dim_to_ndof(vertices.cols()));
    jac.setZero();

    long rb_v0_i  = 1;
    //@javidf: not sure if I should use PoseD::Zero or PoseD::Eye below
    rb.world_vertices_diff(PoseD::Zero(vertices.cols()), rb_v0_i, V, jac);

    // Check split functions have the same result
    Eigen::MatrixXd V2;
    V2.resize(vertices.rows() + 4, vertices.cols());
    V2.setZero();
    //@javidf: not sure if I should use PoseD::Zero or PoseD::Eye below
    rb.world_vertices(PoseD::Zero(vertices.cols()), rb_v0_i, V2);
    CHECK((V2 - V).squaredNorm() < 1E-6);

    Eigen::MatrixXd jac2;
    jac2.resize((vertices.rows() + 1) * vertices.cols(), PoseD::dim_to_ndof(vertices.cols()));
    jac2.setZero();
    rb.world_vertices_jacobian(rb_v0_i, jac2);
    CHECK((jac2 - jac).squaredNorm() < 1E-6);

    auto& v = vertices;

    Eigen::MatrixXd jac_expected((v.rows() + 1) * v.cols(), 12);
    jac_expected << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    1.0, 0.0, 0.0, v(0,0), v(0,1), v(0,2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, v(0,0), v(0,1), v(0,2), 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, v(0,0), v(0,1), v(0,2),
                    1.0, 0.0, 0.0, v(1,0), v(1,1), v(1,2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, v(1,0), v(1,1), v(1,2), 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, v(1,0), v(1,1), v(1,2),
                    1.0, 0.0, 0.0, v(2,0), v(2,1), v(2,2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, v(2,0), v(2,1), v(2,2), 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, v(2,0), v(2,1), v(2,2),
                    1.0, 0.0, 0.0, v(3,0), v(3,1), v(3,2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, v(3,0), v(3,1), v(3,2), 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, v(3,0), v(3,1), v(3,2),
                    1.0, 0.0, 0.0, v(4,0), v(4,1), v(4,2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, v(4,0), v(4,1), v(4,2), 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, v(4,0), v(4,1), v(4,2),
                    1.0, 0.0, 0.0, v(5,0), v(5,1), v(5,2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, v(5,0), v(5,1), v(5,2), 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, v(5,0), v(5,1), v(5,2),
                    1.0, 0.0, 0.0, v(6,0), v(6,1), v(6,2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, v(6,0), v(6,1), v(6,2), 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, v(6,0), v(6,1), v(6,2),
                    1.0, 0.0, 0.0, v(7,0), v(7,1), v(7,2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, v(7,0), v(7,1), v(7,2), 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, v(7,0), v(7,1), v(7,2);

    // Uncomment to help with debugging
    // std::cout << "World Verts Jac:\n";
    // std::cout << jac << "\n";
    // std::cout << "Expected:\n";
    // std::cout << jac_expected << "\n";
    
    CHECK((jac - jac_expected).squaredNorm() < 1E-6);
}

RigidBody create_Rbody(
    const Eigen::MatrixXd& vertices,
    const Eigen::MatrixXi& edges,
    const Eigen::MatrixXi& faces)
{
    static int id = 0;
    int dim = vertices.cols();
    Pose<double> pose = Pose<double>::Eye(dim);
    RigidBody rb = RigidBody(
        vertices, edges, faces, pose,
        /*velocity=*/Pose<double>::Zero(pose.dim()),
        /*force=*/Pose<double>::Zero(pose.dim()),
        /*denisty=*/1.0,
        /*oriented=*/false,
        /*group_id=*/id++);
    rb.vertices = vertices; // Cancel out the inertial rotation for testing
    rb.pose.position.setZero();
    rb.pose.transform.setZero();
    return rb;
}

RigidBody
create_Rbody(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& edges)
{
    return create_Rbody(vertices, edges, Eigen::MatrixXi());
}

// TODO: Add checks
TEST_CASE("1D Bodies in 3D", "[RB][1D]")
{
    int dim = 3;
    int ndof = Pose<double>::dim_to_ndof(dim);
    int pos_ndof = Pose<double>::dim_to_pos_ndof(dim);
    int rot_ndof = Pose<double>::dim_to_pos_ndof(dim);

    Eigen::MatrixXd bodyA_vertices(2, dim);
    bodyA_vertices.row(0) << -1, 0, 0;
    bodyA_vertices.row(1) << 1, 0, 0;
    Eigen::MatrixXd bodyB_vertices(2, dim);
    bodyB_vertices.row(0) << 0, 0, -1;
    bodyB_vertices.row(1) << 0, 0, 1;

    Eigen::MatrixXi bodyA_edges(1, 2);
    bodyA_edges.row(0) << 0, 1;
    Eigen::MatrixXi bodyB_edges(1, 2);
    bodyB_edges.row(0) << 0, 1;

    RigidBody bodyA = create_Rbody(bodyA_vertices, bodyA_edges);
    RigidBody bodyB = create_Rbody(bodyB_vertices, bodyB_edges);
}