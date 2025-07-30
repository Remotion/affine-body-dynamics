#include <catch2/catch.hpp>

#include <iostream>
#include <Eigen/Geometry>
#include <igl/edges.h>
#include <igl/PI.h>
#include <problems/linear_constraint_handler.hpp>
#include <utils/row_echelon.hpp>

using namespace ipc;
using namespace ipc::rigid;


//------------------
// Helpers

RigidBody rigid_body_3D(
    Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, Eigen::MatrixXi& faces, Pose<double> initial_pose)
{
    static int id = 0;
    int ndof = Pose<double>::dim_to_ndof(vertices.cols());
    return RigidBody(
        vertices, edges, faces,
        /*pose=*/initial_pose,
        /*velcoity=*/Pose<double>::Zero(vertices.cols()),
        /*force=*/Pose<double>::Zero(vertices.cols()),
        /*density=*/1.0,
        /*oriented=*/false,
        /*group=*/id++);
}

void rectangular_3D(double width, double depth, double height, Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, Eigen::MatrixXi& faces){
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
// Each row of Rect dimensions specifies: width, depth, height of the rectangular prism
RigidBodyAssembler rectangular_bodies_assembly(Eigen::MatrixXd rect_dimensions){

    auto create_body = [](double width, double depth, double height){
        Eigen::MatrixXd verts;
        Eigen::MatrixXi edges;
        Eigen::MatrixXi faces;
        rectangular_3D(width, depth, height, verts, edges, faces);
        return rigid_body_3D(verts, edges, faces, PoseD::Eye(verts.cols()));
    };

    std::vector<RigidBody> rbs;
    for (int ri = 0; ri < rect_dimensions.rows(); ri++){
        rbs.push_back(create_body(rect_dimensions(ri,0), rect_dimensions(ri,1), rect_dimensions(ri,2)));
    }

    RigidBodyAssembler bodies;
    bodies.init(rbs);

    return bodies;
}

//------------------

//------------------
// Tests

TEST_CASE("Test pinned constraint", "[constraints][abd][assembler]")
{
    Eigen::MatrixXd rects_dims;
    rects_dims.resize(2, 3);
                /* width, depth, height */
    rects_dims <<   20.0, 40.0, 80.0,
                    40.0, 40.0, 40.0;

    RigidBodyAssembler bodies = rectangular_bodies_assembly(rects_dims);
    
    // Check that material coordinates wrt to input coordinates are as expected
    CHECK(bodies[0].pose.position.isZero());

    MatrixMax3d expected_R0(3,3);
    expected_R0 <<   0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0;

    CHECK(bodies[0].R0.isApprox(expected_R0));

    // Move the body before initializing constraint
    VectorMax3d disp(3);
    disp << 1.0, 2.0, 10.0;

    bodies[0].pose.position += disp;

    // Create constraint
    VectorMax3d body_input_point(3);
    body_input_point << 10.0, 1.0, 2.0;

    PinWorldConstraint pin_cons(bodies, 0 /*body_id*/, body_input_point);
    
    // Checking world point calculated correctly
    CHECK(pin_cons.world_point.isApprox(body_input_point + disp));

    // Constraint matrix and bcs
    Eigen::MatrixXd constraint_mat(bodies.num_bodies() * bodies.dim(), bodies.num_bodies() * PoseD::dim_to_ndof(bodies.dim()));
    constraint_mat.setZero();
    Eigen::VectorXd bcs(bodies.num_bodies() * bodies.dim());
    bcs.setZero();

    pin_cons.constraint_matrix(constraint_mat, bodies.dim(), bodies.dim());
    pin_cons.boundary_conditions(bcs, bodies.dim(), bodies.dim());

    // Uncomment to help with debugging
    // std::cout << "Constraint Matrix:\n" << constraint_mat << std::endl;
    // std::cout << "Boundary Conditions:\n" << bcs << std::endl;

    int dim = bodies.dim();
    int ndof = PoseD::dim_to_ndof(dim);
    CHECK(constraint_mat.middleRows(0, dim).isZero());
    CHECK(constraint_mat.middleCols(ndof, ndof).isZero());
    CHECK(constraint_mat.block(dim, 0, dim, ndof).isApprox(RigidBody::J_matrix(pin_cons.body_point)));

    CHECK(bcs.head(dim).isZero());
    CHECK(bcs.tail(dim).isApprox(pin_cons.world_point));
}

TEST_CASE("Test pin joint constraint", "[constraints][abd][assembler]")
{
    Eigen::MatrixXd rects_dims;
    rects_dims.resize(2, 3);
                /* width, depth, height */
    rects_dims <<   20.0, 40.0, 80.0,
                    40.0, 40.0, 40.0;

    RigidBodyAssembler bodies = rectangular_bodies_assembly(rects_dims);
    
    // Check that material coordinates wrt to input coordinates are as expected
    CHECK(bodies[0].pose.position.isZero());

    MatrixMax3d expected_R0(3,3);
    expected_R0 <<   0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0;

    CHECK(bodies[0].R0.isApprox(expected_R0));
    CHECK(bodies[1].R0.isIdentity());

    // Move the body before initializing constraint
    VectorMax3d disp(3);
    disp << 1.0, 2.0, 10.0;

    bodies[0].pose.position += disp;

    // Create pin joint constraint
    VectorMax3d bodyA_input_point(3);
    bodyA_input_point << 10.0, 1.0, 2.0;

    PinJointConstraint pinj_cons(bodies, 0 /*bodyA_id*/, 1 /*bodyB_id*/, bodyA_input_point);
    
    // Checking body B point calculated correctly
    CHECK(pinj_cons.bodyB_point.isApprox(bodyA_input_point + disp));

    // Constraint matrix and bcs
    Eigen::MatrixXd constraint_mat(bodies.num_bodies() * bodies.dim(), bodies.num_bodies() * PoseD::dim_to_ndof(bodies.dim()));
    constraint_mat.setZero();
    Eigen::VectorXd bcs(bodies.num_bodies() * bodies.dim());
    bcs.setZero();

    pinj_cons.constraint_matrix(constraint_mat, bodies.dim(), bodies.dim());
    pinj_cons.boundary_conditions(bcs, bodies.dim(), bodies.dim());

    // Uncomment to help with debugging
    // std::cout << "Constraint Matrix:\n" << constraint_mat << std::endl;
    // std::cout << "Boundary Conditions:\n" << bcs << std::endl;

    int dim = bodies.dim();
    int ndof = PoseD::dim_to_ndof(dim);
    CHECK(constraint_mat.middleRows(0, dim).isZero());
    CHECK(constraint_mat.block(dim, 0, dim, ndof).isApprox(RigidBody::J_matrix(pinj_cons.bodyA_point)));
    CHECK(constraint_mat.block(dim, ndof, dim, ndof).isApprox(-RigidBody::J_matrix(pinj_cons.bodyB_point)));

    CHECK(bcs.isZero());
}

TEST_CASE("Test creating constraints from JSON", "[constraints][abd]")
{
    // Create an assembly
    Eigen::MatrixXd rects_dims;
    rects_dims.resize(2, 3);
                /* width, depth, height */
    rects_dims <<   20.0, 40.0, 80.0,
                    40.0, 40.0, 40.0;

    RigidBodyAssembler bodies = rectangular_bodies_assembly(rects_dims);

    // The input json
    using namespace nlohmann;
    auto j = R"({
                    "linear_constraints": [
                        {
                            "type": "pin_world",
                            "body_name": "AffineBody",
                            "body_point": [0.0, 0.0, 1.0]
                        },
                        {
                            "type": "pin_joint",
                            "bodyA_name": "AffineBody",
                            "bodyA_point": [1.0, 3.0, 0.0],
                            "bodyB_name": "AffineBody1"
                        }
                    ]
                })"_json;

    // Create the constraint handler and pass json
    LinearConstraintHandler cons_handler;
    cons_handler.settings(j["linear_constraints"], bodies);
    cons_handler.assemble_constraints();

    // Check that we got the right types
    LinearConstraint& pinw = cons_handler[0];
    PinWorldConstraint* ptr_pinw = dynamic_cast<PinWorldConstraint*>(&pinw);
    CHECK(ptr_pinw != nullptr);

    LinearConstraint& pinj = cons_handler[1];
    PinJointConstraint* ptr_pinj = dynamic_cast<PinJointConstraint*>(&pinj);
    CHECK(ptr_pinj != nullptr);

    // Check the values are correct
    CHECK(ptr_pinw->body_id == 0);
    CHECK(ptr_pinj->bodyA_id == 0);
    CHECK(ptr_pinj->bodyB_id == 1);
    
    cons_handler.init();

    // Check the assembled constraint matrix and bcs are correct
    const Eigen::MatrixXd& con_mat = cons_handler.constraint_matrix();
    const Eigen::VectorXd& con_bcs = cons_handler.boundary_conditions();

    // Uncomment to help with debugging
    // std::cout << "Constraint matrix: \n" << con_mat << std::endl;
    // std::cout << "Constraint BCs: \n" << con_bcs << std::endl;

    // Pin World
    int ndof = PoseD::dim_to_ndof(3);
    CHECK(con_mat.block(0, 0, pinw.dim(), ndof).isApprox(RigidBody::J_matrix(ptr_pinw->body_point)));
    CHECK(con_mat.block(0, ndof, pinw.dim(), ndof).isZero());

    // Pin World
    CHECK(con_mat.block(pinw.dim(), 0, pinw.dim(), ndof).isApprox(RigidBody::J_matrix(ptr_pinj->bodyA_point)));
    CHECK(con_mat.block(pinw.dim(), ndof, pinw.dim(), ndof).isApprox(-RigidBody::J_matrix(ptr_pinj->bodyB_point)));
}

//------------------