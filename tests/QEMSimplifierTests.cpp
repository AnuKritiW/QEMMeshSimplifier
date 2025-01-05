#include <gtest/gtest.h>

#include "parser.h"
#include "QEMSimplifier.h"
#include "QEMSimplifierUtils.h"

constexpr float g_tolerance = 1e-6;

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/*
  * Tests for the Parser class
*/

TEST(Parser, LoadValidMesh)
{
    TriMesh mesh;
    std::string validFile = "../object-files/icosahedron.obj";
    ASSERT_TRUE(Parser::loadMesh(validFile, mesh));
    EXPECT_EQ(mesh.n_vertices(), 12);
    EXPECT_EQ(mesh.n_faces(), 20);
}

TEST(Parser, FailToLoadInvalidMesh)
{
    TriMesh mesh;
    std::string invalidFile = "../nonexistent.obj";
    ASSERT_FALSE(Parser::loadMesh(invalidFile, mesh));
}

/*
 * Tests for QEMSimplifier
*/

TEST(QEMSimplifierUtils, InitializePropertyInt)
{
    TriMesh mesh;

    // Add vertices to the mesh
    auto v1 = mesh.add_vertex(TriMesh::Point(0.0, 0.0, 0.0));
    auto v2 = mesh.add_vertex(TriMesh::Point(1.0, 0.0, 0.0));
    auto v3 = mesh.add_vertex(TriMesh::Point(0.0, 1.0, 0.0));

    // Define a property and initialize it
    OpenMesh::VPropHandleT<int> intProperty;
    QEMSimplifierUtils::initializeProperty(mesh, intProperty, "v:intProperty", 42);

    // Ensure the property is initialized and default values are set
    ASSERT_TRUE(mesh.get_property_handle(intProperty, "v:intProperty"));
    EXPECT_EQ(mesh.property(intProperty, v1), 42);
    EXPECT_EQ(mesh.property(intProperty, v2), 42);
    EXPECT_EQ(mesh.property(intProperty, v3), 42);
}

TEST(QEMSimplifierUtils, InitializePropertyMatrix)
{
    TriMesh mesh;

    // Add vertices to the mesh
    auto v1 = mesh.add_vertex(TriMesh::Point(0.0, 0.0, 0.0));
    auto v2 = mesh.add_vertex(TriMesh::Point(1.0, 0.0, 0.0));
    auto v3 = mesh.add_vertex(TriMesh::Point(0.0, 1.0, 0.0));

    // Define another property for testing a different type
    OpenMesh::VPropHandleT<Eigen::Matrix4d> matrixProperty;
    QMatrix defaultMatrix = Eigen::Matrix4d::Identity();
    QEMSimplifierUtils::initializeProperty(mesh, matrixProperty, "v:matrixProperty", defaultMatrix);

    // Ensure the property is initialized and default values are set
    ASSERT_TRUE(mesh.get_property_handle(matrixProperty, "v:matrixProperty"));
    EXPECT_TRUE(mesh.property(matrixProperty, v1).isApprox(defaultMatrix));
    EXPECT_TRUE(mesh.property(matrixProperty, v2).isApprox(defaultMatrix));
    EXPECT_TRUE(mesh.property(matrixProperty, v3).isApprox(defaultMatrix));
}

TEST(QEMSimplifierUtils, MergeVertexPropertiesInt)
{
    TriMesh mesh;

    // Add vertices to the mesh
    auto v1 = mesh.add_vertex(TriMesh::Point(0.0, 0.0, 0.0));
    auto v2 = mesh.add_vertex(TriMesh::Point(1.0, 0.0, 0.0));

    // Define a property and initialize it
    OpenMesh::VPropHandleT<int> intProperty;
    QEMSimplifierUtils::initializeProperty(mesh, intProperty, "v:intProperty", 0);

    // Set custom values for the vertices
    mesh.property(intProperty, v1) = 10;
    mesh.property(intProperty, v2) = 20;

    // Merge the properties
    int mergedValue = QEMSimplifierUtils::mergeVertexProperties(mesh, v1, v2, intProperty);
    EXPECT_EQ(mergedValue, 30); // 10 + 20
}
TEST(QEMSimplifierUtils, MergeVertexPropertiesMatrix)
{
    TriMesh mesh;

    // Add vertices to the mesh
    auto v1 = mesh.add_vertex(TriMesh::Point(0.0, 0.0, 0.0));
    auto v2 = mesh.add_vertex(TriMesh::Point(1.0, 0.0, 0.0));

    // Define another property for testing Eigen::Matrix4d
    OpenMesh::VPropHandleT<Eigen::Matrix4d> matrixProperty;
    Eigen::Matrix4d defaultMatrix = Eigen::Matrix4d::Zero();
    QEMSimplifierUtils::initializeProperty(mesh, matrixProperty, "v:matrixProperty", defaultMatrix);

    // Set custom values for the vertices
    QMatrix matrix1 = Eigen::Matrix4d::Identity();
    QMatrix matrix2 = Eigen::Matrix4d::Constant(1.0);
    mesh.property(matrixProperty, v1) = matrix1;
    mesh.property(matrixProperty, v2) = matrix2;

    // Merge the properties
    QMatrix mergedMatrix = QEMSimplifierUtils::mergeVertexProperties(mesh, v1, v2, matrixProperty);

    // Verify the merged matrix
    QMatrix expectedMatrix = matrix1 + matrix2;
    EXPECT_TRUE(mergedMatrix.isApprox(expectedMatrix));
}

// computePlaneEquation
TEST(QEMSimplifierUtils, computePlaneEquation)
{
    /*
    Input: A triangle lying in the XY plane ((0,0,0), (1,0,0), (0,1,0))
    Expected Output: The plane equation should be (z = 0)
                     So, a = 0, b = 0, c = 1, d = 0
    */

    // Triangle vertices in the XY plane
    TriMesh::Point p0(0, 0, 0);
    TriMesh::Point p1(1, 0, 0);
    TriMesh::Point p2(0, 1, 0);

    Eigen::Vector4d plane = QEMSimplifierUtils::computePlaneEquation(p0, p1, p2);

    EXPECT_FLOAT_EQ(plane[0], 0.0); // a = 0
    EXPECT_FLOAT_EQ(plane[1], 0.0); // b = 0
    EXPECT_FLOAT_EQ(plane[2], 1.0); // c = 1
    EXPECT_FLOAT_EQ(plane[3], 0.0); // d = 0
}

// computePlaneEquation
TEST(QEMSimplifierUtils, computePlaneEquationTranslatedZ)
{
    /*
    Input: A triangle parallel to the XY plane but shifted in the Z direction ((0,0,1), (1,0,1), (0,1,1))
    Expected Output: The plane equation should be (z − 1 = 0)
                     So, a = 0, b = 0, c = 1, d = -1
    */

    // Triangle vertices in the XY plane
    TriMesh::Point p0(0, 0, 1);
    TriMesh::Point p1(1, 0, 1);
    TriMesh::Point p2(0, 1, 1);

    Eigen::Vector4d plane = QEMSimplifierUtils::computePlaneEquation(p0, p1, p2);

    EXPECT_FLOAT_EQ(plane[0], 0.0);     // a = 0
    EXPECT_FLOAT_EQ(plane[1], 0.0);     // b = 0
    EXPECT_FLOAT_EQ(plane[2], 1.0);     //  c = 1
    EXPECT_FLOAT_EQ(plane[3], -1.0);    // d = -1
}

// computePlaneEquation
TEST(QEMSimplifierUtils, computePlaneEquationRandomTri)
{
    /*
    Input: A triangle that is not aligned with any axis
    Expected Output: Commented inline below
    */

    // Triangle vertices in the XY plane
    TriMesh::Point p0(0, 0, 0);
    TriMesh::Point p1(1, 0, 0);
    TriMesh::Point p2(0, 1, 1);

    /**
     * v1 = p1 - p0 = (1, 0, 0)
     * v2 = p2 - p0 = (0, 1, 1)
     * n = cross(v1, v2) = (0, -1, 1)
     * ||n|| = sqrt(2)
     * Normalize n = (0/||n||, -1/||n||, 1/||n||) = (0,−0.7071,0.7071)
     * Sub into plane eqn to get d = 0
     */


    Eigen::Vector4d plane = QEMSimplifierUtils::computePlaneEquation(p0, p1, p2);

    const float c = (1.0 / std::sqrt(2.0));
    const float b = -c;

    // Normal vector should be (0.7071, 0.7071, 0.0) normalized
    // EXPECT_NEAR to prevent floating-point precision errors
    EXPECT_NEAR(plane[0], 0.0,  g_tolerance); // a ~ 0
    EXPECT_NEAR(plane[1], b,    g_tolerance); // b ~ -0.7071
    EXPECT_NEAR(plane[2], c,    g_tolerance); // c ~ 0.7071
    EXPECT_NEAR(plane[3], 0.0,  g_tolerance); // d = 0
}

// computeFaceQuadric
TEST(QEMSimplifierUtils, computeFaceQuadricKnownTriangle)
{
    TriMesh mesh;

    // Create a triangle in the XY plane
    TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
    TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
    TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0, 1, 0));
    TriMesh::FaceHandle fh = mesh.add_face(v0, v1, v2);

    QMatrix quadric = QEMSimplifierUtils::computeFaceQuadric(mesh, fh);

    QMatrix expectedQuadric;
    expectedQuadric.setZero();
    expectedQuadric(2, 2) = 1.0; // Precomputed for z=0

    EXPECT_TRUE(quadric.isApprox(expectedQuadric, 1e-6));
}

TEST(QEMSimplifierUtils, computeFaceQuadricArbitraryTriangle)
{
    TriMesh mesh;

    // Create a triangle on an arbitrary plane
    TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
    TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
    TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0, 1, 1));
    TriMesh::FaceHandle fh = mesh.add_face(v0, v1, v2);

    QMatrix quadric = QEMSimplifierUtils::computeFaceQuadric(mesh, fh);

    // Plane equation: computed dynamically
    Eigen::Vector4d plane = QEMSimplifierUtils::computePlaneEquation(
        mesh.point(v0), mesh.point(v1), mesh.point(v2));

    float a = plane[0];
    float b = plane[1];
    float c = plane[2];
    float d = plane[3];

    /** Kp = outer product of plane eq [a,b,c,d]
     * [ a^2  ab   ac   ad
     *   ab   b^2  bc   bd
     *   ac   bc   c^2  cd
     *   ad   bd   cd   d^2 ]
     */

    QMatrix Kp;
    Kp.setZero();
    Kp(0,0) = a*a; Kp(0,1) = a*b; Kp(0,2) = a*c; Kp(0,3) = a*d;
    Kp(1,0) = b*a; Kp(1,1) = b*b; Kp(1,2) = b*c; Kp(1,3) = b*d;
    Kp(2,0) = c*a; Kp(2,1) = c*b; Kp(2,2) = c*c; Kp(2,3) = c*d;
    Kp(3,0) = d*a; Kp(3,1) = d*b; Kp(3,2) = d*c; Kp(3,3) = d*d;

    // needs precision tolerance
    EXPECT_TRUE(quadric.isApprox(Kp, g_tolerance));
}

// computeQuadrics
TEST(QEMSimplifier, computeQuadrics)
{
    QEMSimplifier qem;
    TriMesh mesh;

    // Create a simple triangle mesh with two faces
    TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
    TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
    TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0, 1, 0));
    TriMesh::VertexHandle v3 = mesh.add_vertex(TriMesh::Point(1, 1, 0));
    mesh.add_face({v0, v1, v2});
    mesh.add_face({v1, v3, v2});

    // Initialize property handle for quadrics
    static OpenMesh::VPropHandleT<QMatrix> vQuadric;
    if (!mesh.get_property_handle(vQuadric, "v:quadric"))
    {
        mesh.add_property(vQuadric, "v:quadric");
    }

    qem.computeQuadrics(mesh);

    // Validate vertex quadrics
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        QMatrix quadric = mesh.property(vQuadric, *v_it);

        EXPECT_FALSE(quadric.isZero()); // Ensure quadric is updated
    }
}

TEST(QEMSimplifier, computeQuadricsMeshFile)
{
    QEMSimplifier qem;
    TriMesh mesh;

    ASSERT_TRUE(Parser::loadMesh("../object-files/icosahedron.obj", mesh));

    // Initialize property handle for quadrics
    static OpenMesh::VPropHandleT<QMatrix> vQuadric;
    if (!mesh.get_property_handle(vQuadric, "v:quadric"))
    {
        mesh.add_property(vQuadric, "v:quadric");
    }

    qem.computeQuadrics(mesh);

    // Validate vertex quadrics
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        QMatrix quadric = mesh.property(vQuadric, *v_it);

        EXPECT_FALSE(quadric.isZero()); // Ensure quadric is updated
    }
}

TEST(QEMSimplifier, computeEdgeCollapseCostFromObj)
{
    QEMSimplifier qem;
    TriMesh mesh;

    std::string objFilePath = "../object-files/gourd.obj";
    ASSERT_TRUE(Parser::loadMesh(objFilePath, mesh));

    qem.computeQuadrics(mesh);

    // Validate quadrics are non-zero
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, *v_it).isZero());
    }

    // Iterate through edges and compute collapse cost
    for (auto e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it)
    {
        Eigen::Vector3d optPos;
        float cost = QEMSimplifierUtils::computeEdgeCollapseCost(mesh, *e_it, optPos);

        // Retrieve vertex handles for the edge
        auto he = mesh.halfedge_handle(*e_it, 0);
        TriMesh::VertexHandle v0 = mesh.to_vertex_handle(he);
        TriMesh::VertexHandle v1 = mesh.from_vertex_handle(he);

        QMatrix Q = QEMSimplifierUtils::getVertexQuadric(mesh, v0) + QEMSimplifierUtils::getVertexQuadric(mesh, v1);

        // Note that there is duplication of logic here from the function, so the calculations are not fully independent
        // What's being tested here is that we are not entering the fallback logic in this test
        Eigen::Matrix3d A = Q.topLeftCorner<3, 3>();
        Eigen::Vector3d b(-Q(0, 3), -Q(1, 3), -Q(2, 3));
        float det = A.determinant();

        // Ensure determinant is non-zero and we are NOT entering the fallback case
        EXPECT_GT(det, 1e-12);

        // Validate the computed optimal position (vOpt)
        if (det > 1e-12) // Ensure we're not in the fallback condition
        {
            Eigen::Vector3d expectedOptPos = A.inverse() * b;
            EXPECT_NEAR(optPos[0], expectedOptPos[0], g_tolerance);
            EXPECT_NEAR(optPos[1], expectedOptPos[1], g_tolerance);
            EXPECT_NEAR(optPos[2], expectedOptPos[2], g_tolerance);
        }

        // Ensure collapse cost is greater than zero
        EXPECT_GT(cost, 0.0f);
    }
}

TEST(QEMSimplifier, ComputeEdgeCollapseCostFallback) // Hits fallback for A
{
    QEMSimplifier qem;
    TriMesh mesh;

    TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
    TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
    TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0, 1, 0)); // Non-collinear vertex

    mesh.add_face({v0, v1, v2}); // Create a valid triangular face to generate edges
    qem.computeQuadrics(mesh);

    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, *v_it).isZero());
    }

    auto edge = *mesh.edges_begin(); // Get the first edge in the mesh
    Eigen::Vector3d optPos;
    float cost = QEMSimplifierUtils::computeEdgeCollapseCost(mesh, edge, optPos);

    EXPECT_NEAR(optPos[0], 0.5, g_tolerance); // Midpoint x-coordinate
    EXPECT_NEAR(optPos[1], 0.0, g_tolerance); // Midpoint y-coordinate
    EXPECT_NEAR(optPos[2], 0.0, g_tolerance); // Midpoint z-coordinate

    // The cost is 0 because the geo and quadrics align such that no error is incurred at the optPos
    // This happens because the triangle lies flat in the XY-plane, and the edge collapse does not deviate from this plane
    EXPECT_EQ(cost, 0.0);
}

TEST(QEMSimplifier, collapseEdge)
{
    QEMSimplifier qem;
    TriMesh mesh;

    // TODO: move into a function
    // Example inspired from Figure 1 in the paper "Surface Simplification Using Quadric Error Metrics"
    TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(0.0, 0.0, 0.0));
    TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(1.0, 0.0, 0.0));
    TriMesh::VertexHandle v3 = mesh.add_vertex(TriMesh::Point(-0.5, 1.0, 0.0));
    TriMesh::VertexHandle v4 = mesh.add_vertex(TriMesh::Point(0.5, 1.5, 0.0));
    TriMesh::VertexHandle v5 = mesh.add_vertex(TriMesh::Point(1.5, 1.0, 0.0));
    TriMesh::VertexHandle v6 = mesh.add_vertex(TriMesh::Point(-1.0, 0.0, 0.0));
    TriMesh::VertexHandle v7 = mesh.add_vertex(TriMesh::Point(2.0, 0.0, 0.0));
    TriMesh::VertexHandle v8 = mesh.add_vertex(TriMesh::Point(-0.5, -1.0, 0.0));
    TriMesh::VertexHandle v9 = mesh.add_vertex(TriMesh::Point(0.5, -1.5, 0.0));
    TriMesh::VertexHandle v10 = mesh.add_vertex(TriMesh::Point(1.5, -1.0, 0.0));

    mesh.add_face({v1, v8, v9});
    mesh.add_face({v1, v6, v8});
    mesh.add_face({v1, v3, v6});
    mesh.add_face({v1, v4, v3});
    mesh.add_face({v1, v2, v4});
    mesh.add_face({v1, v9, v2});
    mesh.add_face({v2, v5, v4});
    mesh.add_face({v2, v7, v5});
    mesh.add_face({v2, v10, v7});
    mesh.add_face({v2, v9, v10});

    // Verify initial mesh structure
    EXPECT_EQ(mesh.n_vertices(), 10);
    EXPECT_EQ(mesh.n_faces(), 10);
    EXPECT_EQ(mesh.n_edges(), 19);

    qem.computeQuadrics(mesh);

    // Ensure quadrics are initialized
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v1).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v2).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v3).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v4).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v5).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v6).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v7).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v8).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v9).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v10).isZero());

    // Request necessary status flags
    // needed for is_collapse_ok()
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();

    // Find the edge between v1 and v2
    TriMesh::EdgeHandle edge;
    for (auto e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it)
    {
        auto he0 = mesh.halfedge_handle(*e_it, 0);
        auto he1 = mesh.halfedge_handle(*e_it, 1);

        if ((mesh.to_vertex_handle(he0) == v2 && mesh.from_vertex_handle(he0) == v1) ||
            (mesh.to_vertex_handle(he1) == v1 && mesh.from_vertex_handle(he1) == v2))
        {
            edge = *e_it;
            break;
        }
    }

    ASSERT_TRUE(edge.is_valid());

    // Check if the collapse is allowed
    TriMesh::HalfedgeHandle he0 = mesh.halfedge_handle(edge, 0);
    ASSERT_TRUE(mesh.is_collapse_ok(he0));

    TriMesh::VertexHandle vKeep = mesh.to_vertex_handle(he0); // By convention, keep "to_vertex"

    // Compute the new position for vKeep (midpoint of v1 and v2)
    Eigen::Vector3d newPos;
    TriMesh::Point p1 = mesh.point(v1);
    TriMesh::Point p2 = mesh.point(v2);
    newPos[0] = (p1[0] + p2[0]) / 2.0;
    newPos[1] = (p1[1] + p2[1]) / 2.0;
    newPos[2] = (p1[2] + p2[2]) / 2.0;

    TriMesh::VertexHandle vKeepArg;
    // Collapse the edge
    ASSERT_TRUE(qem.collapseEdge(mesh, edge, newPos, vKeepArg));

    // Remove deleted vertices, edges, and faces from the mesh
    mesh.garbage_collection();

    // Verify the state of the mesh after collapse
    EXPECT_EQ(mesh.n_vertices(), 9);
    EXPECT_EQ(mesh.n_faces(), 8);
    EXPECT_EQ(mesh.n_edges(), 16);

    // Verify the position of the new vertex
    TriMesh::Point vKeepPos = mesh.point(vKeep);

    EXPECT_FLOAT_EQ(vKeepPos[0], newPos[0]) << "x-coordinate of new vertex is incorrect.";
    EXPECT_FLOAT_EQ(vKeepPos[1], newPos[1]) << "y-coordinate of new vertex is incorrect.";
    EXPECT_FLOAT_EQ(vKeepPos[2], newPos[2]) << "z-coordinate of new vertex is incorrect.";
}

TEST(QEMSimplifier, collapseEdgeNotAllowed)
{
    QEMSimplifier qem;
    TriMesh mesh;

    // Create a simple triangle mesh
    TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0.0, 0.0, 0.0));
    TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1.0, 0.0, 0.0));
    TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0.5, 1.0, 0.0));
    mesh.add_face({v0, v1, v2});

    // Request necessary status flags
    // needed for is_collapse_ok()
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();

    qem.computeQuadrics(mesh);

    // Ensure quadrics are initialized
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v0).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v1).isZero());
    ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, v2).isZero());

    // Get the edge between v0 and v1
    TriMesh::EdgeHandle edge;
    for (auto he_it = mesh.voh_iter(v0); he_it.is_valid(); ++he_it)
    {
        if (mesh.to_vertex_handle(*he_it) == v1)
        {
            edge = mesh.edge_handle(*he_it);
            break;
        }
    }

    // New position for vKeep
    Eigen::Vector3d newPos(0.5, 0.0, 0.0); // Midpoint of v0 and v1

    TriMesh::VertexHandle vKeep;
    // Collapse the edge
    // Should fail because only a planar triangle has been passed
    ASSERT_FALSE(qem.collapseEdge(mesh, edge, newPos, vKeep));

    // Validate the mesh after collapse fails
    EXPECT_EQ(mesh.n_vertices(), 3);
    EXPECT_EQ(mesh.n_edges(), 3);
    EXPECT_EQ(mesh.n_faces(), 1);
}