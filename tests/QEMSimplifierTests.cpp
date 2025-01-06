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
 * Tests for QEMSimplifierUtils
*/

// =============================================================================
//                             Initialize Property Tests
// =============================================================================

class QEMSimplifierUtilsTests : public ::testing::Test
{
protected:
    TriMesh mesh;

    void SetUp() override {
        mesh.add_vertex(TriMesh::Point(0.0, 0.0, 0.0));
        mesh.add_vertex(TriMesh::Point(1.0, 0.0, 0.0));
        mesh.add_vertex(TriMesh::Point(0.0, 1.0, 0.0));
    }
};

TEST_F(QEMSimplifierUtilsTests, InitializePropertyInt)
{
    // Define a property and initialize it
    OpenMesh::VPropHandleT<int> intProperty;
    QEMSimplifierUtils::initializeProperty(mesh, intProperty, "v:intProperty", 42);

    // Ensure the property is initialized and default values are set
    ASSERT_TRUE(mesh.get_property_handle(intProperty, "v:intProperty"));
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
        EXPECT_EQ(mesh.property(intProperty, *v_it), 42);
    }
}

TEST_F(QEMSimplifierUtilsTests, InitializePropertyMatrix)
{
    // Define another property for testing a different type
    OpenMesh::VPropHandleT<Eigen::Matrix4d> matrixProperty;
    QMatrix defaultMatrix = Eigen::Matrix4d::Identity();
    QEMSimplifierUtils::initializeProperty(mesh, matrixProperty, "v:matrixProperty", defaultMatrix);

    // Ensure the property is initialized and default values are set
    ASSERT_TRUE(mesh.get_property_handle(matrixProperty, "v:matrixProperty"));
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
        EXPECT_TRUE(mesh.property(matrixProperty, *v_it).isApprox(defaultMatrix));
    }
}

// =============================================================================
//                             Merge Property Tests
// =============================================================================

TEST_F(QEMSimplifierUtilsTests, MergeVertexPropertiesInt)
{
    // Define a property and initialize it
    OpenMesh::VPropHandleT<int> intProperty;
    QEMSimplifierUtils::initializeProperty(mesh, intProperty, "v:intProperty", 0);

    // Assign values to vertices
    auto v1 = *mesh.vertices_begin();
    auto v2 = *(++mesh.vertices_begin());

    // Set custom values for the vertices
    mesh.property(intProperty, v1) = 10;
    mesh.property(intProperty, v2) = 20;

    // Merge the properties
    int mergedValue = QEMSimplifierUtils::mergeVertexProperties(mesh, v1, v2, intProperty);
    EXPECT_EQ(mergedValue, 30); // 10 + 20
}

TEST_F(QEMSimplifierUtilsTests, MergeVertexPropertiesMatrix)
{
    // Define another property for testing Eigen::Matrix4d
    OpenMesh::VPropHandleT<Eigen::Matrix4d> matrixProperty;
    Eigen::Matrix4d defaultMatrix = Eigen::Matrix4d::Zero();
    QEMSimplifierUtils::initializeProperty(mesh, matrixProperty, "v:matrixProperty", defaultMatrix);

    // Assign values to vertices
    auto v1 = *mesh.vertices_begin();
    auto v2 = *(++mesh.vertices_begin());

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

// =============================================================================
//                             Compute Plane Equation Tests
// =============================================================================

class PlaneEquationTest : public ::testing::TestWithParam<std::tuple<TriMesh::Point, TriMesh::Point, TriMesh::Point, Eigen::Vector4d>> {};

TEST_P(PlaneEquationTest, computePlaneEquation)
{
    /*
    Input: A triangle lying in the XY plane ((0,0,0), (1,0,0), (0,1,0))
    Expected Output: The plane equation should be (z = 0)
                     So, a = 0, b = 0, c = 1, d = 0
    */

    // Triangle vertices in the XY plane
    auto [p0, p1, p2, expectedPlane] = PlaneEquationTest::GetParam();
    Eigen::Vector4d plane = QEMSimplifierUtils::computePlaneEquation(p0, p1, p2);

    EXPECT_FLOAT_EQ(plane[0], expectedPlane[0]); // a
    EXPECT_FLOAT_EQ(plane[1], expectedPlane[1]); // b
    EXPECT_FLOAT_EQ(plane[2], expectedPlane[2]); // c
    EXPECT_FLOAT_EQ(plane[3], expectedPlane[3]); // d
}

// Custom name generator for parameterized tests
std::string PlaneEquationTestName(const ::testing::TestParamInfo<std::tuple<TriMesh::Point, TriMesh::Point, TriMesh::Point, Eigen::Vector4d>>& info) {
    switch (info.index) {
        case 0: return "XYPlane";
        case 1: return "TranslatedZ";
        case 2: return "RandomTriangle";
        default: return "Unknown";
    }
}

// Instantiate test cases
INSTANTIATE_TEST_SUITE_P(
    PlaneEquationTests,
    PlaneEquationTest,
    ::testing::Values(
        std::make_tuple(
            TriMesh::Point(0, 0, 0), TriMesh::Point(1, 0, 0), TriMesh::Point(0, 1, 0),
            Eigen::Vector4d(0, 0, 1, 0)
        ),
        std::make_tuple(
            TriMesh::Point(0, 0, 1), TriMesh::Point(1, 0, 1), TriMesh::Point(0, 1, 1),
            Eigen::Vector4d(0, 0, 1, -1)
        ),
        std::make_tuple(
            TriMesh::Point(0, 0, 0), TriMesh::Point(1, 0, 0), TriMesh::Point(0, 1, 1),
            Eigen::Vector4d(0, -1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0), 0)
        )
    ),
    PlaneEquationTestName
);

// =============================================================================
//                             Compute Face Quadric Tests
// =============================================================================

class QEMSimplifierUtilsFaceQuadricTest : public ::testing::Test {
protected:
    TriMesh mesh;

    void createXYTriangle() {
        mesh.add_vertex(TriMesh::Point(0, 0, 0));
        mesh.add_vertex(TriMesh::Point(1, 0, 0));
        mesh.add_vertex(TriMesh::Point(0, 1, 0));
    }

    void createArbitraryTriangle() {
        mesh.add_vertex(TriMesh::Point(0, 0, 0));
        mesh.add_vertex(TriMesh::Point(1, 0, 0));
        mesh.add_vertex(TriMesh::Point(0, 1, 1));
    }
};

TEST_F(QEMSimplifierUtilsFaceQuadricTest, computeFaceQuadricKnownTriangle)
{
    // Create a triangle in the XY plane
    createXYTriangle();
    TriMesh::FaceHandle fh = mesh.add_face(mesh.vertex_handle(0), mesh.vertex_handle(1), mesh.vertex_handle(2));

    QMatrix quadric = QEMSimplifierUtils::computeFaceQuadric(mesh, fh);

    QMatrix expectedQuadric;
    expectedQuadric.setZero();
    expectedQuadric(2, 2) = 1.0; // Precomputed for z=0

    EXPECT_TRUE(quadric.isApprox(expectedQuadric, g_tolerance));
}

TEST_F(QEMSimplifierUtilsFaceQuadricTest, computeFaceQuadricArbitraryTriangle)
{
    // Create a triangle on an arbitrary plane
    createArbitraryTriangle();
    TriMesh::FaceHandle fh = mesh.add_face(mesh.vertex_handle(0), mesh.vertex_handle(1), mesh.vertex_handle(2));

    QMatrix quadric = QEMSimplifierUtils::computeFaceQuadric(mesh, fh);

    // Plane equation: computed dynamically
    Eigen::Vector4d plane = QEMSimplifierUtils::computePlaneEquation(mesh.point(mesh.vertex_handle(0)),
                                                                     mesh.point(mesh.vertex_handle(1)),
                                                                     mesh.point(mesh.vertex_handle(2)));

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

// =============================================================================
//                             Compute Quadrics Tests
// =============================================================================

class ComputeQuadricsTest : public ::testing::Test
{
protected:
    QEMSimplifier qem;
    TriMesh mesh;
    OpenMesh::VPropHandleT<QMatrix> vQuadric;

    void SetUp() override
    {
        // Initialize property handle for quadrics
        if (!mesh.get_property_handle(vQuadric, "v:quadric"))
        {
            mesh.add_property(vQuadric, "v:quadric");
        }
    }

    void createSimpleMesh()
    {
        mesh.add_vertex(TriMesh::Point(0, 0, 0));
        mesh.add_vertex(TriMesh::Point(1, 0, 0));
        mesh.add_vertex(TriMesh::Point(0, 1, 0));
        mesh.add_vertex(TriMesh::Point(1, 1, 0));
        mesh.add_face({mesh.vertex_handle(0), mesh.vertex_handle(1), mesh.vertex_handle(2)});
        mesh.add_face({mesh.vertex_handle(1), mesh.vertex_handle(3), mesh.vertex_handle(2)});
    }
};

TEST_F(ComputeQuadricsTest, computeQuadrics)
{
    createSimpleMesh();
    qem.computeQuadrics(mesh);

    // Validate vertex quadrics
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        QMatrix quadric = mesh.property(vQuadric, *v_it);
        EXPECT_FALSE(quadric.isZero()); // Ensure quadric is updated
    }
}

TEST_F(ComputeQuadricsTest, computeQuadricsMeshFile)
{
    ASSERT_TRUE(Parser::loadMesh("../object-files/icosahedron.obj", mesh));
    qem.computeQuadrics(mesh);

    // Validate vertex quadrics
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        QMatrix quadric = mesh.property(vQuadric, *v_it);

        EXPECT_FALSE(quadric.isZero()); // Ensure quadric is updated
    }
}

// =============================================================================
//                             Compute Edge Collapse Cost Tests
// =============================================================================

class ComputeEdgeCollapseCostTests : public ::testing::Test
{
protected:
    QEMSimplifier qem;
    TriMesh mesh;

    void loadTestMesh()
    {
        std::string validFile = "../object-files/gourd.obj";
        ASSERT_TRUE(Parser::loadMesh(validFile, mesh));
    }

    void createSimpleMesh()
    {
        TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
        TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
        TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0, 1, 0)); // Non-collinear vertex
        mesh.add_face({v0, v1, v2}); // Create a valid triangular face to generate edges
    }

    void validateVertexQuadrics()
    {
        for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
        {
            ASSERT_FALSE(QEMSimplifierUtils::getVertexQuadric(mesh, *v_it).isZero());
        }
    }
};


TEST_F(ComputeEdgeCollapseCostTests, computeEdgeCollapseCostFromObj)
{
    loadTestMesh();
    qem.computeQuadrics(mesh);

    // Validate quadrics are non-zero
    validateVertexQuadrics();

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

TEST_F(ComputeEdgeCollapseCostTests, ComputeEdgeCollapseCostFallback) // Hits fallback for A
{
    createSimpleMesh();
    qem.computeQuadrics(mesh);

    // Validate quadrics are non-zero
    validateVertexQuadrics();

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

// =============================================================================
//                             Collapse Edge Tests
// =============================================================================

class CollapseEdgeTest : public ::testing::Test
{
protected:
    QEMSimplifier qem;
    TriMesh mesh;

    void loadTestMesh()
    {
        // Example inspired from Figure 1 in the paper "Surface Simplification Using Quadric Error Metrics"
        std::string validFile = "../object-files/test.obj";
        ASSERT_TRUE(Parser::loadMesh(validFile, mesh));
    }

    void createSimpleTriangleMesh()
    {
        TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0.0, 0.0, 0.0));
        TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1.0, 0.0, 0.0));
        TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0.5, 1.0, 0.0));
        mesh.add_face({v0, v1, v2});
    }

    void initializeMeshStatusFlags()
    {
        // Request necessary status flags
        // needed for is_collapse_ok()
        mesh.request_vertex_status();
        mesh.request_edge_status();
        mesh.request_face_status();
    }

    TriMesh::VertexHandle findVertexByPoint(const TriMesh::Point& point)
    {
        for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
        {
            if (mesh.point(*v_it) == point)
            {
                return *v_it;
            }
        }
        return TriMesh::VertexHandle();
    }

    void findEdgeBetweenVertices(TriMesh::VertexHandle _v1, TriMesh::VertexHandle _v2, TriMesh::EdgeHandle& _edge)
    {
        for (auto e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it)
        {
            auto he0 = mesh.halfedge_handle(*e_it, 0);
            auto he1 = mesh.halfedge_handle(*e_it, 1);

            if ((mesh.to_vertex_handle(he0) == _v2 && mesh.from_vertex_handle(he0) == _v1) ||
                (mesh.to_vertex_handle(he1) == _v1 && mesh.from_vertex_handle(he1) == _v2))
            {
                _edge = *e_it;
                break;
            }
        }
        ASSERT_TRUE(_edge.is_valid()) << "Edge between specified vertices not found.";
    }
};

TEST_F(CollapseEdgeTest, collapseEdge)
{
    loadTestMesh();

    // Verify initial mesh structure
    EXPECT_EQ(mesh.n_vertices(), 10);
    EXPECT_EQ(mesh.n_faces(), 10);
    EXPECT_EQ(mesh.n_edges(), 19);

    // Find v1 and v2
    auto v1 = findVertexByPoint(TriMesh::Point(0.0, 0.0, 0.0));
    auto v2 = findVertexByPoint(TriMesh::Point(1.0, 0.0, 0.0));
    // Ensure that v1 and v2 are valid
    ASSERT_TRUE(v1.is_valid());
    ASSERT_TRUE(v2.is_valid());

    qem.computeQuadrics(mesh);

    // Ensure quadrics are initialized
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        QMatrix vQuadric = QEMSimplifierUtils::getVertexQuadric(mesh, *v_it);
        ASSERT_FALSE(vQuadric.isZero()); // Ensure quadric is updated
    }

    initializeMeshStatusFlags();

    // Find the edge between v1 and v2
    TriMesh::EdgeHandle edge;
    findEdgeBetweenVertices(v1, v2, edge);

    // Check if the collapse is allowed
    TriMesh::HalfedgeHandle he0 = mesh.halfedge_handle(edge, 0);
    ASSERT_TRUE(mesh.is_collapse_ok(he0));

    // TriMesh::VertexHandle vKeep = mesh.to_vertex_handle(he0); // By convention, keep "to_vertex"

    // Compute the new position for vKeep (midpoint of v1 and v2)
    Eigen::Vector3d newPos;
    TriMesh::Point p1 = mesh.point(v1);
    TriMesh::Point p2 = mesh.point(v2);
    newPos[0] = (p1[0] + p2[0]) / 2.0;
    newPos[1] = (p1[1] + p2[1]) / 2.0;
    newPos[2] = (p1[2] + p2[2]) / 2.0;

    TriMesh::VertexHandle vKeep;
    // Collapse the edge
    ASSERT_TRUE(qem.collapseEdge(mesh, edge, newPos, vKeep));

    // Remove deleted vertices, edges, and faces from the mesh
    mesh.garbage_collection();

    // Verify the state of the mesh after collapse
    EXPECT_EQ(mesh.n_vertices(), 9);
    EXPECT_EQ(mesh.n_faces(), 8);
    EXPECT_EQ(mesh.n_edges(), 16);

    // Verify the position of the new vertex
    TriMesh::Point vKeepPos = mesh.point(vKeep);

    EXPECT_FLOAT_EQ(vKeepPos[0], newPos[0]);
    EXPECT_FLOAT_EQ(vKeepPos[1], newPos[1]);
    EXPECT_FLOAT_EQ(vKeepPos[2], newPos[2]);
}

TEST_F(CollapseEdgeTest, collapseEdgeNotAllowed)
{
    createSimpleTriangleMesh();
    initializeMeshStatusFlags();

    qem.computeQuadrics(mesh);

    // Ensure quadrics are initialized
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        QMatrix vQuadric = QEMSimplifierUtils::getVertexQuadric(mesh, *v_it);
        ASSERT_FALSE(vQuadric.isZero()); // Ensure quadric is updated
    }

    // Find the edge between v0 and v1
    auto v0 = mesh.vertex_handle(0); // First vertex
    auto v1 = mesh.vertex_handle(1); // Second vertex
    ASSERT_TRUE(v0.is_valid());
    ASSERT_TRUE(v1.is_valid());

    // Get the edge between v0 and v1
    TriMesh::EdgeHandle edge;
    findEdgeBetweenVertices(v0, v1, edge);

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