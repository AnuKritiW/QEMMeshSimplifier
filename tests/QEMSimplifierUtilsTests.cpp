#include <gtest/gtest.h>

#include "QEMSimplifierUtils.h"
#include "TestConstants.h"

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
//                       Compute Quadrics Parallel Tests
// =============================================================================

TEST_F(QEMSimplifierUtilsTests, computeQuadricsInParallel)
{
    // Prepare global quadrics for each vertex
    std::vector<QMatrix> globalQuadrics(mesh.n_vertices(), QMatrix::Zero());

    // Compute quadrics in parallel
    QEMSimplifierUtils::computeQuadricsInParallel(mesh, globalQuadrics);

    // Validate quadrics
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        size_t vertexIdx = v_it->idx();

        // Expected quadric: Sum of quadrics of all adjacent faces
        QMatrix expectedQuadric = QMatrix::Zero();
        for (auto vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it)
        {
            expectedQuadric += QEMSimplifierUtils::computeFaceQuadric(mesh, *vf_it);
        }

        EXPECT_TRUE(globalQuadrics[vertexIdx].isApprox(expectedQuadric, 1e-6));
    }
}

TEST_F(QEMSimplifierUtilsTests, computeQuadricsInParallelEmptyMesh) {
    TriMesh emptyMesh;
    std::vector<QMatrix> emptyQuadrics;

    QEMSimplifierUtils::computeQuadricsInParallel(emptyMesh, emptyQuadrics);

    EXPECT_TRUE(emptyQuadrics.empty());
}

TEST_F(QEMSimplifierUtilsTests, ComputeQuadricsInParallelExtendedMesh)
{
    // Extend the shared mesh
    auto v3 = mesh.add_vertex(TriMesh::Point(1.0, 1.0, 0.0));
    mesh.add_face({mesh.vertex_handle(0), mesh.vertex_handle(1), v3});
    mesh.add_face({mesh.vertex_handle(1), mesh.vertex_handle(2), v3});
    mesh.add_face({mesh.vertex_handle(2), mesh.vertex_handle(0), v3});

    std::vector<QMatrix> globalQuadrics(mesh.n_vertices(), QMatrix::Zero());

    QEMSimplifierUtils::computeQuadricsInParallel(mesh, globalQuadrics);

    // Validate results
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        size_t vertexIdx = v_it->idx();

        QMatrix expectedQuadric = QMatrix::Zero();
        for (auto vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it)
        {
            expectedQuadric += QEMSimplifierUtils::computeFaceQuadric(mesh, *vf_it);
        }
        EXPECT_TRUE(globalQuadrics[vertexIdx].isApprox(expectedQuadric, 1e-6));
    }
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