#include <gtest/gtest.h>

#include "parser.h"
#include "QEMSimplifier.h"

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

// initializeQuadricsToZero
TEST(QEMSimplifier, initializeQuadricsToZero)
{
    QEMSimplifier qem;
    TriMesh mesh;
    std::string validFile = "../object-files/gourd.obj";
    ASSERT_TRUE(Parser::loadMesh(validFile, mesh));

    static OpenMesh::VPropHandleT<QMatrix> vQuadric;
    if (!mesh.get_property_handle(vQuadric, "v:quadric"))
    {
        mesh.add_property(vQuadric, "v:quadric");
    }

    ASSERT_TRUE(mesh.get_property_handle(vQuadric, "v:quadric"));

    // Call the function
    qem.initializeQuadricsToZero(mesh);

    // Verify all quadrics are set to zero
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        QMatrix quadric = mesh.property(vQuadric, *v_it);
        EXPECT_TRUE(quadric.isZero());
    }
}

// computePlaneEquation
TEST(QEMSimplifier, computePlaneEquation)
{
    /*
    Input: A triangle lying in the XY plane ((0,0,0), (1,0,0), (0,1,0))
    Expected Output: The plane equation should be (z = 0)
                     So, a = 0, b = 0, c = 1, d = 0
    */

    QEMSimplifier qem;

    // Triangle vertices in the XY plane
    TriMesh::Point p0(0, 0, 0);
    TriMesh::Point p1(1, 0, 0);
    TriMesh::Point p2(0, 1, 0);

    Eigen::Vector4d plane = qem.computePlaneEquation(p0, p1, p2);

    EXPECT_FLOAT_EQ(plane[0], 0.0); // a = 0
    EXPECT_FLOAT_EQ(plane[1], 0.0); // b = 0
    EXPECT_FLOAT_EQ(plane[2], 1.0); // c = 1
    EXPECT_FLOAT_EQ(plane[3], 0.0); // d = 0
}

// computePlaneEquation
TEST(QEMSimplifier, computePlaneEquationTranslatedZ)
{
    /*
    Input: A triangle parallel to the XY plane but shifted in the Z direction ((0,0,1), (1,0,1), (0,1,1))
    Expected Output: The plane equation should be (z − 1 = 0)
                     So, a = 0, b = 0, c = 1, d = -1
    */

    QEMSimplifier qem;

    // Triangle vertices in the XY plane
    TriMesh::Point p0(0, 0, 1);
    TriMesh::Point p1(1, 0, 1);
    TriMesh::Point p2(0, 1, 1);

    Eigen::Vector4d plane = qem.computePlaneEquation(p0, p1, p2);

    EXPECT_FLOAT_EQ(plane[0], 0.0);     // a = 0
    EXPECT_FLOAT_EQ(plane[1], 0.0);     // b = 0
    EXPECT_FLOAT_EQ(plane[2], 1.0);     //  c = 1
    EXPECT_FLOAT_EQ(plane[3], -1.0);    // d = -1
}

// computePlaneEquation
TEST(QEMSimplifier, computePlaneEquationRandomTri)
{
    /*
    Input: A triangle that is not aligned with any axis
    Expected Output: Commented inline below
    */

    QEMSimplifier qem;

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


    Eigen::Vector4d plane = qem.computePlaneEquation(p0, p1, p2);

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
TEST(QEMSimplifier, computeFaceQuadricKnownTriangle)
{
    QEMSimplifier qem;
    TriMesh mesh;

    // Create a triangle in the XY plane
    TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
    TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
    TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0, 1, 0));
    TriMesh::FaceHandle fh = mesh.add_face(v0, v1, v2);

    QMatrix quadric = qem.computeFaceQuadric(mesh, fh);

    QMatrix expectedQuadric;
    expectedQuadric.setZero();
    expectedQuadric(2, 2) = 1.0; // Precomputed for z=0

    EXPECT_TRUE(quadric.isApprox(expectedQuadric, 1e-6));
}

TEST(QEMSimplifier, computeFaceQuadricArbitraryTriangle)
{
    QEMSimplifier qem;
    TriMesh mesh;

    // Create a triangle on an arbitrary plane
    TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
    TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
    TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0, 1, 1));
    TriMesh::FaceHandle fh = mesh.add_face(v0, v1, v2);

    QMatrix quadric = qem.computeFaceQuadric(mesh, fh);

    // Plane equation: computed dynamically
    Eigen::Vector4d plane = qem.computePlaneEquation(
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

TEST(QEMSimplifier, ComputeQuadricsMeshFile) {
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