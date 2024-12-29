#include <gtest/gtest.h>

#include "parser.h"
#include "QEMSimplifier.h"

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

TEST(QEMSimplifier, ComputeQuadricsSetsQuadricsToZero)
{
    QEMSimplifier qem;
    TriMesh mesh;
    std::string validFile = "../object-files/gourd.obj";
    ASSERT_TRUE(Parser::loadMesh(validFile, mesh));

    static OpenMesh::VPropHandleT<Eigen::Matrix4d> vQuadric;
    if (!mesh.get_property_handle(vQuadric, "v:quadric"))
    {
        mesh.add_property(vQuadric, "v:quadric");
    }

    ASSERT_TRUE(mesh.get_property_handle(vQuadric, "v:quadric"));

    // Call the function
    qem.computeQuadrics(mesh);

    // Verify all quadrics are set to zero
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        Eigen::Matrix4d quadric = mesh.property(vQuadric, *v_it);
        EXPECT_TRUE(quadric.isZero());
    }
}