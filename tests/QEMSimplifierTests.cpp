#include <gtest/gtest.h>

#include "parser.h"

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
