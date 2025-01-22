#include <gtest/gtest.h>

#include "parser.h"
#include "QEMSimplifier.h"
#include "QEMSimplifierUtils.h"
#include "TestConstants.h"

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
            // normalising makes the test more robust to the natural limitations of floating-point arithmetic
            // better aligned with the practical significance of the differences
            EXPECT_NEAR(optPos[0] / expectedOptPos.norm(), expectedOptPos[0] / expectedOptPos.norm(), g_tolerance);
            EXPECT_NEAR(optPos[1] / expectedOptPos.norm(), expectedOptPos[1] / expectedOptPos.norm(), g_tolerance);
            EXPECT_NEAR(optPos[2] / expectedOptPos.norm(), expectedOptPos[2] / expectedOptPos.norm(), g_tolerance);
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

// =============================================================================
//                       Initialize Priority Queue Tests
// =============================================================================

class InitializePriorityQueueTest : public ::testing::Test
{
protected:
    QEMSimplifier qem;
    TriMesh mesh;
    OpenMesh::VPropHandleT<int> vVersion; // Vertex version property

    using EdgeInfo = QEMSimplifier::EdgeInfo;

    void SetUp() override
    {
        // Initialize vertex version property
        if (!mesh.get_property_handle(vVersion, "v:version"))
        {
            mesh.add_property(vVersion, "v:version");
            for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
            {
                mesh.property(vVersion, *v_it) = 0; // Initialize to 0
            }
        }

        // Create a simple triangle mesh
        TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
        TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
        TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0.5, 1, 0));
        TriMesh::VertexHandle v3 = mesh.add_vertex(TriMesh::Point(0.0, 0.0, 1.0));
        mesh.add_face({v0, v1, v2});
        mesh.add_face({v0, v1, v3});
        mesh.add_face({v0, v2, v3});
        mesh.add_face({v1, v2, v3});

        // Request necessary status flags
        // needed for is_collapse_ok()
        mesh.request_vertex_status();
        mesh.request_edge_status();
        mesh.request_face_status();
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

TEST_F(InitializePriorityQueueTest, initializePriorityQueue)
{
    // Prepare the priority queue
    std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> priQ;

    qem.computeQuadrics(mesh);
    qem.initializePriorityQueue(mesh, priQ);

    // Validate priority queue size
    EXPECT_EQ(priQ.size(), mesh.n_edges());

    // Validate edge information in the queue
    while (!priQ.empty())
    {
        EdgeInfo edgeInfo = priQ.top();
        priQ.pop();

        // Ensure the cost is populated; it can be 0
        EXPECT_GE(edgeInfo.cost, 0.0f);

        // Ensure the optimal position is within expected bounds
        EXPECT_GE(edgeInfo.optPos[0], 0.0);
        EXPECT_GE(edgeInfo.optPos[1], 0.0);
        EXPECT_GE(edgeInfo.optPos[2], 0.0);
    }
}

TEST_F(InitializePriorityQueueTest, initializePriorityQueueEmptyMesh)
{
    TriMesh emptyMesh;
    std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> priQ;

    qem.initializePriorityQueue(emptyMesh, priQ);

    // Priority queue should be empty for an empty mesh
    EXPECT_EQ(priQ.size(), 0);
}

TEST_F(InitializePriorityQueueTest, initializePriorityQueueDeletedEdges)
{
    const int numEdges = mesh.n_edges();
    // Find the edge between v0 and v1
    auto v0 = mesh.vertex_handle(0); // First vertex
    auto v1 = mesh.vertex_handle(1); // Second vertex
    ASSERT_TRUE(v0.is_valid());
    ASSERT_TRUE(v1.is_valid());

    // Get the edge between v0 and v1
    TriMesh::EdgeHandle edge;
    findEdgeBetweenVertices(v0, v1, edge);
    mesh.garbage_collection();

    // Mark one edge as deleted
    mesh.status(edge).set_deleted(true);

    std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> priQ;
    qem.computeQuadrics(mesh);
    qem.initializePriorityQueue(mesh, priQ);

    // Priority queue should only contain the non-deleted edge
    EXPECT_EQ(priQ.size(), (numEdges - 1));

    EdgeInfo edgeInfo = priQ.top();
    EXPECT_GE(edgeInfo.cost, 0.0f);
}

// =============================================================================
//                              Simplify Mesh Tests
// =============================================================================

class SimplifyMeshTest : public ::testing::Test
{
protected:
    QEMSimplifier simplifier;
    TriMesh mesh;
    OpenMesh::VPropHandleT<int> vVersion;

    void SetUp() override {
        // Initialize the mesh
        if (!mesh.get_property_handle(vVersion, "v:version"))
        {
            mesh.add_property(vVersion, "v:version");
            for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
            {
                mesh.property(vVersion, *v_it) = 0;
            }
        }

        // Create a simple triangular mesh
        TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
        TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
        TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0.5, 1, 0));
        TriMesh::VertexHandle v3 = mesh.add_vertex(TriMesh::Point(1, 1, 0));
        TriMesh::VertexHandle v4 = mesh.add_vertex(TriMesh::Point(0, 1, 0));

        mesh.add_face({v0, v1, v2});
        mesh.add_face({v1, v3, v2});
        mesh.add_face({v2, v3, v4});
        mesh.add_face({v4, v0, v2});
    }
};

TEST_F(SimplifyMeshTest, simplifyMesh)
{
    size_t targetFaces = 2;

    // Simplify the mesh
    simplifier.simplifyMesh(mesh, targetFaces);

    // Verify the number of faces
    EXPECT_LE(mesh.n_faces(), targetFaces);

    EXPECT_GE(mesh.n_vertices(), 3);
    EXPECT_GE(mesh.n_edges(), 3);
}

TEST_F(SimplifyMeshTest, simplifyEmptyMesh)
{
    TriMesh emptyMesh;

    // Simplify the empty mesh
    simplifier.simplifyMesh(emptyMesh, 0);

    // Verify the mesh is still empty
    EXPECT_EQ(emptyMesh.n_faces(), 0);
    EXPECT_EQ(emptyMesh.n_vertices(), 0);
    EXPECT_EQ(emptyMesh.n_edges(), 0);
}

TEST_F(SimplifyMeshTest, simplifyMeshTgtFacesGreaterThanBaseMesh)
{
    size_t currentFaces = mesh.n_faces();
    size_t targetFaces = currentFaces + 2;

    // Simplify the mesh
    simplifier.simplifyMesh(mesh, targetFaces);

    // Verify the number of faces remains unchanged
    EXPECT_EQ(mesh.n_faces(), currentFaces);
}

TEST_F(SimplifyMeshTest, simplifyMeshNegativeFaces)
{
    size_t tgtFaces = static_cast<size_t>(-1); // Negative target

    // Simplify the mesh
    simplifier.simplifyMesh(mesh, tgtFaces);

    EXPECT_GE(mesh.n_faces(), 4);
    EXPECT_GE(mesh.n_vertices(), 5);
}

// =============================================================================
//                         Recalculate Edge Cost Tests
// =============================================================================

class RecalculateEdgeCostsTest : public ::testing::Test
{
protected:
    QEMSimplifier qem;
    TriMesh mesh;
    OpenMesh::VPropHandleT<int> vVersion;

     using EdgeInfo = QEMSimplifier::EdgeInfo;

    void SetUp() override {
        // Initialize the mesh
        if (!mesh.get_property_handle(vVersion, "v:version"))
        {
            mesh.add_property(vVersion, "v:version");
            for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
            {
                mesh.property(vVersion, *v_it) = 0;
            }
        }

        // Create a simple quad mesh (two connected triangles)
        TriMesh::VertexHandle v0 = mesh.add_vertex(TriMesh::Point(0, 0, 0));
        TriMesh::VertexHandle v1 = mesh.add_vertex(TriMesh::Point(1, 0, 0));
        TriMesh::VertexHandle v2 = mesh.add_vertex(TriMesh::Point(0, 1, 0));
        TriMesh::VertexHandle v3 = mesh.add_vertex(TriMesh::Point(1, 1, 0));

        mesh.add_face({v0, v1, v2}); // First triangle
        mesh.add_face({v1, v3, v2}); // Second triangle

        mesh.request_vertex_status();
        mesh.request_edge_status();
        mesh.request_face_status();
    }
};

TEST_F(RecalculateEdgeCostsTest, recalculateEdgeCosts)
{
    qem.computeQuadrics(mesh);

    std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> priQ;
    qem.initializePriorityQueue(mesh, priQ);

    // Simulate collapsing an edge
    auto he = mesh.find_halfedge(mesh.vertex_handle(0), mesh.vertex_handle(1));
    auto edge = mesh.edge_handle(he);
    auto vKeep = mesh.vertex_handle(0); // Keep v0

    // Mark the edge as collapsed
    mesh.status(edge).set_deleted(true);

    // Recalculate edge costs around vKeep
    qem.recalculateEdgeCosts(mesh, vKeep, priQ);

    // Expected recalculated costs
    std::unordered_map<int, float> expectedCosts;
    for (auto vv_it = mesh.vv_iter(vKeep); vv_it.is_valid(); ++vv_it)
    {
        auto vNeighbor = *vv_it;
        auto he = mesh.find_halfedge(vKeep, vNeighbor);
        auto edge = mesh.edge_handle(he);

        // Compute the expected cost
        Eigen::Vector3d optPos;
        float expectedCost = QEMSimplifierUtils::computeEdgeCollapseCost(mesh, edge, optPos);

        expectedCosts[edge.idx()] = expectedCost;
    }

    // Validate recalculated costs in the priority queue
    while (!priQ.empty())
    {
        EdgeInfo edgeInfo = priQ.top();
        priQ.pop();

        if (mesh.status(edgeInfo.edgeHandle).deleted())
        {
            continue; // skip
        }

        // Validate recalculated cost
        EXPECT_NEAR(edgeInfo.cost, expectedCosts[edgeInfo.edgeHandle.idx()], 1e-6);

        // Validate the version sum
        int expectedVersionSum = QEMSimplifierUtils::mergeVertexProperties(
            mesh,
            mesh.to_vertex_handle(mesh.halfedge_handle(edgeInfo.edgeHandle, 0)),
            mesh.from_vertex_handle(mesh.halfedge_handle(edgeInfo.edgeHandle, 0)),
            vVersion
        );
        EXPECT_EQ(edgeInfo.versionSum, expectedVersionSum);
    }
}

TEST_F(RecalculateEdgeCostsTest, recalculateEdgeCostsNoValidNeighbors)
{
    std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> priQ;

    // Mark all edges as deleted
    auto vKeep = mesh.vertex_handle(0);
    for (auto vv_it = mesh.vv_iter(vKeep); vv_it.is_valid(); ++vv_it)
    {
        auto he = mesh.find_halfedge(vKeep, *vv_it);
        mesh.status(mesh.edge_handle(he)).set_deleted(true);
    }

    qem.recalculateEdgeCosts(mesh, vKeep, priQ);

    // Ensure the priority queue remains empty
    EXPECT_TRUE(priQ.empty());
}

TEST_F(RecalculateEdgeCostsTest, recalculateEdgeCostsEmptyMesh)
{
    TriMesh emptyMesh;
    std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> priQ;

    // Attempt to recalculate edge costs on an empty mesh
    qem.recalculateEdgeCosts(emptyMesh, TriMesh::VertexHandle(0), priQ);

    // Ensure the priority queue remains empty
    EXPECT_TRUE(priQ.empty());
}
