#ifndef QEMSIMPLIFIER_H_
#define QEMSIMPLIFIER_H_

#include <Eigen/Dense>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <queue>

#ifdef UNIT_TEST
#include <gtest/gtest.h> // Provide the real FRIEND_TEST macro
#endif

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;
typedef Eigen::Matrix4d QMatrix;

class QEMSimplifier
{
public:
    QEMSimplifier(); // Constructor
    void simplifyMesh(TriMesh& _mesh, size_t _tgtNumFaces);

private:
    struct EdgeInfo
    {
        TriMesh::EdgeHandle edgeHandle;
        float cost;
        Eigen::Vector3d optPos;

        int versionSum = 0;
        bool operator>(const EdgeInfo& rhs) const
        {
            return cost > rhs.cost;
        }
    };

private:
    void computeQuadrics(TriMesh& _mesh);
#ifdef UNIT_TEST
    FRIEND_TEST(ComputeQuadricsTest, computeQuadrics);
    FRIEND_TEST(ComputeQuadricsTest, computeQuadricsMeshFile);
    FRIEND_TEST(ComputeEdgeCollapseCostTests, computeEdgeCollapseCostFromObj);
    FRIEND_TEST(ComputeEdgeCollapseCostTests, ComputeEdgeCollapseCostFallback);
#endif

    void initializePriorityQueue(TriMesh& _mesh,
                                 std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>>& _priQ);
    friend class InitializePriorityQueueTest;
#ifdef UNIT_TEST
    FRIEND_TEST(InitializePriorityQueueTest, initializePriorityQueue);
    FRIEND_TEST(InitializePriorityQueueTest, initializePriorityQueueEmptyMesh);
    FRIEND_TEST(InitializePriorityQueueTest, initializePriorityQueueDeletedEdges);
#endif

    bool collapseEdge(TriMesh& _mesh,
                      TriMesh::EdgeHandle _edge,
                      const Eigen::Vector3d& _newPos,
                      TriMesh::VertexHandle& _vKeep);
#ifdef UNIT_TEST
    FRIEND_TEST(CollapseEdgeTest, collapseEdge);
    FRIEND_TEST(CollapseEdgeTest, collapseEdgeNotAllowed);
#endif

    void recalculateEdgeCosts(TriMesh& _mesh,
                              TriMesh::VertexHandle _vKeep,
                              std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>>& _priQ);
    friend class RecalculateEdgeCostsTest;
#ifdef UNIT_TEST
    FRIEND_TEST(RecalculateEdgeCostsTest, recalculateEdgeCosts);
    FRIEND_TEST(RecalculateEdgeCostsTest, recalculateEdgeCostsNoValidNeighbors);
    FRIEND_TEST(RecalculateEdgeCostsTest, recalculateEdgeCostsEmptyMesh);
    FRIEND_TEST(RecalculateEdgeCostsTest, recalculateEdgeCostsStaleEntries);
#endif
};

#endif // QEMSIMPLIFIER_H_