#ifndef QEMSIMPLIFIER_H_
#define QEMSIMPLIFIER_H_

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <queue>

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
    void initializePriorityQueue(TriMesh& _mesh,
                                 std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>>& _priQ);
    bool collapseEdge(TriMesh& _mesh,
                      TriMesh::EdgeHandle _edge,
                      const Eigen::Vector3d& _newPos,
                      TriMesh::VertexHandle& _vKeep);
    void recalculateEdgeCosts(TriMesh& _mesh,
                              TriMesh::VertexHandle _vKeep,
                              std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>>& _priQ);

    FRIEND_TEST(ComputeQuadricsTest, computeQuadrics);
    FRIEND_TEST(ComputeQuadricsTest, computeQuadricsMeshFile);
    FRIEND_TEST(ComputeEdgeCollapseCostTests, computeEdgeCollapseCostFromObj);
    FRIEND_TEST(ComputeEdgeCollapseCostTests, ComputeEdgeCollapseCostFallback);
    FRIEND_TEST(CollapseEdgeTest, collapseEdge);
    FRIEND_TEST(CollapseEdgeTest, collapseEdgeNotAllowed);
};

#endif // QEMSIMPLIFIER_H_