#ifndef QEMSIMPLIFIER_H_
#define QEMSIMPLIFIER_H_

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <Eigen/Dense>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;
typedef Eigen::Matrix4d QMatrix;

class QEMSimplifier
{
public:
    QEMSimplifier(); // Constructor

    Eigen::Vector4d computePlaneEquation(const TriMesh::Point& _p0,
                                         const TriMesh::Point& _p1,
                                         const TriMesh::Point& _p2);
    void computeQuadrics(TriMesh& _mesh);
    QMatrix computeFaceQuadric(TriMesh& _mesh, TriMesh::FaceHandle _face);
    void initializeQuadricsToZero(TriMesh& _mesh);
    void simplifyMesh(TriMesh& _mesh, size_t _tgtNumFaces);
    float computeEdgeCollapseCost(TriMesh& _mesh, TriMesh::EdgeHandle _edge, Eigen::Vector3d& _optPos);
    QMatrix& getVertexQuadric(TriMesh& mesh, const TriMesh::VertexHandle& vh) const {
        return mesh.property(vQuadric, vh);
    }

    struct EdgeInfo {
        TriMesh::EdgeHandle edgeHandle;
        double cost;
        Eigen::Vector3d optPos;

        // For priority queue -> we want a min-heap, so operator> sorts by cost ascending
        bool operator>(const EdgeInfo& rhs) const {
            return cost > rhs.cost;
        }
    };

    OpenMesh::VPropHandleT<Eigen::Matrix4d> vQuadric;
};

#endif // QEMSIMPLIFIER_H_