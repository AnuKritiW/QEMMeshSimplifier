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
    void computeQuadricsInParallel(TriMesh& _mesh, std::vector<QMatrix>& globalQuadrics);
    void computeQuadrics(TriMesh& _mesh);
    QMatrix computeFaceQuadric(TriMesh& _mesh, TriMesh::FaceHandle _face);
    void simplifyMesh(TriMesh& _mesh, size_t _tgtNumFaces);
    float computeEdgeCollapseCost(TriMesh& _mesh, TriMesh::EdgeHandle _edge, Eigen::Vector3d& _optPos);
    bool collapseEdge(TriMesh& _mesh, TriMesh::EdgeHandle _edge, const Eigen::Vector3d& _newPos, TriMesh::VertexHandle& _vKeep);

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

    QMatrix& getVertexQuadric(TriMesh& _mesh, const TriMesh::VertexHandle& _vh) const;

    // Template utility for property initialization
    template <typename T>
    void initializeProperty(TriMesh& _mesh, OpenMesh::VPropHandleT<T>& property, const std::string& name, const T& defaultValue)
    {
        if (!_mesh.get_property_handle(property, name))
        {
            _mesh.add_property(property, name);
            for (auto v_it = _mesh.vertices_begin(); v_it != _mesh.vertices_end(); ++v_it)
            {
                _mesh.property(property, *v_it) = defaultValue;
            }
        }
    }

};

#endif // QEMSIMPLIFIER_H_