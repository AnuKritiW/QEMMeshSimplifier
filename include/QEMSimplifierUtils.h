#ifndef QEMSIMPLIFIERUTILS_H_
#define QEMSIMPLIFIERUTILS_H_

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <Eigen/Dense>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;
typedef Eigen::Matrix4d QMatrix;

class QEMSimplifierUtils
{
public:
    // Template utility for property initialization
    template <typename T>
    static void initializeProperty(TriMesh& _mesh, OpenMesh::VPropHandleT<T>& property, const std::string& name, const T& defaultValue)
    {
        if (!_mesh.get_property_handle(property, name))
        {
            _mesh.add_property(property, name);
            for (auto v_it = _mesh.vertices_begin(); v_it != _mesh.vertices_end(); ++v_it)
            {
                _mesh.property(property, *v_it) = defaultValue;
            }
        }
    };

    template <typename T>
    static T mergeVertexProperties(const TriMesh& _mesh, TriMesh::VertexHandle v0, TriMesh::VertexHandle v1, OpenMesh::VPropHandleT<T>& property)
    {
        return _mesh.property(property, v0) + _mesh.property(property, v1);
    }

    // Compute Functions
    static Eigen::Vector4d computePlaneEquation(const TriMesh::Point& _p0,
                                                const TriMesh::Point& _p1,
                                                const TriMesh::Point& _p2);
    static QMatrix computeFaceQuadric(TriMesh& _mesh, TriMesh::FaceHandle _face);
    static void computeQuadricsInParallel(TriMesh& _mesh, std::vector<QMatrix>& globalQuadrics);
    static float computeEdgeCollapseCost(TriMesh& _mesh, TriMesh::EdgeHandle _edge, Eigen::Vector3d& _optPos);

    // getters
    static OpenMesh::VPropHandleT<Eigen::Matrix4d>& getQuadricHandle();
    static OpenMesh::VPropHandleT<int>& getVersionHandle();
    static QMatrix& getVertexQuadric(TriMesh& _mesh, const TriMesh::VertexHandle& _vh);

private:
    static OpenMesh::VPropHandleT<Eigen::Matrix4d> vQuadric;
    static OpenMesh::VPropHandleT<int> vVersion;
};

#endif // QEMSIMPLIFIERUTILS_H_