#ifndef QEMSIMPLIFIERUTILS_H_
#define QEMSIMPLIFIERUTILS_H_

#include "QEMSimplifierUtils_config.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <Eigen/Dense>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;
typedef Eigen::Matrix4d QMatrix;

#if defined(QEM_BACKEND_METAL)
extern "C" void computeQuadricsInParallel_Metal(TriMesh& mesh, std::vector<QMatrix>& globalQuadrics);
#endif

class QEMSimplifierUtils
{
public:
    // Template utility for property initialization
    template <typename T>
    static void initializeProperty(TriMesh& _mesh, OpenMesh::VPropHandleT<T>& _property, const std::string& _name, const T& _defaultValue)
    {
        if (!_mesh.get_property_handle(_property, _name))
        {
            _mesh.add_property(_property, _name);
            for (auto v_it = _mesh.vertices_begin(); v_it != _mesh.vertices_end(); ++v_it)
            {
                _mesh.property(_property, *v_it) = _defaultValue;
            }
        }
    };

    template <typename T>
    static T mergeVertexProperties(const TriMesh& _mesh, TriMesh::VertexHandle _v0, TriMesh::VertexHandle _v1, OpenMesh::VPropHandleT<T>& _property)
    {
        return _mesh.property(_property, _v0) + _mesh.property(_property, _v1);
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