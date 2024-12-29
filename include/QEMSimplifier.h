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
    Eigen::Vector4d computePlaneEquation(const TriMesh::Point& p0,
                                         const TriMesh::Point& p1,
                                         const TriMesh::Point& p2);
    void computeQuadrics(TriMesh& mesh);
    QMatrix computeFaceQuadric(TriMesh& mesh, TriMesh::FaceHandle fh);
    void initializeQuadricsToZero(TriMesh& mesh);
};

#endif // QEMSIMPLIFIER_H_