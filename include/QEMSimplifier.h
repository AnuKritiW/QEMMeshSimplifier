#ifndef QEMSIMPLIFIER_H_
#define QEMSIMPLIFIER_H_

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;

class QEMSimplifier
{
public:
    void computeQuadrics(TriMesh& mesh);
};

#endif // QEMSIMPLIFIER_H_