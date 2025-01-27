#ifndef PARSER_H_
#define PARSER_H_

#include <Eigen/Dense>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <string>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;

struct Parser
{
    static bool loadMesh(const std::string& _filename, TriMesh& _mesh);
    static void extractMeshData(const TriMesh& _mesh, Eigen::MatrixXd& _vertices_matrix, Eigen::MatrixXi& _faces_matrix);

};

#endif // PARSER_H_