#ifndef PARSER_H_
#define PARSER_H_

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <string>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;

struct Parser
{
    static bool loadMesh(const std::string& _filename, TriMesh& _mesh);
};

#endif // PARSER_H_