#include <iostream>

#include "parser.h"

bool Parser::loadMesh(const std::string& _filename, TriMesh& _mesh) {

    if (!OpenMesh::IO::read_mesh(_mesh, _filename)) {
        std::cerr << "Error: Cannot read mesh from " << _filename << std::endl;
        return false;
    }

    std::cout << "Successfully loaded mesh with " 
              << _mesh.n_vertices() << " vertices and "
              << _mesh.n_faces() << " faces." << std::endl;

    return true;
}
