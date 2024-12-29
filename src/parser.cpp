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

void Parser::extractMeshData(const TriMesh& _mesh, Eigen::MatrixXd& vertices_matrix, Eigen::MatrixXi& faces_matrix)
{
    // Extract vertices
    vertices_matrix = Eigen::MatrixXd(_mesh.n_vertices(), 3);  // Matrix with n_vertices rows and 3 columns (X, Y, Z)
    size_t idx = 0;
    for (const auto& vert : _mesh.vertices()) {
        const auto& point = _mesh.point(vert);  // Access vertex position
        vertices_matrix(idx, 0) = point[0];  // X
        vertices_matrix(idx, 1) = point[1];  // Y
        vertices_matrix(idx, 2) = point[2];  // Z
        idx++;
    }

    // Extract faces
    faces_matrix = Eigen::MatrixXi(_mesh.n_faces(), 3);  // Matrix with n_faces rows and 3 columns (indices)
    idx = 0;
    for (const auto& face : _mesh.faces()) {
        auto fv_iter = _mesh.cfv_iter(face);
        int i = 0;
        for (auto fv = fv_iter; fv.is_valid(); ++fv) {
            faces_matrix(idx, i) = fv->idx();  // Store vertex index
            i++;
        }
        idx++;
    }
}
