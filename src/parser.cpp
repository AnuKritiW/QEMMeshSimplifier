#include <iostream>

#include "parser.h"

/**
 * @brief Loads a mesh from a file into the provided `TriMesh` object.
 *
 * @param _filename The path to the mesh file to be loaded.
 * @param _mesh The `TriMesh` object where the loaded mesh data will be stored.
 * @return `true` if the mesh was loaded successfully; `false` otherwise.
 *
 * This function uses OpenMesh to read a mesh file and populate the provided
 * `TriMesh` object. It logs the success or failure of the operation and
 * displays the number of vertices and faces loaded on success.
 */
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

/**
 * @brief Extracts vertex and face data from a `TriMesh` object into Eigen matrices.
 *
 * @param _mesh The `TriMesh` object containing the mesh data to extract.
 * @param _vertices_matrix An Eigen matrix to store vertex positions. Each row represents a vertex (X, Y, Z).
 * @param _faces_matrix An Eigen matrix to store face indices. Each row represents a face (v0, v1, v2).
 *
 * This function iterates through the vertices and faces of the given `TriMesh`
 * and extracts their data into Eigen matrices for further processing or visualization.
 */
void Parser::extractMeshData(const TriMesh& _mesh, Eigen::MatrixXd& _vertices_matrix, Eigen::MatrixXi& _faces_matrix)
{
    // Extract vertices
    _vertices_matrix = Eigen::MatrixXd(_mesh.n_vertices(), 3);  // Matrix with n_vertices rows and 3 columns (X, Y, Z)
    size_t idx = 0;
    for (const auto& vert : _mesh.vertices()) {
        const auto& point = _mesh.point(vert);  // Access vertex position
        _vertices_matrix(idx, 0) = point[0];  // X
        _vertices_matrix(idx, 1) = point[1];  // Y
        _vertices_matrix(idx, 2) = point[2];  // Z
        idx++;
    }

    // Extract faces
    _faces_matrix = Eigen::MatrixXi(_mesh.n_faces(), 3);  // Matrix with n_faces rows and 3 columns (indices)
    idx = 0;
    for (const auto& face : _mesh.faces()) {
        auto fv_iter = _mesh.cfv_iter(face);
        int i = 0;
        for (auto fv = fv_iter; fv.is_valid(); ++fv) {
            _faces_matrix(idx, i) = fv->idx();  // Store vertex index
            i++;
        }
        idx++;
    }
}
