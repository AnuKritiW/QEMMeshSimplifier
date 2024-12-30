#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>

#include "parser.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "QEMSimplifier.h"

int main() {
    TriMesh mesh;
    std::string filename = "object-files/gourd.obj";

    if (!Parser::loadMesh(filename, mesh)) {
        return EXIT_FAILURE;
    }

    std::cout << "Mesh loaded successfully!" << std::endl;

    // TODO: add button to polyscope viewer which allows real time simplification
    const unsigned int tgtNumFaces = 200;

    // Initialize the QEMSimplifier and simplify the mesh
    QEMSimplifier simplifier;
    simplifier.simplifyMesh(mesh, tgtNumFaces);

    // Create matrices to store vertices and faces
    Eigen::MatrixXd vertices_matrix;
    Eigen::MatrixXi faces_matrix;

    // Extract vertex and face data from the mesh
    Parser::extractMeshData(mesh, vertices_matrix, faces_matrix);

    // Polyscope viewer
    polyscope::init();
    polyscope::registerSurfaceMesh("icosahedron", vertices_matrix, faces_matrix);
    polyscope::show();
    std::cout << "Polyscope viewer initialised!\n";

    return EXIT_SUCCESS;
}
