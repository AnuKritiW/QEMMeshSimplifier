#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>

#include "parser.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "QEMSimplifier.h"

// Helper to re-extract the geometry from mesh and update Polyscope
void rebuildPolyscopeMesh(TriMesh& _mesh)
{
    // Convert mesh to arrays
    Eigen::MatrixXd vertices_matrix;
    Eigen::MatrixXi faces_matrix;
    Parser::extractMeshData(_mesh, vertices_matrix, faces_matrix);

    polyscope::removeStructure("Surface Mesh");
    polyscope::registerSurfaceMesh("Surface Mesh", vertices_matrix, faces_matrix);
}

int main() {
    TriMesh mesh;
    std::string filename = "object-files/gourd.obj";

    if (!Parser::loadMesh(filename, mesh)) {
        return EXIT_FAILURE;
    }

    std::cout << "Mesh loaded successfully!" << std::endl;

    // Initialize the QEMSimplifier and simplify the mesh

    // Create matrices to store vertices and faces
    Eigen::MatrixXd vertices_matrix;
    Eigen::MatrixXi faces_matrix;

    // Extract vertex and face data from the mesh
    Parser::extractMeshData(mesh, vertices_matrix, faces_matrix);

    // Polyscope viewer
    polyscope::init();
    polyscope::registerSurfaceMesh("Surface Mesh", vertices_matrix, faces_matrix);

    polyscope::state::userCallback = [&mesh]()
    {
        // Polyscope uses ImGui
        if (ImGui::Button("Simplify Once"))
        {
            QEMSimplifier simplifier;
            simplifier.simplifyMesh(mesh, mesh.n_faces() - 50); // remove 50 faces each time

            rebuildPolyscopeMesh(mesh);
        }
    };

    polyscope::show();
    std::cout << "Polyscope viewer initialised!\n";

    return EXIT_SUCCESS;
}
