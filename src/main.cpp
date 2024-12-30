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

    float simplifyPercentage = 10.0f; // Default to 10%
    polyscope::state::userCallback = [&mesh, &simplifyPercentage]()
    {
        ImGui::Text("Simplification Settings");
        ImGui::SliderFloat("Percentage to Simplify", &simplifyPercentage, 0.0f, 100.0f, "%.1f%%");

        // Polyscope uses ImGui
        if (ImGui::Button("Simplify Mesh"))
        {
            size_t tgtNumFaces = static_cast<size_t>(
                mesh.n_faces() * (1.0f - simplifyPercentage / 100.0f)
            );
            if (tgtNumFaces < 1) {
                tgtNumFaces = 1; // Ensure at least one face remains
            }

            QEMSimplifier simplifier;
            simplifier.simplifyMesh(mesh, tgtNumFaces);

            rebuildPolyscopeMesh(mesh);
        }
    };

    polyscope::show();
    std::cout << "Polyscope viewer initialised!\n";

    return EXIT_SUCCESS;
}
