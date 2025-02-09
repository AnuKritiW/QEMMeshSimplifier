#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <stack>
#include <unistd.h> // For getcwd
#include <sys/stat.h> // For stat

#include "parser.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "QEMSimplifier.h"
#include "tinyfiledialogs.h"

/**
 * @brief Rebuilds the Polyscope mesh display with the current state of the mesh.
 *
 * @param _mesh The input mesh to be visualized in Polyscope.
 *
 * This function extracts vertex and face data from the input mesh and updates
 * the Polyscope viewer with the current geometry.
 */
void rebuildPolyscopeMesh(TriMesh& _mesh)
{
    // Convert mesh to arrays
    Eigen::MatrixXd vertices_matrix;
    Eigen::MatrixXi faces_matrix;
    Parser::extractMeshData(_mesh, vertices_matrix, faces_matrix);

    polyscope::removeStructure("Surface Mesh");
    polyscope::registerSurfaceMesh("Surface Mesh", vertices_matrix, faces_matrix);
}

/**
 * @brief Opens a file explorer dialog to load a new mesh file.
 *
 * @param _mesh The mesh object to load data into from the selected file.
 * @param _isFileDialogOpen A flag indicating whether the file dialog is currently open.
 * @param _filename The name of the file selected by the user.
 *
 * This function presents a file dialog to the user, allowing them to select
 * an `.obj` file. If a valid file is selected, the mesh is loaded and displayed
 * in Polyscope.
 */
void openFileExplorer(TriMesh& _mesh, bool& _isFileDialogOpen, std::string& _filename)
{
    const char* filters[] = { "*.obj" };

    // Get the current working directory
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) == nullptr)
    {
        perror("getcwd error");
        return;
    }

    std::string relativePath = std::string(cwd) + "/../object-files/";
    const char* homeDir = relativePath.c_str();

    // Check if the directory exists
    struct stat info;
    if (stat(homeDir, &info) != 0 || !(info.st_mode & S_IFDIR))
    {
        // If it does not exist or is not a directory, fall back to HOME
        homeDir = getenv("HOME");
    }

    if (!homeDir)
    {
        homeDir = "";
    }

    if (_isFileDialogOpen) {
        const char* filePath = tinyfd_openFileDialog(
            "Open .obj File", homeDir, 1, filters, NULL, 0
        );

        if (filePath && Parser::loadMesh(filePath, _mesh))
        {
            _filename = filePath;
            rebuildPolyscopeMesh(_mesh);
        }

        _isFileDialogOpen = false; // Close the dialog after interaction
    }
}

/**
 * @brief Simplifies the mesh by reducing its face count by a given percentage.
 *
 * @param _mesh The input mesh to be simplified.
 * @param _meshHistory A stack storing previous mesh states for undo functionality.
 * @param _simplifyPercentage The percentage by which to simplify the mesh.
 *
 * This function calculates the target number of faces based on the percentage
 * and uses the QEMSimplifier to simplify the mesh. The previous state of the
 * mesh is saved for undoing the operation.
 */
void simplifyMeshByPercentage(TriMesh& _mesh, std::stack<TriMesh>& _meshHistory, float _simplifyPercentage)
{
    _meshHistory.push(_mesh); // save current state for undo

    size_t tgtNumFaces = static_cast<size_t>(_mesh.n_faces() * (1.0f - _simplifyPercentage / 100.0f));

    if (tgtNumFaces < 1)
    {
        tgtNumFaces = 1; // ensure at least one face remains
    }

    freopen("/Users/Exhale/Desktop/CAVE/ASE/programming-project-AnuKritiW/output.log", "a+", stderr);
    QEMSimplifier simplifier;
    simplifier.simplifyMesh(_mesh, tgtNumFaces);

    rebuildPolyscopeMesh(_mesh);
}

/**
 * @brief Undoes the last mesh simplification.
 *
 * @param _mesh The mesh object to revert to the previous state.
 * @param _meshHistory A stack storing the history of mesh states.
 *
 * This function pops the most recent mesh state from the history stack
 * and restores it to undo the last simplification operation.
 */
void undoLastSimplification(TriMesh& _mesh, std::stack<TriMesh>& _meshHistory)
{
    if (!_meshHistory.empty())
    {
        _mesh = _meshHistory.top();
        _meshHistory.pop();
        rebuildPolyscopeMesh(_mesh);
    }
}

/**
 * @brief Resets the mesh to its original state by reloading it from a file.
 *
 * @param _mesh The mesh object to reset.
 * @param _meshHistory A stack storing the history of mesh states, which is cleared during reset.
 * @param _filename The file from which to reload the mesh.
 *
 * This function clears the history stack and reloads the original mesh
 * from the specified file, then updates the Polyscope display.
 */
void resetMesh(TriMesh& _mesh, std::stack<TriMesh>& _meshHistory, const std::string& _filename)
{
    if (Parser::loadMesh(_filename, _mesh))
    {
        while (!_meshHistory.empty())
        {
            _meshHistory.pop();
        }
        rebuildPolyscopeMesh(_mesh);
    }
}

/**
 * @brief Displays statistics about the current mesh in the Polyscope GUI.
 *
 * @param _mesh The mesh object whose statistics are displayed.
 *
 * This function shows the number of vertices, edges, and faces in the mesh
 * using ImGui within the Polyscope interface.
 */
void displayMeshStatistics(const TriMesh& _mesh)
{
    ImGui::Text("Mesh Statistics");
    ImGui::Text("Vertices: %zu", _mesh.n_vertices());
    ImGui::Text("Edges: %zu", _mesh.n_edges());
    ImGui::Text("Faces: %zu", _mesh.n_faces());
}

/**
 * @brief Main entry point of the program.
 *
 * This function initializes the Polyscope viewer, loads an initial mesh,
 * and sets up the user interface for file selection, simplification, and
 * visualization. It handles user interaction and dynamically updates the
 * Polyscope display based on user actions.
 *
 * @return `EXIT_SUCCESS` if the program completes successfully; otherwise, `EXIT_FAILURE`.
 */
int main()
{
    TriMesh mesh;
    std::string filename = "../object-files/gourd.obj";

    if (!Parser::loadMesh(filename, mesh))
    {
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

    polyscope::state::userCallback = [&mesh, &filename]()
    {
        static bool isFileDialogOpen = false;
        static float simplifyPercentage = 10.0f; // Default to 10%
        static std::stack<TriMesh> meshHistory;  // Stack to hold mesh states for undo

        // Select file
        ImGui::Text("Select File");
        if (ImGui::Button("Open File Explorer"))
        {
            isFileDialogOpen = true; // Set flag to open dialog
        }
        openFileExplorer(mesh, isFileDialogOpen, filename);

        ImGui::Text("Simplification Settings");
        ImGui::SliderFloat("Percentage to Simplify", &simplifyPercentage, 0.0f, 100.0f, "%.1f%%");
        ImGui::InputFloat("Set Percentage", &simplifyPercentage, 5.0f, 10.0f, "%.1f");
        simplifyPercentage = std::clamp(simplifyPercentage, 0.0f, 100.0f);

        // Simplify
        if (ImGui::Button("Simplify Mesh"))
        {
            simplifyMeshByPercentage(mesh, meshHistory, simplifyPercentage);
        }

        // Undo
        if (ImGui::Button("Undo Last Simplification")) {
            undoLastSimplification(mesh, meshHistory);
        }

        // Reset
        if (ImGui::Button("Reset Mesh"))
        {
            resetMesh(mesh, meshHistory, filename);
        }

        // Mesh Statistics
        displayMeshStatistics(mesh);
    };

    polyscope::show();
    std::cout << "Polyscope viewer initialised!\n";

    return EXIT_SUCCESS;
}
