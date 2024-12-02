#include <cstdlib>
#include <iostream>

#include "parser.h"

int main() {
    TriMesh mesh;
    std::string filename = "object-files/icosahedron.obj";

    if (!Parser::loadMesh(filename, mesh)) {
        return EXIT_FAILURE;
    }

    std::cout << "Mesh loaded successfully!" << std::endl;

    // Example: Print vertex information
    for (auto vertex : mesh.vertices()) {
        auto point = mesh.point(vertex);
        std::cout << "Vertex " << vertex.idx() << ": ("
                  << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
    }

    return EXIT_SUCCESS;
}
