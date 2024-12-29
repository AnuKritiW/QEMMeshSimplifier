#include "QEMSimplifier.h"
#include <Eigen/Dense>

static OpenMesh::VPropHandleT<Eigen::Matrix4d> vQuadric;

void QEMSimplifier::computeQuadrics(TriMesh& mesh)
{
    if (!mesh.get_property_handle(vQuadric, "v:quadric"))
    {
        mesh.add_property(vQuadric, "v:quadric");

        // Initialize quadrics to zero for all vertices
        for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
        {
            mesh.property(vQuadric, *v_it) = Eigen::Matrix4d::Zero();
        }
    }
}