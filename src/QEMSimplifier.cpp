#include "QEMSimplifier.h"
#include "QEMSimplifierUtils.h"

auto& vQuadric = QEMSimplifierUtils::getQuadricHandle();
auto& vVersion = QEMSimplifierUtils::getVersionHandle();

/**
 * @brief Constructor for the QEMSimplifier class.
 */
QEMSimplifier::QEMSimplifier()
{
}

/**
 * @brief Computes the quadric error matrices for all vertices in the mesh.
 *
 * @param _mesh The input mesh whose vertex quadrics will be calculated.
 *
 * This function initializes quadric and version properties for each vertex,
 * computes face quadrics in parallel, and accumulates the results into
 * vertex quadrics in a single-threaded step.
 */
void QEMSimplifier::computeQuadrics(TriMesh& _mesh)
{
    QEMSimplifierUtils::initializeProperty(_mesh, vQuadric, "v:quadric", QMatrix::Zero().eval());
    QEMSimplifierUtils::initializeProperty(_mesh, vVersion, "v:version", 0);

    // Allocate global storage for reduction
    std::vector<QMatrix> globalQuadrics(_mesh.n_vertices(), QMatrix::Zero());
    QEMSimplifierUtils::computeQuadricsInParallel(_mesh, globalQuadrics);

    // Update mesh properties in a single-threaded step
    for (size_t v_idx = 0; v_idx < _mesh.n_vertices(); ++v_idx)
    {
        _mesh.property(vQuadric, TriMesh::VertexHandle(v_idx)) = globalQuadrics[v_idx];
    }
}

/**
 * @brief Initializes a priority queue with edge collapse costs for the mesh.
 *
 * @param _mesh The input mesh whose edges will be evaluated.
 * @param _priQ A priority queue of edges, sorted by collapse cost in ascending order.
 *              - The priority queue is implemented as:
 *                `std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>>`
 *              - `EdgeInfo` represents the edge data, including collapse cost, optimal position, and versioning information.
 *              - `std::greater<EdgeInfo>` ensures the smallest cost is at the top of the queue.
 *
 * This function calculates the collapse cost and optimal contraction
 * position for each edge in the mesh, and stores the results in the
 * priority queue.
 */
void QEMSimplifier::initializePriorityQueue(TriMesh& _mesh,
                                            std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>>& _priQ)
{
    std::vector<EdgeInfo> edgeInfos;
    edgeInfos.resize(_mesh.n_edges());

    #pragma omp parallel for
    // for every edge, compute the cost and the new pos
    for (size_t idx = 0; idx < _mesh.n_edges(); ++idx)
    {
        auto e_it = std::next(_mesh.edges_begin(), idx);
        if (_mesh.status(*e_it).deleted()) continue;

        Eigen::Vector3d optPos; // optimal position
        float cost = QEMSimplifierUtils::computeEdgeCollapseCost(_mesh, *e_it, optPos);

        auto he0 = _mesh.halfedge_handle(*e_it, 0);
        auto v0  = _mesh.to_vertex_handle(he0);
        auto v1  = _mesh.from_vertex_handle(he0);
        int verSum = QEMSimplifierUtils::mergeVertexProperties(_mesh, v0, v1, vVersion);

        edgeInfos[idx] = EdgeInfo{*e_it, cost, optPos, verSum};
    }

    for (const auto& edgeInfo : edgeInfos)
    {
        if (edgeInfo.edgeHandle.is_valid())
        {
            _priQ.push(edgeInfo);
        }
    }
}

/**
 * @brief Collapses an edge in the mesh to a new vertex position.
 *
 * @param _mesh The mesh on which the edge collapse is performed.
 * @param _edge The edge to be collapsed.
 * @param _newPos The new vertex position after the collapse.
 * @param _vKeep The vertex that will remain after the collapse.
 * @return True if the edge collapse was successful; otherwise, false.
 *
 * This function merges quadrics, updates vertex properties, moves
 * the remaining vertex to the optimal position, and collapses the edge.
 * It ensures that topological constraints are respected.
 */
bool QEMSimplifier::collapseEdge(TriMesh& _mesh, TriMesh::EdgeHandle _edge, const Eigen::Vector3d& _newPos, TriMesh::VertexHandle& _vKeep)
{
    if (!_edge.is_valid()) return false;

    // Pick which halfedge is used.
    // By convention, choose the halfedge whose "to_vertex()" is the one we keep
    // i.e. remove the src vertex and keep the tgt vertex of the chosen halfedge
    TriMesh::HalfedgeHandle he0 = _mesh.halfedge_handle(_edge, 0);
    if (!he0.is_valid()) return false;

    _vKeep = _mesh.to_vertex_handle(he0);
    TriMesh::VertexHandle vRemove = _mesh.from_vertex_handle(he0);

    if (!_mesh.get_property_handle(vQuadric, "v:quadric")) return false;

    // Merge quadrics
    QMatrix QSum = QEMSimplifierUtils::mergeVertexProperties(_mesh, _vKeep, vRemove, vQuadric);

    // Assign to the kept vert
    _mesh.property(vQuadric, _vKeep) = QSum;

    // Move kept vert to the new position
    _mesh.set_point(_vKeep, TriMesh::Point((float)_newPos[0], (float)_newPos[1], (float)_newPos[2]));

    if (!_mesh.get_property_handle(vVersion, "v:version")) return false;
    _mesh.property(vVersion, _vKeep)++;

    // If OpenMesh forbids collapse due to topological constraints, skip
    if (!_mesh.is_collapse_ok(he0)) return false;

    _mesh.collapse(he0);
    return true;
}

/**
 * @brief Recalculates the collapse costs for edges adjacent to a given vertex.
 *
 * @param _mesh The mesh containing the edges to be updated.
 * @param _vKeep The vertex whose adjacent edges will be recalculated.
 * @param _priQ The priority queue to store updated edge collapse costs.
 *
 * This function iterates through all edges adjacent to the given vertex,
 * recalculates their collapse costs and optimal contraction positions,
 * and updates the priority queue.
 */
void QEMSimplifier::recalculateEdgeCosts(TriMesh& _mesh,
                                         TriMesh::VertexHandle _vKeep,
                                         std::priority_queue<EdgeInfo,std::vector<EdgeInfo>, std::greater<EdgeInfo>>& _priQ)
{
    if (_mesh.n_vertices() == 0)
    {
        std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> emptyQueue;
        std::swap(_priQ, emptyQueue);
        return;
    }

    // Temp container for storing updated edges
    std::vector<EdgeInfo> updatedEdges;

    #pragma omp parallel
    {
        // Thread safe temp container
        std::vector<EdgeInfo> localEdges;

        #pragma omp for nowait
        for (int i = 0; i < static_cast<int>(_mesh.valence(_vKeep)); ++i)
        {
            // Get the i-th neighbor of _vKeep
            auto vv_it = std::next(_mesh.vv_iter(_vKeep), i);
            TriMesh::VertexHandle vNeighbor = *vv_it;

            TriMesh::HalfedgeHandle currHe = _mesh.find_halfedge(_vKeep, vNeighbor);
            if (!currHe.is_valid()) continue;

            TriMesh::EdgeHandle currEdge = _mesh.edge_handle(currHe);
            if (!currEdge.is_valid() || _mesh.status(currEdge).deleted()) continue;

            // Recompute cost
            Eigen::Vector3d localOptPos;
            float localCost = QEMSimplifierUtils::computeEdgeCollapseCost(_mesh, currEdge, localOptPos);

            // Store new version sum
            int verSum2 = QEMSimplifierUtils::mergeVertexProperties(_mesh, _vKeep, vNeighbor, vVersion);

            localEdges.push_back(EdgeInfo{currEdge, localCost, localOptPos, verSum2});
        }

        #pragma omp critical
        {
            updatedEdges.insert(updatedEdges.end(), localEdges.begin(), localEdges.end());
        }
    }

    for (const auto& edgeInfo : updatedEdges)
    {
        _priQ.push(edgeInfo);
    }
}

/**
 * @brief Simplifies the mesh to a target number of faces.
 *
 * @param _mesh The input mesh to be simplified.
 * @param _tgtNumFaces The target number of faces in the simplified mesh.
 *
 * This function computes the initial quadrics, initializes a priority
 * queue with edge collapse costs, and iteratively collapses edges with
 * the least cost until the target number of faces is reached. The mesh
 * is cleaned up after simplification.
 */
void QEMSimplifier::simplifyMesh(TriMesh& _mesh, size_t _tgtNumFaces)
{
    if (_mesh.n_faces() <= _tgtNumFaces) return;
    if (_tgtNumFaces < 1) return;

    computeQuadrics(_mesh);

    // tell the mesh to allocate internal “status” flags for vertices, edges, faces, and halfedges.
    // collapse needs status attrib
    // also allows calling of garbage collection at the end.
    _mesh.request_vertex_status();
    _mesh.request_edge_status();
    _mesh.request_face_status();
    _mesh.request_halfedge_status();

    // Build the initial priority queue of edges
    // the one with the lowest cost is always at the top (min-heap)
    std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> priQ;
    initializePriorityQueue(_mesh, priQ);

    unsigned int numFaces = _mesh.n_faces();
    while (numFaces > _tgtNumFaces && !priQ.empty())
    {
        EdgeInfo top = priQ.top();
        priQ.pop();

        if (!top.edgeHandle.is_valid()) continue; // might be stale
        if (_mesh.status(top.edgeHandle).deleted()) continue; // edge might have been collapsed already

        TriMesh::HalfedgeHandle he0 = _mesh.halfedge_handle(top.edgeHandle, 0);
        TriMesh::VertexHandle v0    = _mesh.to_vertex_handle(he0);
        TriMesh::VertexHandle v1    = _mesh.from_vertex_handle(he0);
        int currVerSum = QEMSimplifierUtils::mergeVertexProperties(_mesh, v0, v1, vVersion);

        if (currVerSum != top.versionSum) continue; // stale

        // Check cost in case local neighborhood changed
        Eigen::Vector3d verifyPos;
        float verifyCost = QEMSimplifierUtils::computeEdgeCollapseCost(_mesh, top.edgeHandle, verifyPos);
        if (std::fabs(verifyCost - top.cost) > 1e-7) continue; // stale/changed

        TriMesh::VertexHandle vKeep;
        // collapseEdge needs status attrib
        if (!collapseEdge(_mesh, top.edgeHandle, top.optPos, vKeep)) continue;

        if (_mesh.face_handle(he0).is_valid())
        {
            numFaces--;
        }

        if (_mesh.opposite_face_handle(he0).is_valid())
        {
            numFaces--;
        }

        recalculateEdgeCosts(_mesh, vKeep, priQ);
    }

    // Clean up
    _mesh.garbage_collection();
}