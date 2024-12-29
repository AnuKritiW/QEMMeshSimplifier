[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/RM1pL2Qm)

# Surface Simplification Using Quadric Error Metrics

\[Paper found [here](./References/Surface%20Simplification%20Using%20Quadric%20Error%20Metrics.pdf)\]

* Produce high quality approximations of polygonal models
* Iterative contractions of vertex pairs to simplify models
* Maintains surface error approximations using quadric matrices

* Algorithm:
    1. Compute the **Q** matrices for all initial vertices
        - **Q** matrices are a 4x4 error matrix that is computed using a  heuristic given by Ronfard and Rossignac
    2. Select all valid pairs
    3. Compute the optimal contraction target <strong><span style="text-decoration:overline;">v</span></strong> for each valid pair <strong>(v<sub>1</sub>, v<sub>2</sub>)</strong>. The error <strong>
  <span style="text-decoration:overline;">v</span><sup>T</sup> (Q<sub>1</sub> + Q<sub>2</sub>) <span style="text-decoration:overline;">v</span></strong> of this target vertex becomes the _cost_ of contracting that pair.
    4. Place all the pairs in a heap keyed on cost with the minimum cost pair at the top
    5. Iteratively remove the pair <strong>(v<sub>1</sub>, v<sub>2</sub>)</strong> of least cost from the heap, constract this pair, and update the costs of all valid pairs involving <strong>v<sub>1</sub></strong>
</strong>

* Psuedo-ish code
```
Function simplifyMesh(mesh, targetFaces):
    # Precompute vertex quadrics
    computeQuadrics(mesh)

    # Initialize priority queue (min-heap) for edges
    Declare priority queue, pq
    For each edge e in the mesh:
        computeEdgeCollapseCost(mesh, e) --> optimal position; collapse cost
        Create EdgeInfo(edgeHandle, cost, optpos)
        pq.insert(EdgeInfo)

    # Simplify mesh
    While (numFaces in mesh > targetFaces) && (!pq.empty()):
        # Pop the edge with the smallest cost
        topEdge = pq.pop()

        # Attempt to collapse the edge
        If !collapseEdge(mesh, topEdge.edgeHandle, topEdge.optimalPos):
            continue;

        # Rebuild the priority queue after the collapse
        Remove deleted elements
        Clear the priority queue

        For each remaining edge e in the mesh:
            If e is deleted, skip
            Ensure the quadric property exists for vertices
            Compute the cost and optimal position for e using computeEdgeCollapseCost
            Create EdgeInfo object and insert into pq

    # Cleanup
End Function
```