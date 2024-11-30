[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/RM1pL2Qm)

# Navigating Intrinsic Triangulations

\[Paper found [here](./References/NavigatingIntrinsicTriangulations/Navigating%20Intrinsic%20Triangulations.pdf); project page found [here](http://www.cs.cmu.edu/~kmcrane/Projects/NavigatingIntrinsicTriangulations/index.html)\]

* Present a **signpost data structure** that makes it easy to run computational geometry algorithms on **poor-quality surface meshes** (e.g. meshes with skinny triangles)
* Operates as a black-box -- eliminating need to change the geometry for processing
* Method considers **intrinsic triangulation** which connect vertices by straight paths along the exact geometry of the input mesh
* **Signpost data structure** stores the direction and distance to each neighbouring vertex.

<img src="./References/NavigatingIntrinsicTriangulations/Signpost.png" width="350" />
<img src="./References/NavigatingIntrinsicTriangulations/signpost_intrinsic_triangulation.jpg" width="450" />

<img src="./References/NavigatingIntrinsicTriangulations/InitSignpostMesh_Algo4.png" width="550" />

* the **Signpost data structure** supports the following queries:
    1. Tracing query
        - follows a signpost to its destination
        - e.g. used when intializing the data structure, edge flips, vertex insertions, etc.\
        <img src="./References/NavigatingIntrinsicTriangulations/TraceFromVertex_Algo2.png" width="550" />
    2. Signpost update
        - updates the direction of a single signpost
        - essentially unfolding triangles along the path and drawing a straight line; can be carried out in a local 2D coordinate system
        ```
        1. each vertex of a path is found by computing 2D ray-line intersections and moving to the closest intersection point.
        2. direction of the ray is transformed into the coord system of the next triangle (by constructing a vector that makes the same angle with the shared edge)
        3. repeat (1) and (2)
        4. final output is the barycentric coordinates of the point q as well as a pointer to the triangle containing q
        ```
        <img src="./References/NavigatingIntrinsicTriangulations/UpdateSignpost_Algo1.png" width="550" />
    3. Vertex update
        - combines (1) and (2) to update the signposts around a vertex
        - Many local operations would compute new lengths for edges incident on a single vertex
        - would need to use these new lengths to update other quantities\
        <img src="./References/NavigatingIntrinsicTriangulations/UpdateVertex_Algo3.png" width="550" />
    * These allow all other operations to be implemented

* The paper also explores how many existing operations can then be applied such as:
    * Edge flip
    * Vertex Insertion
    * Vertex Repositioning
* Psuedocodes are provided for these operations as well.

* The **Signpost data structure** is particularly attractive owing to it's 1:1 **correspondence** between points on an intrinsic triangulation and the underlying extrinsic triangulation.
    - can query the relationship directly without building an auxiliary structure
    - makes the **signpost data structure** compatible with sample-based rendering like ray-tracing

* The paper further goes to show how difference classes of retriangulation algorithms can be implemented using the signpost data structure. Namely:
    - Delaunay Flipping
    - Intrinsic Delaunay Refinement (iDR)
    - Intrinsic Optimal Delaunay Triangulation (iODT)

* The paper also discusses how the **signpost data structure** enables a broad range of techniques from computational geometry and scientific computing to be applied in the polyhedral setting. Examples considered include:
    - Steiner Tree Approximation
    - Finite Elements
    - Geodesic Distance
    - Adaptive Mesh Refinement
    - Tangent Vector Field Processing