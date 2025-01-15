
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <Eigen/Dense>
#import <Metal/Metal.h>           // For Metal APIs like MTLDevice, MTLCommandQueue
#import <Foundation/Foundation.h> // For NSString, NSURL, NSError, etc.
#import <simd/simd.h>             // For types like float3 and float4

// A small CPU struct that matches how you expect to send Face data to GPU
struct Face
{
    uint16_t v0;
    uint16_t v1;
    uint16_t v2;
};

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;
typedef Eigen::Matrix<double, 4, 4> QMatrix;

// Initialize the device & command queue
static id<MTLDevice>               gDevice             = MTLCreateSystemDefaultDevice(); // selects the default GPU
static id<MTLCommandQueue>         gCommandQueue       = [gDevice newCommandQueue];      // manages execution of Metal tasks
static id<MTLLibrary>              gLibrary            = nil;                            // contains precompiled metal shader code
static id<MTLComputePipelineState> faceQuadricPipeline = nil;                            // defines how metal executes the computeFaceQuadricKernel
static id<MTLComputePipelineState> accumulatePipeline  = nil;                            // defines how metal executes the accumulateQuadricsKernel (vertices)

void initializeMetal()
{
    // Only initialize gLibrary once
    if (gLibrary) return;

    NSString* metallibPath = [NSString stringWithUTF8String:"${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib"];
    NSError* err = nil;

    NSURL* metallibURL = [NSURL fileURLWithPath:metallibPath];
    gLibrary = [gDevice newLibraryWithURL:metallibURL error:&err];
    if (!gLibrary || err)
    {
        NSLog(@"Failed to load Metal library: %@", err.localizedDescription);
        return;
    }

    // TODO: consider should this be moved down?
    // Build pipelines
    id<MTLFunction> f1  = [gLibrary newFunctionWithName:@"computeFaceQuadricKernel"];
    faceQuadricPipeline = [gDevice newComputePipelineStateWithFunction:f1 error:&err];

    id<MTLFunction> f2 = [gLibrary newFunctionWithName:@"accumulateQuadricsKernel"];
    accumulatePipeline = [gDevice newComputePipelineStateWithFunction:f2 error:&err];

    if (err || !faceQuadricPipeline || !accumulatePipeline)
    {
        fprintf(stderr, "Failed to create compute pipeline: %s\n", [[err localizedDescription] UTF8String]);
        return;
    }
}

/*
1. Send _mesh data to the GPU.
2. Execute the computeFaceQuadricKernel on the GPU.
3. Retrieve results and populate _globalQuadrics.
*/
extern "C" void computeQuadricsInParallel_Metal(TriMesh& _mesh, std::vector<QMatrix>& _globalQuadrics)
{
    // validate
    const size_t numFaces = _mesh.n_faces();
    const size_t numVerts = _mesh.n_vertices();
    if (numFaces == 0 || numVerts == 0) return;

    initializeMetal();

    // validate
    if (!gLibrary || !faceQuadricPipeline) return;
    NSLog(@"Metal library and compute pipeline initialized successfully.");

    // Convert FaceHandle data to primitive types (ushort indices) for GPU compatibility.
    // GPUs handle tightly packed primitives efficiently. They cannot handle complex types like FaceHandle.
    // Store the actual vertex positions to get from the index
    std::vector<Face> facesVec(numFaces);
    std::vector<simd::float3> vertPos(numVerts);
    std::vector<bool> visitedVerts(numVerts, false); // track visited vertices

    int i = 0;
    for (auto f_it = _mesh.faces_begin(); f_it != _mesh.faces_end(); ++f_it, ++i)
    {
        auto fh = *f_it;
        std::vector<TriMesh::VertexHandle> fvs;
        for (auto fv_it = _mesh.fv_iter(fh); fv_it.is_valid(); ++fv_it)
        {
            auto vh = *fv_it;
            fvs.push_back(vh);

            // if not already visited, populate vertPos
            if (!visitedVerts[vh.idx()])
            {
                auto p = _mesh.point(vh);
                vertPos[vh.idx()] = {p[0], p[1], p[2]};
                visitedVerts[vh.idx()] = true;
            }
        }

        facesVec[i].v0 = (ushort)fvs[0].idx();
        facesVec[i].v1 = (ushort)fvs[1].idx();
        facesVec[i].v2 = (ushort)fvs[2].idx();
    }

    // buffers store the results of GPU computations
    std::vector<simd::float4x4> faceQuadrics(numFaces, simd::float4x4(0.0f)); // stores the Kp
    std::vector<simd::float4x4> vertexQuadrics(numVerts, simd::float4x4(0.0f));

    // Metal buffers that transfer data to and from the GPU
    id<MTLBuffer> facesBuf    = [gDevice newBufferWithBytes:facesVec.data()
                                 length:facesVec.size() * sizeof(Face)
                                 options:MTLResourceStorageModeShared];

    id<MTLBuffer> vertPosBuf  = [gDevice newBufferWithBytes:vertPos.data()
                                 length:vertPos.size() * sizeof(simd::float3)
                                 options:MTLResourceStorageModeShared];

    id<MTLBuffer> faceQuadBuf = [gDevice newBufferWithBytes:faceQuadrics.data()
                                 length:faceQuadrics.size() * sizeof(simd::float4x4)
                                 options:MTLResourceStorageModeShared];

    id<MTLBuffer> vertQuadBuf = [gDevice newBufferWithBytes:vertexQuadrics.data()
                                 length:vertexQuadrics.size() * sizeof(simd::float4x4)
                                 options:MTLResourceStorageModeShared];
    // id<MTLBuffer> vertexQuadBuf = [gDevice newBufferWithLength:numVerts * sizeof(atomic_float) * 16
    //                                                options:MTLResourceStorageModeShared];

    // For the uniform data like numFaces, using a "constant" avoids the overhead of creating and managing a buffer
    uint faceCount = (uint)numFaces;

    // command buffer & encoder for pass 1 -- face quadrics
    id<MTLCommandBuffer> cmdBuf = [gCommandQueue commandBuffer];
    id<MTLComputeCommandEncoder> encoder = [cmdBuf computeCommandEncoder];
    [encoder setComputePipelineState:faceQuadricPipeline];

    // set buffers (the argument indices match .metal kernel)
    [encoder setBuffer:facesBuf       offset:0 atIndex:0];                  // which verts form each face
    [encoder setBuffer:vertPosBuf     offset:0 atIndex:1];                  // positions of verts
    [encoder setBuffer:faceQuadBuf    offset:0 atIndex:2];                  // kernel writes computed face quadrics into this
    [encoder setBytes:&faceCount      length:sizeof(faceCount) atIndex:3];  // constant face count

    // launch
    MTLSize gridSize           = MTLSizeMake(numFaces, 1, 1);                       // one thread per face
    NSUInteger threadgroupSize = faceQuadricPipeline.maxTotalThreadsPerThreadgroup; // num threads per thread group
    if (threadgroupSize > numFaces)
    {
        threadgroupSize = numFaces;
    }

    [encoder dispatchThreadgroups:MTLSizeMake((numFaces + threadgroupSize - 1)/threadgroupSize, 1, 1)
          threadsPerThreadgroup:MTLSizeMake(threadgroupSize, 1, 1)];

    [encoder endEncoding];


    // accumulate into the vertices for pass 2
    id<MTLComputeCommandEncoder> encoder2 = [cmdBuf computeCommandEncoder];
    [encoder2 setComputePipelineState:accumulatePipeline];

    [encoder2 setBuffer:facesBuf       offset:0 atIndex:0];
    [encoder2 setBuffer:faceQuadBuf    offset:0 atIndex:1];
    [encoder2 setBuffer:vertQuadBuf    offset:0 atIndex:2];
    [encoder2 setBytes:&faceCount      length:sizeof(faceCount) atIndex:3];

    [encoder2 dispatchThreadgroups:MTLSizeMake((numFaces + threadgroupSize - 1)/threadgroupSize, 1, 1)
            threadsPerThreadgroup:MTLSizeMake(threadgroupSize, 1, 1)];

    [encoder2 endEncoding];


    [cmdBuf commit];
    [cmdBuf waitUntilCompleted];

    // Read back results
    simd::float4x4* finalVerts = (simd::float4x4*)vertQuadBuf.contents;
    // Copy into _globalQuadrics
    for (size_t v = 0; v < numVerts; ++v)
    {
        Eigen::Matrix4d& M = _globalQuadrics[v];
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                M(r, c) = (double)finalVerts[v].columns[c][r];
            }
        }
    }
}