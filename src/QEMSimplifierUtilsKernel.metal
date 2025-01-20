#include <metal_stdlib>
using namespace metal;

typedef atomic_float atomic_float4x4[16];

// Simple struct for face indices
struct Face
{
    ushort v0;
    ushort v1;
    ushort v2;
};

// Pass 1: compute the face quadric
kernel void computeFaceQuadricKernel( const device Face*   faces        [[ buffer(0) ]],
                                      const device float3* vertexPos    [[ buffer(1) ]],
                                      device float4x4*     faceQuadrics [[ buffer(2) ]],
                                      constant uint&       numFaces     [[ buffer(3) ]],
                                      device float4x4* debugLogBuffer [[buffer(4)]],
                                      uint                 gid          [[ thread_position_in_grid ]] )
{
    if (gid >= numFaces) return;

    Face f = faces[gid];
    float3 p0 = vertexPos[f.v0];
    float3 p1 = vertexPos[f.v1];
    float3 p2 = vertexPos[f.v2];

    // Normal
    float3 n = normalize(cross(p1 - p0, p2 - p0));
    float d = -dot(n, p0);
    float4 plane = float4(n, d);

    // Outer product: plane * plane^T
    float4x4 Kp;
    for (uint r = 0; r < 4; r++)
    {
        for (uint c = 0; c < 4; c++)
        {
            Kp[r][c] = plane[r] * plane[c];
        }
    }

    debugLogBuffer[gid] = Kp;

    faceQuadrics[gid] = Kp;
}

kernel void accumulateQuadricsKernel( const device Face*      faces           [[buffer(0)]],
                                      const device float4x4*  faceQuadrics    [[buffer(1)]],
                                      device atomic_float4x4* vertexQuadrics  [[buffer(2)]],
                                      constant uint&          numFaces        [[buffer(3)]],
                                      uint                    gid             [[thread_position_in_grid]])
{
    if (gid >= numFaces) return;

    // Get current face and its quadric
    Face f = faces[gid];
    float4x4 Kp = faceQuadrics[gid];

    // Atomically accumulate Kp into each vertex's quadric
    for (uint r = 0; r < 4; ++r)
    {
        for (uint c = 0; c < 4; ++c)
        {
            uint index = r * 4 + c; // Flatten row-column index
            atomic_fetch_add_explicit(&vertexQuadrics[f.v0][index], Kp[r][c], memory_order_relaxed);
            atomic_fetch_add_explicit(&vertexQuadrics[f.v1][index], Kp[r][c], memory_order_relaxed);
            atomic_fetch_add_explicit(&vertexQuadrics[f.v2][index], Kp[r][c], memory_order_relaxed);
        }
    }
}
