#pragma once

#include <cmath>
#include <cstdlib>
#include <limits>

#include <openvdb/openvdb.h>

namespace ffa
{

// Exact voxel traversal (Amanatides & Woo) for line-of-sight checks on the
// cell-centered VDB grid (voxel i spans [i-0.5, i+0.5) in index space).
//
// Replaces the former max-axis SAMPLED line used by
// {ObserveAstar,FovAstar}::rayTraceClear* and rayTraceClearKnownOcc: sampling
// one rounded voxel per longest-axis step skips voxels the segment actually
// crosses between consecutive samples, so a near-tangent ray could thread a
// KNOWN-OCCUPIED thin inclined slab (stair flights, window transoms) and
// certify a physically blind viewpoint (stairwell_scan debugging, 2026-08-13;
// same class as the scene3 "rounded-DDA thin-sheet skip"). This visits every
// crossed voxel.
//
// Semantics preserved from the samplers: the START and END voxels are exempt
// (the end voxel is the observation target itself, typically unknown), and
// `blocked` decides what obstructs (unknowns stay transparent at the caller).
// Returns true if no interior voxel blocks.
template <typename BlockedFn>
inline bool rayTraceVisitClear(const openvdb::math::Transform &grid_tf,
                               const openvdb::Vec3d &start_world,
                               const openvdb::Vec3d &end_world,
                               BlockedFn &&blocked)
{
    const openvdb::Vec3d p0 = grid_tf.worldToIndex(start_world);
    const openvdb::Vec3d p1 = grid_tf.worldToIndex(end_world);
    openvdb::Coord c = openvdb::Coord::round(p0);
    const openvdb::Coord c_end = openvdb::Coord::round(p1);
    if (c == c_end)
    {
        return true;
    }

    const openvdb::Vec3d d = p1 - p0;
    int step[3];
    double t_max[3];
    double t_delta[3];
    for (int a = 0; a < 3; ++a)
    {
        if (d[a] > 0.0)
        {
            step[a] = 1;
            t_delta[a] = 1.0 / d[a];
            t_max[a] = (static_cast<double>(c[a]) + 0.5 - p0[a]) / d[a];
        }
        else if (d[a] < 0.0)
        {
            step[a] = -1;
            t_delta[a] = -1.0 / d[a];
            t_max[a] = (static_cast<double>(c[a]) - 0.5 - p0[a]) / d[a];
        }
        else
        {
            step[a] = 0;
            t_delta[a] = std::numeric_limits<double>::infinity();
            t_max[a] = std::numeric_limits<double>::infinity();
        }
    }

    // A segment crosses at most |dx|+|dy|+|dz| voxel boundaries; the guard is
    // unreachable for finite inputs and only protects against numeric freak
    // cases (matching the old sampler's permissive fallthrough).
    int guard = std::abs(c_end.x() - c.x()) + std::abs(c_end.y() - c.y()) +
                std::abs(c_end.z() - c.z()) + 3;
    while (guard-- > 0)
    {
        int a = 0;
        if (t_max[1] < t_max[a])
        {
            a = 1;
        }
        if (t_max[2] < t_max[a])
        {
            a = 2;
        }
        c[a] += step[a];
        t_max[a] += t_delta[a];
        if (c == c_end)
        {
            return true;
        }
        if (blocked(c))
        {
            return false;
        }
    }
    return true;
}

}  // namespace ffa
