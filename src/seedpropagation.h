#ifndef SEEDPROPAGATION_H
#define SEEDPROPAGATION_H
#include "scorefunc.h"


/////////////////////////////////////////////////////////////////////////////////////
/// define matching propagation structure
/////////////////////////////////////////////////////////////////////////////////////
struct str_seedPropagation{
    // neighbor search radius
    f32 searchRadius;

    // matching threshold (color, 3d descriptor, motion distance)
    f32 colorThd;
    f32 descrThd;
    f32 motDist;
};

class seedPropagation
{
public:
    seedPropagation();

    // match point cloud in a small region
    void localMatching(const PointCloudT::Ptr& cloudRef, const PointCloudT::Ptr& seedRef,
                       const PointCloudT::Ptr& cloudMot, const PointCloudT::Ptr& seedMot,
                       const str_seedPropagation& strSeedPropag);

    // matching propagation
    void propagateMatching(const PointCloudT::Ptr& cloudRef, const PointCloudT::Ptr& seedRef,
                           const PointCloudT::Ptr& cloudMot, const PointCloudT::Ptr& seedMot,
                           const str_seedPropagation& strSeedPropag);

    // get Knn neighbors
    void getKnnNeighbors(const PointCloudT::Ptr& cloud,
                         const PointCloudT::Ptr &ptQuery,
                         const f32& searchRadius,
                         std::vector< std::vector<s16> > & neighIdx);

    // get the indexed points from point cloud
    void copyIdxPtsFromCloud(const std::vector<s16> &idx,
                             const PointCloudT::Ptr &cloud,
                             PointCloudT::Ptr &idxPts);

    // search cloest matching without using knn tree
    void matchKnnNeighbors(const PointCloudT::Ptr &knnRef, const PointCloudT::Ptr &knnMot,
                           std::vector<s16> knnIdxRef, std::vector<s16> knnIdxMot,
                           const std::vector< triplet<s16, s16, f32> > newMatches);
};

#endif // SEEDPROPAGATION_H
