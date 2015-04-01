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
    // denseMatching result
    std::vector< triplet<s16, s16, f32> > denseMatches;
    PointCloudT::Ptr denseRef;
    PointCloudT::Ptr denseMot;

    // propagation number
    s16 propaNumber;
};

class seedPropagation
{
public:
    seedPropagation();

    // match point cloud in a small region
    void localMatching(const PointCloudT::Ptr &cloudRef, const PointCloudT::Ptr &cloudMot,
                       const PointCloudT::Ptr &seedRef,  const PointCloudT::Ptr &seedMot,
                       str_seedPropagation& strSeedPropag);

    // matching propagation
    void propagateMatching(const PointCloudT::Ptr &cloudRef, const PointCloudT::Ptr &cloudMot,
                           PointCloudT::Ptr &seedRef, PointCloudT::Ptr &seedMot,
                           str_seedPropagation& strSeedPropag);

    // get Knn neighbors Radius search
    void getKnnRadius(const PointCloudT::Ptr &cloud, const PointCloudT::Ptr &ptQuery,
                      const f32 &searchRadius, std::vector< std::vector<s16> > &neighIdx);

    void getKnnRadius(const PointCloudT::Ptr &cloud, const PointCloudT::Ptr &ptQuery,
                      const f32 &searchRadius, std::vector< std::vector<s16> > &neighIdx,
                      std::vector< std::vector<f32> > &neighDist);

    // get Knn neighbors Nearest search
    void getKnnNearestK(const PointCloudT::Ptr &cloud, const PointCloudT::Ptr &ptQuery,
                        std::vector<s16> &neighIdx, std::vector<f32> &neighDist);

    // get the indexed points from point cloud
    void copyIdxPtsFromCloud(const std::vector<s16> &idx,
                             const PointCloudT::Ptr &cloud,
                             PointCloudT::Ptr &idxPts);

    // search cloest matching without using knn tree
    void matchKnnNeighbors(const PointCloudT::Ptr &knnRef, const PointCloudT::Ptr &knnMot,
                           std::vector<s16> &knnIdxRef, std::vector<s16> &knnIdxMot,
                           std::vector< triplet<s16, s16, f32> > &newMatches);

    // search cloest matching using knn tree
    void matchKnnNeighbKdTree(const PointCloudT::Ptr &knnRef, const PointCloudT::Ptr &knnMot,
                              std::vector<s16> &knnIdxRef, std::vector<s16> &knnIdxMot,
                              std::vector< triplet<s16, s16, f32> > &newMatches);

    // cross match knn neighbors with global index
    // of point cloud (search for all point cloud)
    void crossMatching(const std::vector<s16> &idxRef2Mot,
                       const std::vector<s16> &idxMot2Ref,
                       const std::vector<f32> &distRef2Mot,
                       std::vector< triplet<s16, s16, f32> > &newMatches);

    // cross match knn neighbors overload func with
    // local index of point cloud (match for a local region)
    void crossMatching(const std::vector<s16> &idxRef2Mot,
                       const std::vector<s16> &idxMot2Ref,
                       const std::vector<s16> &knnIdxRef,
                       const std::vector<s16> &knnIdxMot,
                       const std::vector<f32> &distRef2Mot,
                       std::vector< triplet<s16, s16, f32> > &newMatches);

    // estimate rigid transformation from point correspondences
    void getTransformMatrix(const PointCloudT::Ptr &featRef,
                            const PointCloudT::Ptr &featMot,
                            Eigen::Matrix4f &transMat);
};

#endif // SEEDPROPAGATION_H
