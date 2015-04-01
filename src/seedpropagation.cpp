#include "seedpropagation.h"

seedPropagation::seedPropagation()
{
}


/////////////////////////////////////////////////////////////////////////////////////
/// Func to search Knn neighbors from seeds
/////////////////////////////////////////////////////////////////////////////////////
void seedPropagation::getKnnNearestK(const PointCloudT::Ptr &ptQuery,
                                     const PointCloudT::Ptr &cloud,
                                     std::vector<s16> &neighIdx,
                                     std::vector<f32> &neighDist)
{
    pcl::KdTreeFLANN<PointT> kdTree;
    kdTree.setInputCloud(cloud);
    std::vector<s16> knnIdx;
    std::vector<f32> knnDist;
    for(size_t i = 0; i<ptQuery->points.size(); i++)
    {
        PointT pQuery = ptQuery->points.at(i);
        kdTree.nearestKSearch(pQuery, 1, knnIdx, knnDist);
        neighIdx.push_back(knnIdx[0]);
        neighDist.push_back(knnDist[0]);
    }
}

/////////////////////////////////////////////////////////////////////////////////////
/// Func to search Knn neighbors from seeds
/////////////////////////////////////////////////////////////////////////////////////
void seedPropagation::getKnnRadius(const PointCloudT::Ptr &cloud,
                                   const PointCloudT::Ptr &ptQuery,
                                   const f32& searchRadius,
                                   std::vector< std::vector<s16> > &neighIdx)
{
    pcl::KdTreeFLANN<PointT> kdTree;
    kdTree.setInputCloud(cloud);
    std::vector<s16> knnIdx;
    std::vector<f32> knnDist;

    //    std::string fileName = "checkKnnDist.txt";
    //    std::ofstream ofile;
    // create a new file or select the existing files to continue saving selected features
    //    ofile.open(fileName.c_str(), std::ios_base::app);
    for(size_t i = 0; i<ptQuery->points.size(); i++)
    {
        PointT pQuery = ptQuery->points.at(i);
        kdTree.radiusSearch(pQuery, searchRadius, knnIdx, knnDist);
        neighIdx.push_back(knnIdx);
        //        for(size_t j=0; j<knnDist.size();j++)
        //        {
        //            ofile << knnDist.at(j)<<"\n";
        //        }
        //        ofile <<"\n\n";
    }
    //    ofile.close();
}
// overload func with knn distance output
void seedPropagation::getKnnRadius(const PointCloudT::Ptr &cloud,
                                   const PointCloudT::Ptr &ptQuery,
                                   const f32& searchRadius,
                                   std::vector< std::vector<s16> > &neighIdx,
                                   std::vector< std::vector<f32> > &neighDist)
{
    pcl::KdTreeFLANN<PointT> kdTree;
    kdTree.setInputCloud(cloud);
    std::vector<s16> knnIdx;
    std::vector<f32> knnDist;

    //    std::string fileName = "checkKnnDist.txt";
    //    std::ofstream ofile;
    // create a new file or select the existing files to continue saving selected features
    //    ofile.open(fileName.c_str(), std::ios_base::app);
    for(size_t i = 0; i<ptQuery->points.size(); i++)
    {
        PointT pQuery = ptQuery->points.at(i);
        kdTree.radiusSearch(pQuery, searchRadius, knnIdx, knnDist);
        neighIdx.push_back(knnIdx);
        neighDist.push_back(knnDist);
        knnIdx.clear();
        knnDist.clear();
        //        for(size_t j=0; j<knnDist.size();j++)
        //        {
        //            ofile << knnDist.at(j)<<"\n";
        //        }
        //        ofile <<"\n\n";
    }
    //    ofile.close();
}
/////////////////////////////////////////////////////////////////////////////////////
/// Func to get the indexed points from point cloud
/////////////////////////////////////////////////////////////////////////////////////
void seedPropagation::copyIdxPtsFromCloud(const std::vector<s16> &idx,
                                          const PointCloudT::Ptr &cloud,
                                          PointCloudT::Ptr &idxPts)
{
    for(u16 i=0; i<idx.size(); i++)
    {
        idxPts->points.push_back(cloud->points.at(idx[i]));
    }
}

/////////////////////////////////////////////////////////////////////////////////////
/// Func to search cloest point without using knn tree
/////////////////////////////////////////////////////////////////////////////////////
void seedPropagation::matchKnnNeighbors(const PointCloudT::Ptr &knnRef,
                                        const PointCloudT::Ptr &knnMot,
                                        std::vector<s16> &knnIdxRef,
                                        std::vector<s16> &knnIdxMot,
                                        std::vector< triplet<s16, s16, f32> > &newMatches)
{

}


/////////////////////////////////////////////////////////////////////////////////////
/// Func to search cloest point using knn tree
/////////////////////////////////////////////////////////////////////////////////////
void seedPropagation::crossMatching(const std::vector<s16> &idxRef2Mot,
                                    const std::vector<s16> &idxMot2Ref,
                                    const std::vector<f32> &distRef2Mot,
                                    std::vector< triplet<s16, s16, f32> > &newMatches)
{
    for(u16 i=0; i<idxRef2Mot.size(); ++i)
    {
        if( i == idxMot2Ref.at(idxRef2Mot.at(i)) )
        {
            triplet<s16, s16, f32> matchTriplet;
            matchTriplet.idxRef = i;
            matchTriplet.idxMot = idxRef2Mot.at(i);
            matchTriplet.matchDist = distRef2Mot.at(i);
            newMatches.push_back(matchTriplet);
        }
    }
}
// overload func
void seedPropagation::crossMatching(const std::vector<s16> &idxRef2Mot,
                                    const std::vector<s16> &idxMot2Ref,
                                    const std::vector<s16> &knnIdxRef,
                                    const std::vector<s16> &knnIdxMot,
                                    const std::vector<f32> &distRef2Mot,
                                    std::vector< triplet<s16, s16, f32> > &newMatches)
{
    for(u16 i=0; i<idxRef2Mot.size(); ++i)
    {
        if( i == idxMot2Ref.at(idxRef2Mot.at(i)) )
        {
            triplet<s16, s16, f32> matchTriplet;
            matchTriplet.idxRef = knnIdxRef.at(i);
            matchTriplet.idxMot = knnIdxMot.at( idxRef2Mot.at(i) );
            matchTriplet.matchDist = distRef2Mot.at(i);
            newMatches.push_back(matchTriplet);
        }
    }

    std::cout<<"crossMatching size before unique: "<<newMatches.size()<<std::endl;
    std::sort(newMatches.begin(), newMatches.end(), commonFunc::sort_triplet);
    newMatches.erase(std::unique(newMatches.begin(), newMatches.end(),
                                 commonFunc::unique_triplet), newMatches.end());
    std::cout<<"crossMatching size after unique: "<<newMatches.size()<<std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////
/// Func to search cloest point using knn tree
/////////////////////////////////////////////////////////////////////////////////////
void seedPropagation::matchKnnNeighbKdTree(const PointCloudT::Ptr &knnRef,
                                           const PointCloudT::Ptr &knnMot,
                                           std::vector<s16> &knnIdxRef,
                                           std::vector<s16> &knnIdxMot,
                                           std::vector< triplet<s16, s16, f32> > &newMatches)
{
    std::vector<s16> idxRef2Mot,  idxMot2Ref;
    std::vector<f32> distRef2Mot, distMot2Ref;

    // search cloest point in Euclidean Space
    getKnnNearestK(knnRef, knnMot,idxRef2Mot,distRef2Mot);
    getKnnNearestK(knnMot, knnRef,idxMot2Ref,distMot2Ref);

    // cross matching for two sides
    crossMatching(idxRef2Mot, idxMot2Ref, knnIdxRef,
                  knnIdxMot, distRef2Mot, newMatches);
}


/////////////////////////////////////////////////////////////////////////////////////
/// Func to estimate rigid transformation from point correspondences
/////////////////////////////////////////////////////////////////////////////////////
void seedPropagation::getTransformMatrix(const PointCloudT::Ptr &featRef,
                                         const PointCloudT::Ptr &featMot,
                                         Eigen::Matrix4f &transMat)
{
    pcl::TransformationFromCorrespondences transFromCorr;
    for ( size_t i =0;i<featRef->points.size();i++)
    {
        Eigen::Vector3f from(featRef->points.at(i).x,
                             featRef->points.at(i).y,
                             featRef->points.at(i).z);

        Eigen::Vector3f  to (featMot->points.at(i).x,
                             featMot->points.at(i).y,
                             featMot->points.at(i).z);

        transFromCorr.add(from, to, 1.0);//all the same weight
    }
    transMat= transFromCorr.getTransformation().matrix();
}

/////////////////////////////////////////////////////////////////////////////////////
/// Func to match the 3D point clouds  in a local rigion
/////////////////////////////////////////////////////////////////////////////////////
void seedPropagation::localMatching(const PointCloudT::Ptr &cloudRef,
                                    const PointCloudT::Ptr &cloudMot,
                                    const PointCloudT::Ptr &seedRef,
                                    const PointCloudT::Ptr &seedMot,
                                    str_seedPropagation &strSeedPropag)
{
    std::vector< std::vector<s16> > knnIdxRef;
    std::vector< std::vector<s16> > knnIdxMot;

    // get the knn neighbors of the seeds
    getKnnRadius(cloudRef, seedRef, strSeedPropag.searchRadius, knnIdxRef);
    getKnnRadius(cloudMot, seedMot, strSeedPropag.searchRadius, knnIdxMot);

    // First loop for all the seeds
    for(uc8 i=0; i<seedRef->points.size();i++)
    {
        PointCloudT::Ptr idxPtsRef (new PointCloudT);
        PointCloudT::Ptr idxPtsMot (new PointCloudT);

        // Get the knn neighbors from point cloud
        copyIdxPtsFromCloud(knnIdxRef[i], cloudRef, idxPtsRef);
        copyIdxPtsFromCloud(knnIdxMot[i], cloudMot, idxPtsMot);

        // match knn neighbours using KdTree search
        matchKnnNeighbKdTree(idxPtsRef, idxPtsMot, knnIdxRef[i], knnIdxMot[i],
                             strSeedPropag.denseMatches);
    }

}

/////////////////////////////////////////////////////////////////////////////////////
/// Func to propagate the matcing from seeds
/////////////////////////////////////////////////////////////////////////////////////
void seedPropagation::propagateMatching(const PointCloudT::Ptr &cloudRef,                                        
                                        const PointCloudT::Ptr &cloudMot,
                                        PointCloudT::Ptr &seedRef,
                                        PointCloudT::Ptr &seedMot,
                                        str_seedPropagation &strSeedPropag)
{
    std::vector< std::vector<s16> > knnIdxRef;
    std::vector< std::vector<s16> > knnIdxMot;
    std::vector< std::vector<f32> > knnDistRef;
    std::vector< std::vector<f32> > knnDistMot;

    std::cout<<"start dense matching using propagation...\n";

    // take 10% of the best matching as seed for next dense matching
    // how to stop?
    uc8 stopNow = 0;
    while(stopNow<strSeedPropag.propaNumber)
    {
        // get the knn neighbors of the seeds
        getKnnRadius(cloudRef, seedRef, strSeedPropag.searchRadius, knnIdxRef, knnDistRef);
        getKnnRadius(cloudMot, seedMot, strSeedPropag.searchRadius, knnIdxMot, knnDistMot);

        std::cout<<"cloudRef size: "<<cloudRef->points.size()
                <<", cloudMot size: "<<cloudMot->points.size()<<std::endl;

        std::cout<<"seedRef size: "<<seedRef->points.size()
                <<", seedMot size: "<<seedMot->points.size()<<std::endl;

        std::cout<<"get knn radius done.\n";

        // First loop for all the seeds
        for(u16 i=0; i<seedRef->points.size();i++)
        {
            PointCloudT::Ptr idxPtsRef (new PointCloudT);
            PointCloudT::Ptr idxPtsMot (new PointCloudT);

            // Get the knn neighbors from point cloud
            copyIdxPtsFromCloud(knnIdxRef[i], cloudRef, idxPtsRef);
            copyIdxPtsFromCloud(knnIdxMot[i], cloudMot, idxPtsMot);

            std::cout<<"copy knn idxed points done.\n";

            std::cout<<"idxPtsRef size: "<<idxPtsRef->points.size()
                    <<", idxPtsMot size: "<<idxPtsMot->points.size()<<std::endl;

            matchKnnNeighbKdTree(idxPtsRef, idxPtsMot, knnIdxRef[i], knnIdxMot[i],
                                 strSeedPropag.denseMatches);

            std::cout<<"size of new matches is: "<< strSeedPropag.denseMatches.size()<<std::endl;

            std::cout<<"match neighbors kdTree done.\n";
        }

        seedRef->clear();
        seedMot->clear();
        // sort the matching result, propagation only take 10% of best matching
        for(u16 i=0; i<strSeedPropag.denseMatches.size(); i++)
        {
            triplet<s16, s16, f32> tmpTriplet = strSeedPropag.denseMatches.at(i);
            seedRef->push_back(cloudRef->points.at(tmpTriplet.idxRef));
            seedMot->push_back(cloudMot->points.at(tmpTriplet.idxMot));
        }
        std::cout<<"updated size of seeds: "<<seedRef->points.size()<<", "
                <<seedMot->points.size()<<std::endl;
        std::cout<<"sore triplet done.\n";

        //        for(int j=0; j<50; j+=5)
        //        {
        //            std::cout<<strSeedPropag.denseMatches.at(j).matchDist<<std::endl;
        //        }
        ++stopNow;
    }
}
