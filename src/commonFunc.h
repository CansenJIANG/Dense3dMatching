#ifndef COMMONFUNC_H
#define COMMONFUNC_H
#include "commonHeader.h"

namespace commonFunc
{
    // overload func to calculate l2 norm
    inline float l2norm(float x, float y, float z)
    {   return std::sqrt(x*x + y*y +z*z);   }

    inline float l2norm(float x, float y)
    {   return std::sqrt(x*x + y*y);    }

    inline float l2norm(std::vector<float> vec)
    {
        int i= vec.size();
        double sum = 0.0;
        while(i--)
        {   sum += vec.at(i)*vec.at(i);    }
        return std::sqrt(sum);
    }

    // return median value
    template<typename T>
    T getMedian(std::vector<T> &input);

    // func to generat radom sample indices of ransac

    inline void randomIdx(u16 randIdx[], u16 idxRange)
    {
        bool stop = false;

        /* initialize random seed: */
        std::srand (std::time(NULL));

        /* generate random number from 0 to idxRange */
        s16 randNum = std::rand() % idxRange;
        randIdx[0] = (u16) randNum;
        while(!stop)
        {
            std::srand (std::time(NULL));
            randNum = std::rand() % idxRange;
            if(randIdx[0] != (u16) randNum)
            {
                randIdx[1] = (u16) randNum;
                stop = true;
            }
        }
        while(stop)
        {
            std::srand (std::time(NULL));
            randNum = std::rand() % idxRange;
            if(randIdx[0] != (u16) randNum && randIdx[1] != (u16) randNum)
            {
                randIdx[2] = (u16) randNum;
                stop = false;
            }
        }
    }


    inline bool sort_triplet(const triplet<s16, s16, f32> &left,
                             const triplet<s16, s16, f32> &right)
    {
        return left.matchDist < right.matchDist;
    }

    inline bool unique_triplet( const triplet<s16, s16, f32> &left,
                                const triplet<s16, s16, f32> &right)
    {
        return ( (left.idxRef == right.idxRef) &
                 (left.idxMot == right.idxMot) &
                 (left.matchDist == right.matchDist) );
    }
}

///////////////////////////////////
/// Median Value Estimation
///////////////////////////////////
// Calculate the median value of matching pair distance;
template<typename T>
T getMedian(std::vector<T> &input)
{
    T median;
    std::vector<T> scores;
    for(int i=0;i<input.size();i++)
    {
        scores.push_back(input[i]);
    }
    size_t size = scores.size();

    std::sort(scores.begin(), scores.end());

    if (size  % 2 == 0)
    {
        median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
    }
    else
    {
        median = scores[size / 2];
    }
    return median;
}


#endif // COMMONFUNC_H
