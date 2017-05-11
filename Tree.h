#ifndef TREE_H
#define TREE_H

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{

class Tree
{

    public:

        cv::Point2f center;

        double radius;

        double height;

        int divPart; //this case is divide by 6 parts



    private:




};


}

#endif // TREE_H


