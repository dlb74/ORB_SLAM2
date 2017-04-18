/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "Converter.h"

PointCloudMapping::PointCloudMapping(double resolution_)
{
    if (resolution_ != 0){
        
        //this->resolution = resolution_;
    }
    
    //voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );
    
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    //cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m++ )
    {
        for ( int n=0; n<depth.cols; n++ )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.2 || d>3)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
//            p.r = color.ptr<uchar>(m)[n*3];
//            p.g = color.ptr<uchar>(m)[n*3+1];
//            p.b = color.ptr<uchar>(m)[n*3+2];

            //p.b = 255;
            //p.g = 255;
            //p.r = 255;

            tmp->points.push_back(p);
        }
    }
    
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = true;
    
    //cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}





//void PointCloudMapping::cloudPublish(PointCloud::Ptr cloud)
//{

//    ros::NodeHandle node_;
//    output_pub_ = node_.advertise<PointCloud> ("/scottCloud/pointcloud", 100);

//    //translateHorizental(globalMap);

//    output_pub_.publish(*cloud);
//}





void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");

    globalMap->header.frame_id = "base_link";

    //cv::namedWindow( "Current Depth Frame" );

    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        
        PointCloud::Ptr p;
        PointCloud::Ptr tmp(new PointCloud());
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;



        }

        //cv::Mat im;
        //depthImgs[0].copyTo(im);

        //cout<<"depthImgs[0]: "<<depthImgs[0]<<endl;
        //cv::imshow("Current Depth Frame", im);
        //cv::waitKey(0);

        //voxel.setInputCloud( globalMap );
        //voxel.setLeafSize( resolution, resolution, resolution);
        //voxel.filter( *tmp );
        //globalMap->swap( *tmp );


        viewer.showCloud( globalMap );
//        cout<<"\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n KeyFrame: \n"<< keyframes[N-1]->GetCameraCenter() <<"\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n " <<endl;
//        cout<<"show global map, size="<<globalMap->points.size()<<endl;




        lastKeyframeSize = N;

//        cloudPublish(globalMap);

//        broadcaster.sendTransform(
//              tf::StampedTransform(
//                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
//                ros::Time::now(),"base_link", "camera_depth_optical_frame"));

    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMapping::GetGlobalMap()
{
    unique_lock<mutex> lck(keyframeMutex);
    return globalMap;
}

//KeyFrame PointCloudMapping::GetKeyFrame()
//{
//    //unique_lock<mutex> lock(mMutexMode);
//    return keyframes[0];
//}


vector<KeyFrame*> PointCloudMapping::GetKeyFrames()
{
    unique_lock<mutex> lck(keyframeMutex);
    return keyframes;
}







