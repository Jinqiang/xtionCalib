#ifndef SUBS_H
#define SUBS_H

#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <ecl/threads/thread.hpp>
#include <string>
#include <eigen3/Eigen/Core>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <QApplication>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/geometry.h>
#include "FancyQueue.h"
#include <omp.h>
#include "shared/CalibrationMatrix.h"
#include "Plane.h"
#include "fancyViewer.h"
#include "fancyWindow.h"

using namespace sensor_msgs;
using namespace Eigen;
using namespace std;
class FancyWindow;


class myLaserStructure{
public:
    std::vector<float> angles;
    std::vector<float> ranges;
};

class MySubscriber{
    public:
        MySubscriber(FancyViewer*);
        ~MySubscriber();
        void callback(const LaserScanConstPtr &l1, const sensor_msgs::ImageConstPtr &imgPtr);
        void spin();
        FancyQueue* queue;
        float _validNormalRange;
        Eigen::Vector4f planeCoefficient;
        Eigen::Vector4f planeCentroid;
        bool shutdown_required;
        ecl::Thread thread;
        CalibrationMatrix multiplier;
        myLaserStructure laser1;
        float voxelLeaf;
        float normalRejection;
        bool planeModelInliers;
        bool computeRefenceDistance;
        int refenceDistance;

        void LaserScanCleaner(const sensor_msgs::LaserScanConstPtr &src, myLaserStructure &dst);
        void scanToPointcloud(myLaserStructure &scan, pcl::PointCloud<pcl::PointXYZRGB> & cloud);
        pcl::PointCloud<pcl::PointXYZRGB> laserCloud;

        void computePointcloud();
        void voxelize();
        void computeCenterCloud();
        void computerCenterPlane();
        void computeErrorPerPoint();
        void computeCalibrationMatrix();
        void calibratePointCloudWithMultipliers();
        void computeNormals();
        void pointrejection();
        bool applyCorrection;
        bool recordData;

        FancyWindow* _window;

    private:
        cv_bridge::CvImagePtr _image;
        FancyViewer* _viewer;

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> correctCloud;
        pcl::PointCloud<pcl::PointXYZRGB> errorCloud;
        pcl::PointCloud<pcl::PointXYZRGB> centerCloud;
        pcl::PointCloud<pcl::Normal> cloud_normals;
        cv::Vec3f localToWorld(cv::Vec3f localpoint);
        std::vector<bool> validPoints;
        pcl::PointXYZRGB worldToImagePlane( pcl::PointXYZRGB p);
};

#endif
