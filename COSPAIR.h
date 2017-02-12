/*
 * COSPAIR.h
 *
 *  Created on: Feb 12, 2017
 *      Author: berker
 */

#ifndef COSPAIR_H_
#define COSPAIR_H_

#include <iostream>
#include <string>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <vector>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/point_picking_event.h>

//OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>




class COSPAIR {
public:
	COSPAIR();
	virtual ~COSPAIR();

	std::vector<std::vector<float> > ComputeCOSPAIR(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double R, int levels, int bins, int rgb_type, int rgb_bins, int keypointType, float sampling);
	float SafeAcos (float x);
	void RGB2HSV(float r, float g, float b, float &h, float &s, float &v);
	double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
	double computeCloudSize (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


private:
	float harris_radius;

};

#endif /* COSPAIR_H_ */
