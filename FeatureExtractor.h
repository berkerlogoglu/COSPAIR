/*
 * FeatureExtractor.h
 *
 *  Created on: Dec 1, 2013
 *      Author: berker
 */

#ifndef FEATUREEXTRACTOR_H_
#define FEATUREEXTRACTOR_H_

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

#include "COSPAIR.h"

class FeatureExtractor {
public:
	FeatureExtractor();
	virtual ~FeatureExtractor();
	int ExtractFeatures(std::string pointcloud_list, bool write=false);
	cv::Mat TrainBOW(int numofcenters, const cv::Mat& descriptors);
	cv::Mat ExtractBOWHistograms(std::string vocabulary_name, std::vector<cv::Mat> &descriptors);
	void ConfigFeatures();

	int ExtractCOSPAIR(std::string pointcloud_list, double R=0.05, int levels=2, int bins=9, int rgb_type=0, int rgb_bins=5, int keypointType=2, float sampling=0.0);
	std::vector<std::vector<float> > ComputeCOSPAIR(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double R, int levels, int bins, int rgb_type, int rgb_bins, int keypointType, float sampling);
	float SafeAcos (float x);
	void RGB2HSV(float r, float g, float b, float &h, float &s, float &v);
	double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
	double computeCloudSize (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);



private:
	bool Flag_VFH;
	bool Flag_GFPFH;
	bool Flag_CVFH;
	bool Flag_OURCVFH;
	bool Flag_ESF;
	bool Flag_PFH;
	bool Flag_PFHRGB;
	bool Flag_FPFH;
	bool Flag_SHOT;
	bool Flag_SHOTColor;
	bool Flag_3DSC;
	bool Flag_USC;
	float nn_search_radius;
	float support_radius;
	float harris_radius;
	std::string config_features_file;
	std::vector<bool> flags;
	std::vector<std::string>  feature_names;
	bool Flag_Normals;
	int KeypointsType;
	float SamplingDistance;
	int BOP_K;
	int writeBOP(std::string filename, std::vector<std::vector<bool> > features);
	int writeSPFH(std::string filename, std::vector<std::vector<float> > features);

};

#endif /* FEATUREEXTRACTOR_H_ */
