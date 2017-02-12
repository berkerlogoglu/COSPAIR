///*
// * tez_visualise_compare.cpp
// *
// *  Created on: Sep 14, 2014
// *      Author: berker
// */
//
//#include <iostream>
//#include <string>
//#include <vector>
//#include <fstream>
//#include <algorithm>
//#include <ctime>
//#include <cmath>
//
//#include "FeatureExtractor.h"
//#include "Reader.h"
//#include "Matcher.h"
//#include "Evaluater.h"
//
//
////PCL
//#include <boost/thread/thread.hpp>
//#include <pcl/common/common.h>
//#include <pcl/common/distances.h>
//#include <pcl/common/eigen.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/vfh.h>
//#include <pcl/features/pfh.h>
//#include <pcl/features/shot.h>
//#include <pcl/point_types.h>
//#include <pcl/keypoints/iss_3d.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/histogram_visualizer.h>
//#include <pcl/visualization/point_picking_event.h>
//#include <pcl/PointIndices.h>
//#include <pcl/common/angles.h>
//#include <pcl/registration/transforms.h>
//#include <pcl/visualization/pcl_plotter.h>
//#include <pcl/simulation/shape_generator.h>
//
//
//
//
////OPENCV
//#include <opencv2/core/core.hpp>
//#include "opencv2/features2d/features2d.hpp"
//#include <opencv2/ml/ml.hpp>
//
//
//
//int BOP_K = 250;
//struct BOP3hist
//{
//  float histogram[750];
//};
//
//
//struct BOP1hist
//{
//  float histogram[250];
//};
//
//struct rgb
//{
//	int r;
//	int g;
//	int b;
//};
//
//int anglehistsize = 36;
//
//double angle1hist[36];
//double angle2hist[36];
//double angle3hist[36];
//double angle1hist_single[36];
//double angle2hist_single[36];
//double angle3hist_single[36];
//
////for keypoint visualization!!
//pcl::visualization::PCLVisualizer viewer;
//
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_keypoints (new pcl::PointCloud<pcl::Normal>);
//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shifted (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_shifted (new pcl::PointCloud<pcl::PointXYZI>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_rgb_shifted (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::Normal>::Ptr cloud2_normals (new pcl::PointCloud<pcl::Normal>);
//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointXYZI>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_shifted (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints2_shifted (new pcl::PointCloud<pcl::PointXYZI>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_rgb_shifted (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal> harris;
//pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal> harris2;
//
//pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_signature (new pcl::PointCloud<pcl::FPFHSignature33> ());
//pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_signature2 (new pcl::PointCloud<pcl::FPFHSignature33> ());
//pcl::PointCloud<pcl::SHOT352>::Ptr shot_signature (new pcl::PointCloud<pcl::SHOT352> ());
//pcl::PointCloud<pcl::SHOT352>::Ptr shot_signature2 (new pcl::PointCloud<pcl::SHOT352> ());
//pcl::PointCloud<pcl::SHOT1344>::Ptr shotcolor_signature (new pcl::PointCloud<pcl::SHOT1344> ());
//pcl::PointCloud<pcl::SHOT1344>::Ptr shotcolor_signature2 (new pcl::PointCloud<pcl::SHOT1344> ());
//
//std::string testfile1, testfile2;
//
//pcl::PointIndicesConstPtr indices ( new pcl::PointIndices);
//pcl::PointIndicesConstPtr indices2 ( new pcl::PointIndices);
//std::vector<std::vector<bool> >BOP3_features;
//std::vector<std::vector<bool> >BOP3_features2;
//std::vector<std::vector<bool> >BOP1_features;
//std::vector<std::vector<bool> >BOP1_features2;
//
//std::vector<std::vector<float> > SPFH_features;
//std::vector<std::vector<float> > SPFH_features2;
//
//cv::Mat cvBOP_features;
//cv::Mat cvBOP_features2;
//cv::Mat cvBOP1_features;
//cv::Mat cvBOP1_features2;
//cv::Mat cv_SPFH_features;
//cv::Mat cv_SPFH_features2;
//cv::Mat cv_fpfh_signature;
//cv::Mat cv_fpfh_signature2;
//cv::Mat cv_shot_signature;
//cv::Mat cv_shot_signature2;
//cv::Mat cv_shotcolor_signature;
//cv::Mat cv_shotcolor_signature2;
//float f1,f2,f3,f4;
//bool bf1,bf2,bf3;
//std::vector<cv::DMatch> Matches;
//std::vector<std::vector< cv::DMatch> > vv_Matches;
//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr keypoint_tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//
//
//
//float normal_radius = 0.01;
//float harris_radius = 0.01;
//float support_radius= 0.10;
//
//pcl::PointXYZRGB picked_pt;
//std::vector<int> picked_pt_indice(1);
//pcl::PointXYZRGB picked_keypt;
//pcl::PointXYZRGB picked_keypt2;
//std::vector<int> picked_keypt_indice(1);
//
//std::vector<float> picked_pt_BOP3_features;
//cv::Mat picked_pt_cvBOP_features;
//
//
//int point_density = 100000;
//int feature_type;
//int keypoint_type;
//int picked_keypt_ind;
//
//
//int spfh_levels=7;
//int spfh_bins=9;
//double spfh_radius = 0.10;
//
//float SafeAcos (float x)
//  {
//  if (x < -1.0) x = -1.0 ;
//  else if (x > 1.0) x = 1.0 ;
//  return acos (x) ;
//  }
//double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
//{
//  double res = 0.0;
//  int n_points = 0;
//  int nres;
//  std::vector<int> indices (2);
//  std::vector<float> sqr_distances (2);
//  pcl::search::KdTree<pcl::PointXYZRGB> tree;
//  tree.setInputCloud (cloud);
//
//  for (size_t i = 0; i < cloud->size(); i++)
//  {
//    if (pcl::isFinite(cloud->points[i]) == false)
//    {
//      continue;
//    }
//    //Considering the second neighbor since the first is the point itself.
//    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
//    if (nres == 2)
//    {
//      res += sqrt (sqr_distances[1]);
//      ++n_points;
//    }
//  }
//  if (n_points != 0)
//  {
//    res /= n_points;
//  }
//  return res;
//}
//
//int computeBOP1(bool rgb)
//{
//	bool r,g,b;
//	pcl::PointXYZRGB searchPoint;
//	float R = 0.05;
//
//	std::vector<int> pointIdxNKNSearch(BOP_K);
//	std::vector<float> pointNKNSquaredDistance(BOP_K);
//
//	std::vector<int> pointIdxRadiusSearch;
//	std::vector<float> pointRadiusSquaredDistance;
//
//
//	for(int j=0;j<keypoints->size();j++)
//	{
//		if(pcl::isFinite(cloud_normals->points[indices->indices[j]]) == true)
//		{
//			std::vector<bool> features;
//			if(rgb==true)
//				features.reserve(BOP_K*4);
//			else
//				features.reserve(BOP_K);
//
//			searchPoint.x = keypoints->points[j].x;
//			searchPoint.y = keypoints->points[j].y;
//			searchPoint.z = keypoints->points[j].z;
//
//			//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
//			int nanpoints = 0;
//			int K = BOP_K + 1;
//			if ( tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//			{
//				for (size_t i = 1; i < pointIdxNKNSearch.size(); ++i){
//
//					if(rgb==true)
//					{
//						if(cloud->points[pointIdxNKNSearch[i]].r > cloud->points[indices->indices[j]].r)
//							r=1;
//						else
//							r=0;
//						if(cloud->points[pointIdxNKNSearch[i]].g > cloud->points[indices->indices[j]].g)
//							g=1;
//						else
//							g=0;
//						if(cloud->points[pointIdxNKNSearch[i]].b > cloud->points[indices->indices[j]].b)
//							b=1;
//						else
//							b=0;
//					}
//
//					pcl::Vector4fMap p = cloud->points[indices->indices[j]].getVector4fMap();
//					pcl::Vector4fMap np= cloud_normals->points[indices->indices[j]].getNormalVector4fMap();
//					pcl::Vector4fMap q = cloud->points[pointIdxNKNSearch[i] ].getVector4fMap();
//					pcl::Vector4fMap nq= cloud_normals->points[pointIdxNKNSearch[i] ].getNormalVector4fMap();
//					float convex_test;
//					float dot_test ;
//					dot_test = np.dot(nq);
//					convex_test = (p-q).dot(np-nq);
//					//cout<<"convex test :"<<convex_test<<std::endl;
//					//cout<<"dot test :"<<pcl::rad2deg(acos(dot_test))<<std::endl;
//					if(pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]]) == false)
//					{
//						bf3 = 0;
//					}
//					else{
//						bf1 = bool(convex_test < 0);
//						bf2 = bool(pcl::rad2deg(SafeAcos(dot_test)) < 30);
//						bf3 = bf1 && bf2;
//					}
//					//push features for each comparison
//					features.push_back(bf3);
//					if(rgb==true)
//					{
//						features.push_back(r); features.push_back(g); features.push_back(b);
//					}
//				}
//			}
//			BOP1_features.push_back(features);
//
//		}
//		else
//			std::cout<<"NaN keypoint!"<<std::endl;
//	}
//	//cloud 2
//
//
//	pcl::PointXYZRGB searchPoint2;
//
//
//	std::vector<int> pointIdxNKNSearch2(BOP_K);
//	std::vector<float> pointNKNSquaredDistance2(BOP_K);
//
//	std::vector<int> pointIdxRadiusSearch2;
//	std::vector<float> pointRadiusSquaredDistance2;
//
//
//	for(int j=0;j<keypoints2->size();j++)
//	{
//		if(pcl::isFinite(cloud2_normals->points[indices2->indices[j]]) == true)
//		{
//			std::vector<bool> features;
//			if(rgb==true)
//				features.reserve(BOP_K*4);
//			else
//				features.reserve(BOP_K);
//
//			searchPoint2.x = keypoints2->points[j].x;
//			searchPoint2.y = keypoints2->points[j].y;
//			searchPoint2.z = keypoints2->points[j].z;
//
//			//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
//			int nanpoints = 0;
//			int K = BOP_K + 1;
//			if ( tree2->nearestKSearch (searchPoint2, K, pointIdxNKNSearch2, pointNKNSquaredDistance2) > 0 )
//			{
//				for (size_t i = 1; i < pointIdxNKNSearch2.size(); ++i){
//
//					if(rgb==true)
//					{
//						if(cloud2->points[pointIdxNKNSearch2[i]].r > cloud2->points[indices2->indices[j]].r)
//							r=1;
//						else
//							r=0;
//						if(cloud2->points[pointIdxNKNSearch2[i]].g > cloud2->points[indices2->indices[j]].g)
//							g=1;
//						else
//							g=0;
//						if(cloud2->points[pointIdxNKNSearch2[i]].b > cloud2->points[indices2->indices[j]].b)
//							b=1;
//						else
//							b=0;
//					}
//
//					pcl::Vector4fMap p = cloud2->points[indices2->indices[j]].getVector4fMap();
//					pcl::Vector4fMap np= cloud2_normals->points[indices2->indices[j]].getNormalVector4fMap();
//					pcl::Vector4fMap q = cloud2->points[pointIdxNKNSearch2[i] ].getVector4fMap();
//					pcl::Vector4fMap nq= cloud2_normals->points[pointIdxNKNSearch2[i] ].getNormalVector4fMap();
//					float convex_test;
//					float dot_test ;
//					dot_test = np.dot(nq);
//					convex_test = (p-q).dot(np-nq);
//					//cout<<"convex test :"<<convex_test<<std::endl;
//					//cout<<"dot test :"<<pcl::rad2deg(acos(dot_test))<<std::endl;
//
//					if(pcl::isFinite(cloud2_normals->points[pointIdxNKNSearch2[i]]) == false)
//					{
//						bf3 = 0;
//					}
//					else{
//						bf1 = bool(convex_test < 0);
//						bf2 = bool(pcl::rad2deg(SafeAcos(dot_test)) < 30);
//						bf3 = bf1 && bf2;
//					}
//					//push features for each comparison
//					features.push_back(bf3);
//					if(rgb==true)
//					{
//						features.push_back(r); features.push_back(g); features.push_back(b);
//					}
//				}
//			}
//			BOP1_features2.push_back(features);
//
//		}
//		else
//			std::cout<<"NaN keypoint!"<<std::endl;
//	}
//
//	return 1;
//}
//
//int computeBOP3(bool rgb)
//{
//	bool r,g,b;
//	pcl::PointXYZRGB searchPoint;
//	float R = 0.05;
//
//	std::vector<int> pointIdxNKNSearch(BOP_K);
//	std::vector<float> pointNKNSquaredDistance(BOP_K);
//
//	std::vector<int> pointIdxRadiusSearch;
//	std::vector<float> pointRadiusSquaredDistance;
//
//
//	for(int j=0;j<keypoints->size();j++)
//	{
//		if(pcl::isFinite(cloud_normals->points[indices->indices[j]]) == true)
//		{
//			std::vector<bool> features;
//			if(rgb==true)
//				features.reserve(BOP_K*6);
//			else
//				features.reserve(BOP_K*3);
//			searchPoint.x = keypoints->points[j].x;
//			searchPoint.y = keypoints->points[j].y;
//			searchPoint.z = keypoints->points[j].z;
//
//			//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
//			int nanpoints = 0;
//			int K = BOP_K + 1;
//			if ( tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//			{
//				for (size_t i = 1; i < pointIdxNKNSearch.size(); ++i){
//					if(rgb==true)
//					{
//						if(cloud->points[pointIdxNKNSearch[i]].r > cloud->points[indices->indices[j]].r)
//							r=1;
//						else
//							r=0;
//						if(cloud->points[pointIdxNKNSearch[i]].g > cloud->points[indices->indices[j]].g)
//							g=1;
//						else
//							g=0;
//						if(cloud->points[pointIdxNKNSearch[i]].b > cloud->points[indices->indices[j]].b)
//							b=1;
//						else
//							b=0;
//					}
//					pcl::computePairFeatures(cloud->points[indices->indices[j]].getVector4fMap(),cloud_normals->points[indices->indices[j]].getNormalVector4fMap(),cloud->points[pointIdxNKNSearch[i] ].getVector4fMap(),cloud_normals->points[pointIdxNKNSearch[i] ].getNormalVector4fMap(),f1,f2,f3,f4);
//					//std::cout<<pcl::isFinite(cloud->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud_normals->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud->points[pointIdxNKNSearch[i]])<<" "<<pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]])<< std::endl;
//					//std::cout<<"Pair features: "<<f1<<" "<<f2<<" "<<f3<<" "<<f4<<std::endl;
//					//std::cout<<"Pair features: "<< pcl::rad2deg(f1)<<" "<<pcl::rad2deg(acos(f2))<<" "<<pcl::rad2deg(acos(f3))<<" "<<f4<<std::endl;
//
//					if(pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]]) == false)
//					{
//						bf1 = 0;
//						bf2 = 0;
//						bf3 = 0;
//						nanpoints++;
//					}
//					else{
//						//for viewing histogram of angles
//						double a1,a2,a3;
//						int ia1,ia2,ia3;
//						a1  = pcl::rad2deg(f1) + 180;
//						a2  = pcl::rad2deg(SafeAcos(f2));
//						a3  = pcl::rad2deg(SafeAcos(f3));
//						ia1 = int(floor((a1/359)*(anglehistsize-1)));
//						ia2 = int(floor((a2/359)*(anglehistsize-1)));
//						ia3 = int(floor((a3/359)*(anglehistsize-1)));
//						//std::cout<<"Angles: "<<f1<<" "<<f2<<" "<<f3<<" ; "<<a1<<" "<<a2<<" "<<a3<<" ; "<<ia1<<" "<<ia2<<" "<<ia3<<" "<<std::endl;
//						angle1hist[ia1] = angle1hist[ia1] + 1;
//						angle2hist[ia2] = angle2hist[ia2] + 1;
//						angle3hist[ia3] = angle3hist[ia3] + 1;
//						//end
//
//						if((pcl::rad2deg(f1) + 180) > 150)
//							bf1 = 1;
//						else
//							bf1 = 0;
//
//						if(pcl::rad2deg(SafeAcos(f2)) > 90)
//							bf2 = 1;
//						else
//							bf2 = 0;
//
//						if(pcl::rad2deg(SafeAcos(f3)) > 60)
//							bf3 = 1;
//						else
//							bf3 = 0;
//					}
//					//push features for each comparison
//					features.push_back(bf1); features.push_back(bf2); features.push_back(bf3);
//					if(rgb==true)
//					{
//						features.push_back(r); features.push_back(g); features.push_back(b);
//					}
//				}
//				BOP3_features.push_back(features);
//			}
//
//
//		}
//		else
//			std::cout<<"NaN keypoint!"<<std::endl;
//	}
//
//	//cloud 2
//
//
//	pcl::PointXYZRGB searchPoint2;
//
//
//	std::vector<int> pointIdxNKNSearch2(BOP_K);
//	std::vector<float> pointNKNSquaredDistance2(BOP_K);
//
//	std::vector<int> pointIdxRadiusSearch2;
//	std::vector<float> pointRadiusSquaredDistance2;
//
//
//	for(int j=0;j<keypoints2->size();j++)
//	{
//		if(pcl::isFinite(cloud2_normals->points[indices2->indices[j]]) == true)
//		{
//			std::vector<bool> features;
//			if(rgb==true)
//				features.reserve(BOP_K*6);
//			else
//				features.reserve(BOP_K*3);
//
//			searchPoint2.x = keypoints2->points[j].x;
//			searchPoint2.y = keypoints2->points[j].y;
//			searchPoint2.z = keypoints2->points[j].z;
//
//			//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
//			int nanpoints = 0;
//			int K = BOP_K + 1;
//			if ( tree2->nearestKSearch (searchPoint2, K, pointIdxNKNSearch2, pointNKNSquaredDistance2) > 0 )
//			{
//				for (size_t i = 1; i < pointIdxNKNSearch2.size(); ++i){
//					if(rgb==true)
//					{
//						if(cloud2->points[pointIdxNKNSearch2[i]].r > cloud2->points[indices2->indices[j]].r)
//							r=1;
//						else
//							r=0;
//						if(cloud2->points[pointIdxNKNSearch2[i]].g > cloud2->points[indices2->indices[j]].g)
//							g=1;
//						else
//							g=0;
//						if(cloud2->points[pointIdxNKNSearch2[i]].b > cloud2->points[indices2->indices[j]].b)
//							b=1;
//						else
//							b=0;
//					}
//					pcl::computePairFeatures(cloud2->points[indices2->indices[j]].getVector4fMap(),cloud2_normals->points[indices2->indices[j]].getNormalVector4fMap(),cloud2->points[pointIdxNKNSearch2[i] ].getVector4fMap(),cloud2_normals->points[pointIdxNKNSearch2[i] ].getNormalVector4fMap(),f1,f2,f3,f4);
//					//std::cout<<pcl::isFinite(cloud->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud_normals->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud->points[pointIdxNKNSearch[i]])<<" "<<pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]])<< std::endl;
//					//std::cout<<"Pair features: "<<f1<<" "<<f2<<" "<<f3<<" "<<f4<<std::endl;
//					//std::cout<<"Pair features: "<< pcl::rad2deg(f1)<<" "<<pcl::rad2deg(acos(f2))<<" "<<pcl::rad2deg(acos(f3))<<" "<<f4<<std::endl;
//
//					if(pcl::isFinite(cloud2_normals->points[pointIdxNKNSearch2[i]]) == false)
//					{
//						bf1 = 0;
//						bf2 = 0;
//						bf3 = 0;
//						nanpoints++;
//					}
//					else{
//						if((pcl::rad2deg(f1)+180) > 150)
//							bf1 = 1;
//						else
//							bf1 = 0;
//
//						if(pcl::rad2deg(SafeAcos(f2)) > 90)
//							bf2 = 1;
//						else
//							bf2 = 0;
//
//						if(pcl::rad2deg(SafeAcos(f3)) > 60)
//							bf3 = 1;
//						else
//							bf3 = 0;
//					}
//					//push features for each comparison
//					features.push_back(bf1); features.push_back(bf2); features.push_back(bf3);
//					if(rgb==true)
//					{
//						features.push_back(r); features.push_back(g); features.push_back(b);
//					}
//				}
//				BOP3_features2.push_back(features);
//			}
//
//
//		}
//		else
//			std::cout<<"NaN keypoint!"<<std::endl;
//	}
//
//
//	return 1;
//}
//
//int computeSingleBOP3(bool rgb)
//{
//	picked_pt_BOP3_features.clear();
//	bool r,g,b;
//	pcl::PointXYZRGB searchPoint;
//	float R = 0.05;
//
//	std::vector<int> pointIdxNKNSearch(BOP_K);
//	std::vector<float> pointNKNSquaredDistance(BOP_K);
//
//	std::vector<int> pointIdxRadiusSearch;
//	std::vector<float> pointRadiusSquaredDistance;
//
//
//	if(rgb==true)
//		picked_pt_BOP3_features.reserve(BOP_K*6);
//	else
//		picked_pt_BOP3_features.reserve(BOP_K*3);
//	searchPoint.x = picked_pt.x;
//	searchPoint.y = picked_pt.y;
//	searchPoint.z = picked_pt.z;
//
//	//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
//	int nanpoints = 0;
//	int K = BOP_K + 1;
//	if ( tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//	{
//		for (size_t i = 1; i < pointIdxNKNSearch.size(); ++i){
//			if(rgb==true)
//			{
//				if(cloud->points[pointIdxNKNSearch[i]].r > picked_pt.r)
//					r=1;
//				else
//					r=0;
//				if(cloud->points[pointIdxNKNSearch[i]].g > picked_pt.g)
//					g=1;
//				else
//					g=0;
//				if(cloud->points[pointIdxNKNSearch[i]].b > picked_pt.b)
//					b=1;
//				else
//					b=0;
//			}
//			pcl::computePairFeatures(cloud->points[picked_pt_indice[0]].getVector4fMap(),cloud_normals->points[picked_pt_indice[0]].getNormalVector4fMap(),cloud->points[pointIdxNKNSearch[i] ].getVector4fMap(),cloud_normals->points[pointIdxNKNSearch[i] ].getNormalVector4fMap(),f1,f2,f3,f4);
//			//std::cout<<pcl::isFinite(cloud->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud_normals->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud->points[pointIdxNKNSearch[i]])<<" "<<pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]])<< std::endl;
//			//std::cout<<"Pair features: "<<f1<<" "<<f2<<" "<<f3<<" "<<f4<<std::endl;
//			//std::cout<<"Pair features: "<< pcl::rad2deg(f1)<<" "<<pcl::rad2deg(acos(f2))<<" "<<pcl::rad2deg(acos(f3))<<" "<<f4<<std::endl;
//
//			if(pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]]) == false)
//			{
//				bf1 = 0;
//				bf2 = 0;
//				bf3 = 0;
//				nanpoints++;
//			}
//			else{
//
//				//for viewing histogram of angles
//				double a1,a2,a3;
//				int ia1,ia2,ia3;
//				a1  = pcl::rad2deg(f1) + 180;
//				a2  = pcl::rad2deg(SafeAcos(f2));
//				a3  = pcl::rad2deg(SafeAcos(f3));
//				ia1 = int(floor((a1/359)*(anglehistsize-1)));
//				ia2 = int(floor((a2/359)*(anglehistsize-1)));
//				ia3 = int(floor((a3/359)*(anglehistsize-1)));
//				//std::cout<<"Angles: "<<f1<<" "<<f2<<" "<<f3<<" ; "<<a1<<" "<<a2<<" "<<a3<<" ; "<<ia1<<" "<<ia2<<" "<<ia3<<" "<<std::endl;
//				angle1hist_single[ia1] = angle1hist_single[ia1] + 1;
//				angle2hist_single[ia2] = angle2hist_single[ia2] + 1;
//				angle3hist_single[ia3] = angle3hist_single[ia3] + 1;
//				//end
//
//
//				if((pcl::rad2deg(f1)+180) > 150)
//					bf1 = 1;
//				else
//					bf1 = 0;
//
//				if(pcl::rad2deg(SafeAcos(f2)) > 90)
//					bf2 = 1;
//				else
//					bf2 = 0;
//
//				if(pcl::rad2deg(SafeAcos(f3)) > 60)
//					bf3 = 1;
//				else
//					bf3 = 0;
//			}
//			//push features for each comparison
//			picked_pt_BOP3_features.push_back(bf1); picked_pt_BOP3_features.push_back(bf2); picked_pt_BOP3_features.push_back(bf3);
//			if(rgb==true)
//			{
//				picked_pt_BOP3_features.push_back(r); picked_pt_BOP3_features.push_back(g); picked_pt_BOP3_features.push_back(b);
//			}
//		}
//	}
//
//
//	else
//		std::cout<<"NaN keypoint!"<<std::endl;
//
//
//	return 1;
//}
//
//int convertSingleBOP3toCV()
//{
//	int r = 1;
//	int c = picked_pt_BOP3_features.size();
//	c = c/8;
//	picked_pt_cvBOP_features = cv::Mat(r,c,CV_8U);
//
//
//		for(int i=0;i<c;i++){
//			uchar val = 0;
//			for(int k=0;k<8;k++){
//				uchar val2;
//				val2 =  picked_pt_BOP3_features[i*8+k];
//				val2 << 7-k;
//				val = val + val2;
//			}
//			picked_pt_cvBOP_features.at<uchar>(0,i) = val;
//		}
//
//
//
//	return 1;
//}
//
//int convertBOP3toCV()
//{
//	int r = BOP3_features.size();
//	int c = BOP3_features[0].size();
//	c = c/8;
//	cvBOP_features = cv::Mat(r,c,CV_8U);
//
//	for(int j=0;j<r;j++){
//		for(int i=0;i<c;i++){
//			uchar val = 0;
//			for(int k=0;k<8;k++){
//				uchar val2;
//				val2 =  BOP3_features[j][i*8+k];
//				val2 << 7-k;
//				val = val + val2;
//			}
//			cvBOP_features.at<uchar>(j,i) = val;
//		}
//	}
//
//	int r2 = BOP3_features2.size();
//	int c2 = BOP3_features2[0].size();
//	c2 = c2/8;
//	cvBOP_features2 = cv::Mat(r2,c2,CV_8U);
//
//	for(int j=0;j<r2;j++){
//		for(int i=0;i<c2;i++){
//			uchar val = 0;
//			for(int k=0;k<8;k++){
//				uchar val2;
//				val2 =  BOP3_features2[j][i*8+k];
//				val2 << 7-k;
//				val = val + val2;
//			}
//			cvBOP_features2.at<uchar>(j,i) = val;
//		}
//	}
//
//	return 1;
//}
//
//int convertBOP1toCV()
//{
//	int r = BOP1_features.size();
//	int c = BOP1_features[0].size();
//	c = c/8;
//	cvBOP1_features = cv::Mat(r,c,CV_8U);
//
//	for(int j=0;j<r;j++){
//		for(int i=0;i<c;i++){
//			uchar val = 0;
//			for(int k=0;k<8;k++){
//				uchar val2;
//				val2 =  BOP1_features[j][i*8+k];
//				val2 << 7-k;
//				val = val + val2;
//			}
//			cvBOP1_features.at<uchar>(j,i) = val;
//		}
//	}
//
//	int r2 = BOP1_features2.size();
//	int c2 = BOP1_features2[0].size();
//	c2 = c2/8;
//	cvBOP1_features2 = cv::Mat(r2,c2,CV_8U);
//
//	for(int j=0;j<r2;j++){
//		for(int i=0;i<c2;i++){
//			uchar val = 0;
//			for(int k=0;k<8;k++){
//				uchar val2;
//				val2 =  BOP1_features2[j][i*8+k];
//				val2 << 7-k;
//				val = val + val2;
//			}
//			cvBOP1_features2.at<uchar>(j,i) = val;
//		}
//	}
//
//	return 1;
//}
////int convertBOP3toCV()
////{
////	int r = BOP3_features.size();
////	int c = BOP3_features[0].size();
////	cvBOP_features = cv::Mat(r,c,CV_8U);
////
////	for(int j=0;j<r;j++){
////		for(int i=0;i<c;i++){
////			cvBOP_features.at<uchar>(j,i) = BOP3_features[j][i];
////		}
////	}
////
////	int r2 = BOP3_features2.size();
////	int c2 = BOP3_features2[0].size();
////	cvBOP_features2 = cv::Mat(r2,c2,CV_8U);
////
////	for(int j=0;j<r2;j++){
////		for(int i=0;i<c2;i++){
////			cvBOP_features2.at<uchar>(j,i) = BOP3_features2[j][i];
////		}
////	}
////
////	return 1;
////}
//
//
//int ComputeFSPFH(double R, int levels, int bins, bool rgb)
//{
//
//	//cloud 1
//	SPFH_features.clear();
//	int histsize = levels * bins * 3;
//	int featsize = levels*bins;
//	int levelsize = bins * 3;
//	int levelsearch[levels];
//	//std::cout<<"keypoint size: "<<keypoints->size()<<" "<<indices->indices.size()<<std::endl;
//	std::cout<<"keypoint size: "<<keypoints_rgb->size()<<std::endl;
//	for(int j=0;j<keypoints_rgb->size();j++)
//	{
//		int keypoint_indice;
//
//			keypoint_indice = indices->indices[j];
//
//		if(pcl::isFinite(cloud_normals->points[keypoint_indice]) == true && pcl::isFinite(cloud->points[keypoint_indice]) == true )
//		{
//			std::vector<float> spfh_features(histsize,0);
//			std::vector<int> pointIdxRadiusSearch;
//			std::vector<float> pointRadiusSquaredDistance;
//			float f1,f2,f3,f4;
//			float deg_f1,deg_f2,deg_f3;
//			int bin_f1,bin_f2,bin_f3;
//			int searchstart = 1;
//			for(int l=1;l<=levels;l++)
//			{
//				//std::vector<float> level_features(levelsize,0);
//				float level_features[levelsize];
//				for(int z=0;z<levelsize;z++)
//					level_features[z] = 0.0;
//				double r;
//				int searchsize;
//				unsigned int levelpaircount = 0;
//				r = ((l*1.0)/levels)*R;
//				searchsize = tree->radiusSearch (*cloud, keypoint_indice, r, pointIdxRadiusSearch, pointRadiusSquaredDistance,0);
//				//std::cout<<j<<"; in level "<<l<<" "<<searchsize<<std::endl;
//				levelsearch[l-1] = searchsize;
//				if(l!=1)
//					searchstart = levelsearch[l-2];
//
//				// Iterate over all the points in the neighborhood
//				for (int i_idx = searchstart; i_idx < searchsize; ++i_idx)
//				{
//
//						// If the 3D points are invalid, don't bother estimating, just continue
//						if (pcl::isFinite(cloud->points[pointIdxRadiusSearch[i_idx]])  && pcl::isFinite(cloud_normals->points[pointIdxRadiusSearch[i_idx]]))
//						{
//
//						levelpaircount++;
//						int p1, p2;
//						p1 = pointIdxRadiusSearch[i_idx];
//						p2 = keypoint_indice;
//						//std::cout<<j<<" "<<l<<" "<<searchsize<<std::endl;
//						//std::cout<<i_idx<<" "<<p1<<" "<<p2<<" "<<pointRadiusSquaredDistance[i_idx]<<std::endl;
//						//std::cout<<pcl::isFinite(cloud->points[p1])<<" "<<pcl::isFinite(cloud_normals->points[p1])<<" "<<pcl::isFinite(cloud->points[p2])<<" "<<pcl::isFinite(cloud_normals->points[p2])<< std::endl;
//						pcl::computePairFeatures(cloud->points[p2].getVector4fMap(),cloud_normals->points[p2].getNormalVector4fMap(),cloud->points[p1].getVector4fMap(),cloud_normals->points[p1].getNormalVector4fMap(),f1,f2,f3,f4);
//						deg_f1 = pcl::rad2deg(f1) + 180;
//						deg_f2 = pcl::rad2deg(SafeAcos(f2)) ;
//						deg_f3 = pcl::rad2deg(SafeAcos(f3));
//						bin_f1 = int(floor(deg_f1 / (360.0/bins)));
//						bin_f2 = int(floor(deg_f2 / (180.0/bins)));
//						bin_f3 = int(floor(deg_f3 / (180.0/bins)));
//						//std::cout<<f1<<" "<<f2<<" "<<f3<<" ; "<<deg_f1<<" "<<deg_f2<<" "<<deg_f3<<" ; "<<bin_f1<<" "<<bin_f2<<" "<<bin_f3<<std::endl;
//						//std::cout<<" in level "<<l<<std::endl;
//						//std::cout<<bin_f1<<" "<<1*bins + bin_f2<<" "<<2*bins + bin_f3<<std::endl;
//						//std::cout<<opfh_features[(levelsize*(l-1)) + bin_f1]<<" "<<opfh_features[(levelsize*(l-1)) + 1*bins + bin_f2]<<" "<<opfh_features[(levelsize*(l-1)) + 2*bins + bin_f3]<<std::endl;
//
//						level_features[bin_f1] = level_features[bin_f1] + 1.0;
//						level_features[1*bins + bin_f2] = level_features[1*bins + bin_f2] + 1.0;
//						level_features[2*bins + bin_f3] = level_features[2*bins + bin_f3] + 1.0;
//
//
//						}
//
//				}
//				//std::cout<<levelpaircount<<" pairs in level"<<std::endl;
//				if(levelpaircount!=0){
//					for(int n=0;n<levelsize;n++)
//					{
//						//std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
//						level_features[n] = level_features[n] / levelpaircount;
//						//std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
//						spfh_features[(l-1)*levelsize + n] = level_features[n];
//					}
//				}
//
//			}
//			SPFH_features.push_back(spfh_features);
//		}
//		else
//			std::cout<<"keypoint "<<j<<" ; "<<pcl::isFinite(cloud_normals->points[keypoint_indice]) << " "<<pcl::isFinite(cloud->points[keypoint_indice])  <<std::endl;
//
//	}
//
//	//cloud 2
//	SPFH_features2.clear();
//
//
//	int levelsearch2[levels];
//	//std::cout<<"keypoint size: "<<keypoints->size()<<" "<<indices->indices.size()<<std::endl;
//	std::cout<<"keypoint size: "<<keypoints2_rgb->size()<<std::endl;
//	for(int j=0;j<keypoints2_rgb->size();j++)
//	{
//		int keypoint2_indice;
//
//			keypoint2_indice = indices2->indices[j];
//
//		if(pcl::isFinite(cloud2_normals->points[keypoint2_indice]) == true && pcl::isFinite(cloud2->points[keypoint2_indice]) == true )
//		{
//			std::vector<float> spfh_features(histsize,0);
//			std::vector<int> pointIdxRadiusSearch;
//			std::vector<float> pointRadiusSquaredDistance;
//			float f1,f2,f3,f4;
//			float deg_f1,deg_f2,deg_f3;
//			int bin_f1,bin_f2,bin_f3;
//			int searchstart = 1;
//			for(int l=1;l<=levels;l++)
//			{
//				//std::vector<float> level_features(levelsize,0);
//				float level_features[levelsize];
//				for(int z=0;z<levelsize;z++)
//					level_features[z] = 0.0;
//				double r;
//				int searchsize;
//				unsigned int levelpaircount = 0;
//				r = ((l*1.0)/levels)*R;
//				searchsize = tree2->radiusSearch (*cloud2, keypoint2_indice, r, pointIdxRadiusSearch, pointRadiusSquaredDistance,0);
//				//std::cout<<j<<"; in level "<<l<<" "<<searchsize<<std::endl;
//				levelsearch2[l-1] = searchsize;
//				if(l!=1)
//					searchstart = levelsearch2[l-2];
//
//				// Iterate over all the points in the neighborhood
//				for (int i_idx = searchstart; i_idx < searchsize; ++i_idx)
//				{
//
//						// If the 3D points are invalid, don't bother estimating, just continue
//						if (pcl::isFinite(cloud2->points[pointIdxRadiusSearch[i_idx]])  && pcl::isFinite(cloud2_normals->points[pointIdxRadiusSearch[i_idx]]))
//						{
//
//						levelpaircount++;
//						int p1, p2;
//						p1 = pointIdxRadiusSearch[i_idx];
//						p2 = keypoint2_indice;
//						//std::cout<<j<<" "<<l<<" "<<searchsize<<std::endl;
//						//std::cout<<i_idx<<" "<<p1<<" "<<p2<<" "<<pointRadiusSquaredDistance[i_idx]<<std::endl;
//						//std::cout<<pcl::isFinite(cloud->points[p1])<<" "<<pcl::isFinite(cloud_normals->points[p1])<<" "<<pcl::isFinite(cloud->points[p2])<<" "<<pcl::isFinite(cloud_normals->points[p2])<< std::endl;
//						pcl::computePairFeatures(cloud2->points[p2].getVector4fMap(),cloud2_normals->points[p2].getNormalVector4fMap(),cloud2->points[p1].getVector4fMap(),cloud2_normals->points[p1].getNormalVector4fMap(),f1,f2,f3,f4);
//						deg_f1 = pcl::rad2deg(f1) + 180;
//						deg_f2 = pcl::rad2deg(SafeAcos(f2)) ;
//						deg_f3 = pcl::rad2deg(SafeAcos(f3));
//						bin_f1 = int(floor(deg_f1 / (360.0/bins)));
//						bin_f2 = int(floor(deg_f2 / (180.0/bins)));
//						bin_f3 = int(floor(deg_f3 / (180.0/bins)));
//						//std::cout<<f1<<" "<<f2<<" "<<f3<<" ; "<<deg_f1<<" "<<deg_f2<<" "<<deg_f3<<" ; "<<bin_f1<<" "<<bin_f2<<" "<<bin_f3<<std::endl;
//						//std::cout<<" in level "<<l<<std::endl;
//						//std::cout<<bin_f1<<" "<<1*bins + bin_f2<<" "<<2*bins + bin_f3<<std::endl;
//						//std::cout<<opfh_features[(levelsize*(l-1)) + bin_f1]<<" "<<opfh_features[(levelsize*(l-1)) + 1*bins + bin_f2]<<" "<<opfh_features[(levelsize*(l-1)) + 2*bins + bin_f3]<<std::endl;
//
//						level_features[bin_f1] = level_features[bin_f1] + 1.0;
//						level_features[1*bins + bin_f2] = level_features[1*bins + bin_f2] + 1.0;
//						level_features[2*bins + bin_f3] = level_features[2*bins + bin_f3] + 1.0;
//
//
//						}
//
//				}
//				//std::cout<<levelpaircount<<" pairs in level"<<std::endl;
//				if(levelpaircount!=0){
//					for(int n=0;n<levelsize;n++)
//					{
//						//std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
//						level_features[n] = level_features[n] / levelpaircount;
//						//std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
//						spfh_features[(l-1)*levelsize + n] = level_features[n];
//					}
//				}
//
//			}
//			SPFH_features2.push_back(spfh_features);
//		}
//		else
//			std::cout<<"keypoint "<<j<<" ; "<<pcl::isFinite(cloud2_normals->points[keypoint2_indice]) << " "<<pcl::isFinite(cloud2->points[keypoint2_indice])  <<std::endl;
//
//	}
//return 1;
//}
//
//
//int ConvertFSPFHtoCV()
//{
//	int r = SPFH_features.size();
//	int c = SPFH_features[0].size();
//	cv_SPFH_features = cv::Mat(r,c,CV_32F);
//	float val;
//	for(int j=0;j<r;j++){
//		for(int i=0;i<c;i++){
//				val =  SPFH_features[j][i];
//				cv_SPFH_features.at<float>(j,i) = val;
//		}
//	}
//
//	int r2 = SPFH_features2.size();
//	int c2 = SPFH_features2[0].size();
//	cv_SPFH_features2 = cv::Mat(r2,c2,CV_32F);
//	float val2;
//	for(int j=0;j<r2;j++){
//		for(int i=0;i<c2;i++){
//				val2 =  SPFH_features2[j][i];
//				cv_SPFH_features2.at<float>(j,i) = val2;
//		}
//	}
//
//	return 1;
//}
//
//
//int ComputeCSSPFH(double R, int levels, int bins, int rgb_type)
//{
//
//	//cloud 1
//	SPFH_features.clear();
//	int histsize = levels * bins * 3;
//	int featsize = levels*bins;
//	int levelsize = bins * 3;
//	int levelsearch[levels];
//
//	int levelsize_rgb = bins * 3;
//	int histsize_rgb = (levels * bins * 3);
//
//	int levelsize_total = levelsize + levelsize_rgb;
//	int histsize_total = histsize + histsize_rgb;
//
//	//std::cout<<"keypoint size: "<<keypoints->size()<<" "<<indices->indices.size()<<std::endl;
//	std::cout<<"keypoint size: "<<keypoints_rgb->size()<<std::endl;
//	for(int j=0;j<keypoints_rgb->size();j++)
//	{
//		int keypoint_indice;
//
//			keypoint_indice = indices->indices[j];
//
//		if(pcl::isFinite(cloud_normals->points[keypoint_indice]) == true && pcl::isFinite(cloud->points[keypoint_indice]) == true )
//		{
//			std::vector<float> spfh_features(histsize_total,0);
//			std::vector<int> pointIdxRadiusSearch;
//			std::vector<float> pointRadiusSquaredDistance;
//			float f1,f2,f3,f4;
//			float deg_f1,deg_f2,deg_f3;
//			int bin_f1,bin_f2,bin_f3;
//			int searchstart = 1;
//			for(int l=1;l<=levels;l++)
//			{
//				//std::vector<float> level_features(levelsize,0);
//				float level_features[levelsize];
//				float level_features_rgb[levelsize_rgb];
//				for(int z=0;z<levelsize;z++)
//					level_features[z] = 0.0;
//				for(int z=0;z<levelsize_rgb;z++)
//					level_features_rgb[z] = 0.0;
//				double r;
//				int searchsize;
//				unsigned int levelpaircount = 0;
//				r = ((l*1.0)/levels)*R;
//				searchsize = tree->radiusSearch (*cloud, keypoint_indice, r, pointIdxRadiusSearch, pointRadiusSquaredDistance,0);
//				//std::cout<<j<<"; in level "<<l<<" "<<searchsize<<std::endl;
//				levelsearch[l-1] = searchsize;
//				if(l!=1)
//					searchstart = levelsearch[l-2];
//
//				// Iterate over all the points in the neighborhood
//				for (int i_idx = searchstart; i_idx < searchsize; ++i_idx)
//				{
//
//						// If the 3D points are invalid, don't bother estimating, just continue
//						if (pcl::isFinite(cloud->points[pointIdxRadiusSearch[i_idx]])  && pcl::isFinite(cloud_normals->points[pointIdxRadiusSearch[i_idx]]))
//						{
//
//						levelpaircount++;
//						int p1, p2;
//						p1 = pointIdxRadiusSearch[i_idx];
//						p2 = keypoint_indice;
//						//std::cout<<j<<" "<<l<<" "<<searchsize<<std::endl;
//						//std::cout<<i_idx<<" "<<p1<<" "<<p2<<" "<<pointRadiusSquaredDistance[i_idx]<<std::endl;
//						//std::cout<<pcl::isFinite(cloud->points[p1])<<" "<<pcl::isFinite(cloud_normals->points[p1])<<" "<<pcl::isFinite(cloud->points[p2])<<" "<<pcl::isFinite(cloud_normals->points[p2])<< std::endl;
//						pcl::computePairFeatures(cloud->points[p2].getVector4fMap(),cloud_normals->points[p2].getNormalVector4fMap(),cloud->points[p1].getVector4fMap(),cloud_normals->points[p1].getNormalVector4fMap(),f1,f2,f3,f4);
//						deg_f1 = pcl::rad2deg(f1) + 180;
//						deg_f2 = pcl::rad2deg(SafeAcos(f2)) ;
//						deg_f3 = pcl::rad2deg(SafeAcos(f3));
//						bin_f1 = int(floor(deg_f1 / (360.0/bins)));
//						bin_f2 = int(floor(deg_f2 / (180.0/bins)));
//						bin_f3 = int(floor(deg_f3 / (180.0/bins)));
//						//std::cout<<f1<<" "<<f2<<" "<<f3<<" ; "<<deg_f1<<" "<<deg_f2<<" "<<deg_f3<<" ; "<<bin_f1<<" "<<bin_f2<<" "<<bin_f3<<std::endl;
//						//std::cout<<" in level "<<l<<std::endl;
//						//std::cout<<bin_f1<<" "<<1*bins + bin_f2<<" "<<2*bins + bin_f3<<std::endl;
//						//std::cout<<opfh_features[(levelsize*(l-1)) + bin_f1]<<" "<<opfh_features[(levelsize*(l-1)) + 1*bins + bin_f2]<<" "<<opfh_features[(levelsize*(l-1)) + 2*bins + bin_f3]<<std::endl;
//
//						level_features[bin_f1] = level_features[bin_f1] + 1.0;
//						level_features[1*bins + bin_f2] = level_features[1*bins + bin_f2] + 1.0;
//						level_features[2*bins + bin_f3] = level_features[2*bins + bin_f3] + 1.0;
//
//						if(rgb_type==5)
//								{
//									float l,a,b;
//									pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot;
//									shot.RGB2CIELAB(cloud->points[p1].r,cloud->points[p1].g,cloud->points[p1].b,l,a,b);
//									//std::cout<<int(cloud->points[p1].r)<<" "<<int(cloud->points[p1].g)<<" "<<int(cloud->points[p1].b)<<std::endl;
//									//std::cout<<l<<" "<<a<<" "<<b<<std::endl;
//								    l = 1.0 * l / 100;
//								    a = 1.0 * (a + 86.185) / 184.439;
//								    b = 1.0 * (b + 107.863) / 202.345;
//									int bin_r, bin_g, bin_b;
//									bin_r = int(floor(l / (1.0/bins)));
//									bin_g = int(floor(a / (1.0/bins)));
//									bin_b = int(floor(b / (1.0/bins)));
//									//std::cout<<bin_r<<" "<<bin_g<<" "<<bin_b<<std::endl;
//									level_features_rgb[bin_r] = level_features_rgb[bin_r] + 1.0;
//									level_features_rgb[1*bins + bin_g] = level_features_rgb[1*bins + bin_g] + 1.0;
//									level_features_rgb[2*bins + bin_b] = level_features_rgb[2*bins + bin_b] + 1.0;
//								}
//
//						}
//
//				}
//				//std::cout<<levelpaircount<<" pairs in level"<<std::endl;
//				if(levelpaircount!=0){
//					for(int n=0;n<levelsize;n++)
//					{
//						//std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
//						level_features[n] = level_features[n] / levelpaircount;
//						//std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
//						spfh_features[(l-1)*levelsize + n] = level_features[n];
//					}
//				}
//				if(levelpaircount!=0){
//					for(int n=0;n<levelsize_rgb;n++)
//					{
//						//std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
//						level_features_rgb[n] = level_features_rgb[n] / levelpaircount;
//						//std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
//						spfh_features[(l-1)*levelsize_total + levelsize + n] = level_features_rgb[n];
//					}
//				}
//
//			}
//			SPFH_features.push_back(spfh_features);
//		}
//		else
//			std::cout<<"keypoint "<<j<<" ; "<<pcl::isFinite(cloud_normals->points[keypoint_indice]) << " "<<pcl::isFinite(cloud->points[keypoint_indice])  <<std::endl;
//
//	}
//
//	//cloud 2
//	SPFH_features2.clear();
//
//
//	int levelsearch2[levels];
//	//std::cout<<"keypoint size: "<<keypoints->size()<<" "<<indices->indices.size()<<std::endl;
//	std::cout<<"keypoint size: "<<keypoints2_rgb->size()<<std::endl;
//	for(int j=0;j<keypoints2_rgb->size();j++)
//	{
//		int keypoint2_indice;
//
//			keypoint2_indice = indices2->indices[j];
//
//		if(pcl::isFinite(cloud2_normals->points[keypoint2_indice]) == true && pcl::isFinite(cloud2->points[keypoint2_indice]) == true )
//		{
//			std::vector<float> spfh_features(histsize_total,0);
//			std::vector<int> pointIdxRadiusSearch;
//			std::vector<float> pointRadiusSquaredDistance;
//			float f1,f2,f3,f4;
//			float deg_f1,deg_f2,deg_f3;
//			int bin_f1,bin_f2,bin_f3;
//			int searchstart = 1;
//			for(int l=1;l<=levels;l++)
//			{
//				//std::vector<float> level_features(levelsize,0);
//				float level_features[levelsize];
//				float level_features_rgb[levelsize_rgb];
//				for(int z=0;z<levelsize;z++)
//					level_features[z] = 0.0;
//				for(int z=0;z<levelsize_rgb;z++)
//					level_features_rgb[z] = 0.0;
//				double r;
//				int searchsize;
//				unsigned int levelpaircount = 0;
//				r = ((l*1.0)/levels)*R;
//				searchsize = tree2->radiusSearch (*cloud2, keypoint2_indice, r, pointIdxRadiusSearch, pointRadiusSquaredDistance,0);
//				//std::cout<<j<<"; in level "<<l<<" "<<searchsize<<std::endl;
//				levelsearch2[l-1] = searchsize;
//				if(l!=1)
//					searchstart = levelsearch2[l-2];
//
//				// Iterate over all the points in the neighborhood
//				for (int i_idx = searchstart; i_idx < searchsize; ++i_idx)
//				{
//
//						// If the 3D points are invalid, don't bother estimating, just continue
//						if (pcl::isFinite(cloud2->points[pointIdxRadiusSearch[i_idx]])  && pcl::isFinite(cloud2_normals->points[pointIdxRadiusSearch[i_idx]]))
//						{
//
//						levelpaircount++;
//						int p1, p2;
//						p1 = pointIdxRadiusSearch[i_idx];
//						p2 = keypoint2_indice;
//						//std::cout<<j<<" "<<l<<" "<<searchsize<<std::endl;
//						//std::cout<<i_idx<<" "<<p1<<" "<<p2<<" "<<pointRadiusSquaredDistance[i_idx]<<std::endl;
//						//std::cout<<pcl::isFinite(cloud->points[p1])<<" "<<pcl::isFinite(cloud_normals->points[p1])<<" "<<pcl::isFinite(cloud->points[p2])<<" "<<pcl::isFinite(cloud_normals->points[p2])<< std::endl;
//						pcl::computePairFeatures(cloud2->points[p2].getVector4fMap(),cloud2_normals->points[p2].getNormalVector4fMap(),cloud2->points[p1].getVector4fMap(),cloud2_normals->points[p1].getNormalVector4fMap(),f1,f2,f3,f4);
//						deg_f1 = pcl::rad2deg(f1) + 180;
//						deg_f2 = pcl::rad2deg(SafeAcos(f2)) ;
//						deg_f3 = pcl::rad2deg(SafeAcos(f3));
//						bin_f1 = int(floor(deg_f1 / (360.0/bins)));
//						bin_f2 = int(floor(deg_f2 / (180.0/bins)));
//						bin_f3 = int(floor(deg_f3 / (180.0/bins)));
//						//std::cout<<f1<<" "<<f2<<" "<<f3<<" ; "<<deg_f1<<" "<<deg_f2<<" "<<deg_f3<<" ; "<<bin_f1<<" "<<bin_f2<<" "<<bin_f3<<std::endl;
//						//std::cout<<" in level "<<l<<std::endl;
//						//std::cout<<bin_f1<<" "<<1*bins + bin_f2<<" "<<2*bins + bin_f3<<std::endl;
//						//std::cout<<opfh_features[(levelsize*(l-1)) + bin_f1]<<" "<<opfh_features[(levelsize*(l-1)) + 1*bins + bin_f2]<<" "<<opfh_features[(levelsize*(l-1)) + 2*bins + bin_f3]<<std::endl;
//
//						level_features[bin_f1] = level_features[bin_f1] + 1.0;
//						level_features[1*bins + bin_f2] = level_features[1*bins + bin_f2] + 1.0;
//						level_features[2*bins + bin_f3] = level_features[2*bins + bin_f3] + 1.0;
//
//						if(rgb_type==5)
//						{
//							float l,a,b;
//							pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot;
//							shot.RGB2CIELAB(cloud2->points[p1].r,cloud2->points[p1].g,cloud2->points[p1].b,l,a,b);
//							//std::cout<<int(cloud->points[p1].r)<<" "<<int(cloud->points[p1].g)<<" "<<int(cloud->points[p1].b)<<std::endl;
//							//std::cout<<l<<" "<<a<<" "<<b<<std::endl;
//							l = 1.0 * l / 100;
//							a = 1.0 * (a + 86.185) / 184.439;
//							b = 1.0 * (b + 107.863) / 202.345;
//							int bin_r, bin_g, bin_b;
//							bin_r = int(floor(l / (1.0/bins)));
//							bin_g = int(floor(a / (1.0/bins)));
//							bin_b = int(floor(b / (1.0/bins)));
//							//std::cout<<bin_r<<" "<<bin_g<<" "<<bin_b<<std::endl;
//							level_features_rgb[bin_r] = level_features_rgb[bin_r] + 1.0;
//							level_features_rgb[1*bins + bin_g] = level_features_rgb[1*bins + bin_g] + 1.0;
//							level_features_rgb[2*bins + bin_b] = level_features_rgb[2*bins + bin_b] + 1.0;
//						}
//						}
//
//				}
//				//std::cout<<levelpaircount<<" pairs in level"<<std::endl;
//				if(levelpaircount!=0){
//					for(int n=0;n<levelsize;n++)
//					{
//						//std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
//						level_features[n] = level_features[n] / levelpaircount;
//						//std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
//						spfh_features[(l-1)*levelsize + n] = level_features[n];
//					}
//				}
//				if(levelpaircount!=0){
//					for(int n=0;n<levelsize_rgb;n++)
//					{
//						//std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
//						level_features_rgb[n] = level_features_rgb[n] / levelpaircount;
//						//std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
//						spfh_features[(l-1)*levelsize_total + levelsize + n] = level_features_rgb[n];
//					}
//				}
//
//			}
//			SPFH_features2.push_back(spfh_features);
//		}
//		else
//			std::cout<<"keypoint "<<j<<" ; "<<pcl::isFinite(cloud2_normals->points[keypoint2_indice]) << " "<<pcl::isFinite(cloud2->points[keypoint2_indice])  <<std::endl;
//
//	}
//return 1;
//}
//
//std::vector<cv::DMatch> matchBF_Hamming(cv::Mat &descriptor1, cv::Mat &descriptor2)
//{
//	cv::BFMatcher matcher(cv::NORM_HAMMING,true);
//	std::vector<cv::DMatch> matches;
//	matcher.match(descriptor1,descriptor2,matches);
//	for(int i=0;i<matches.size();i++)
//		std::cout<<"Match "<<i<<": "<<matches[i].queryIdx<<" "<<matches[i].trainIdx<<" "<<matches[i].imgIdx<<" "<<matches[i].distance<<std::endl;
//	return matches;
//}
//
//std::vector<std::vector<cv::DMatch> > matchBF_Hamming_KNN(cv::Mat &queryDescriptors, cv::Mat& trainDescriptors, int k)
//{
//	cv::BFMatcher matcher(cv::NORM_HAMMING,false);
//	std::vector<std::vector<cv::DMatch> > allmatches;
//	matcher.knnMatch(queryDescriptors,trainDescriptors, allmatches,k);
////	for(int i=0;i<allmatches.size();i++)
////		std::cout<<"KNN Match "<<i<<": "<<allmatches[0][i].queryIdx<<" "<<allmatches[0][i].trainIdx<<" "<<allmatches[0][i].imgIdx<<" "<<allmatches[0][i].distance<<std::endl;
//	return allmatches;
//}
//
//std::vector<cv::DMatch> matchBF_Hamming2(cv::Mat &descriptor1, cv::Mat &descriptor2)
//{
//	cv::BFMatcher matcher(cv::NORM_HAMMING);
//	std::vector<std::vector<cv::DMatch> > matches;
//	matcher.knnMatch(descriptor1, descriptor2, matches, 2);  // Find two nearest matches
//	std::vector<cv::DMatch> good_matches;
//	for (int i = 0; i < matches.size(); i++)
//	{
//	    const float ratio = 0.8; // As in Lowe's paper; can be tuned
//	    if (matches[i][0].distance < ratio * matches[i][1].distance)
//	    {
//	        good_matches.push_back(matches[i][0]);
//	    }
//	}
//	for(int i=0;i<good_matches.size();i++)
//		std::cout<<"Match "<<i<<": "<<good_matches[i].queryIdx<<" "<<good_matches[i].trainIdx<<" "<<good_matches[i].imgIdx<<" "<<good_matches[i].distance<<std::endl;
//	return good_matches;
//}
//
//std::vector<cv::DMatch> matchBF_L2(cv::Mat &descriptor1, cv::Mat &descriptor2)
//{
//	cv::BFMatcher matcher(cv::NORM_L2,true);
//	//cv::BFMatcher matcher(cv::NORM_L2);
//	std::vector<cv::DMatch> matches;
//	matcher.match(descriptor1,descriptor2,matches);
//	for(int i=0;i<matches.size();i++)
//		std::cout<<"Match "<<i<<": "<<matches[i].queryIdx<<" "<<matches[i].trainIdx<<" "<<matches[i].imgIdx<<" "<<matches[i].distance<<std::endl;
//	return matches;
//}
//
//
//
//std::vector<std::vector<cv::DMatch> > matchBF_L2_KNN(cv::Mat &queryDescriptors, cv::Mat& trainDescriptors, int k)
//{
//	//std::cout<<"k: "<<k<<std::endl;
//	cv::BFMatcher matcher(cv::NORM_L2,false);
//	std::vector<std::vector<cv::DMatch> > allmatches;
//	matcher.knnMatch(queryDescriptors,trainDescriptors, allmatches,k);
////	for(int i=0;i<allmatches.size();i++)
////		std::cout<<"KNN Match "<<i<<": "<<allmatches[0][i].queryIdx<<" "<<allmatches[0][i].trainIdx<<" "<<allmatches[0][i].imgIdx<<" "<<allmatches[0][i].distance<<std::endl;
//	return allmatches;
//}
//
//std::vector<std::vector<cv::DMatch> > matchFLANN_L2(cv::Mat &queryDescriptors, cv::Mat& trainDescriptors, int k)
//{
//	cv::FlannBasedMatcher matcher;
//	std::vector<std::vector<cv::DMatch> > allmatches;
//	matcher.knnMatch(queryDescriptors,trainDescriptors, allmatches,k);
////	for(int i=0;i<allmatches.size();i++)
////		std::cout<<"KNN Match "<<i<<": "<<allmatches[0][i].queryIdx<<" "<<allmatches[0][i].trainIdx<<" "<<allmatches[0][i].imgIdx<<" "<<allmatches[0][i].distance<<std::endl;
//	return allmatches;
//}
//
//int computeFPFH()
//{
//	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
//	fpfh.setSearchMethod(tree);;
//	fpfh.setInputNormals(cloud_normals);
//	fpfh.setRadiusSearch(support_radius);
//	fpfh.setSearchSurface(cloud);
//	fpfh.setInputCloud(keypoints_rgb);
//	fpfh.compute(*fpfh_signature);
//
//	//CLoud 2
//	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh2;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::copyPointCloud(*keypoints2, *keypoints2_xyzrgb);
//	fpfh2.setSearchMethod(tree2);;
//	fpfh2.setInputNormals(cloud2_normals);
//	fpfh2.setRadiusSearch(support_radius);
//	fpfh2.setSearchSurface(cloud2);
//	fpfh2.setInputCloud(keypoints2_rgb);
//	fpfh2.compute(*fpfh_signature2);
//
//
//	return 1;
//}
//
//int computeSHOT()
//{
//	pcl::SHOTEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
//	shot.setSearchMethod(tree);;
//	shot.setInputNormals(cloud_normals);
//	shot.setRadiusSearch(support_radius);
//	shot.setSearchSurface(cloud);
//	shot.setInputCloud(keypoints_rgb);
//	shot.compute(*shot_signature);
//
//	//CLoud 2
//	pcl::SHOTEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot2;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::copyPointCloud(*keypoints2, *keypoints2_xyzrgb);
//	shot2.setSearchMethod(tree2);;
//	shot2.setInputNormals(cloud2_normals);
//	shot2.setRadiusSearch(support_radius);
//	shot2.setSearchSurface(cloud2);
//	shot2.setInputCloud(keypoints2_rgb);
//	shot2.compute(*shot_signature2);
//
//	return 1;
//}
//
//int computeSHOTcolor()
//{
//	pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
//	shot.setSearchMethod(tree);;
//	shot.setInputNormals(cloud_normals);
//	shot.setRadiusSearch(support_radius);
//	shot.setSearchSurface(cloud);
//	shot.setInputCloud(keypoints_rgb);
//	shot.compute(*shotcolor_signature);
//
//	//CLoud 2
//	pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot2;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::copyPointCloud(*keypoints2, *keypoints2_xyzrgb);
//	shot2.setSearchMethod(tree2);;
//	shot2.setInputNormals(cloud2_normals);
//	shot2.setRadiusSearch(support_radius);
//	shot2.setSearchSurface(cloud2);
//	shot2.setInputCloud(keypoints2_rgb);
//	shot2.compute(*shotcolor_signature2);
//
//	return 1;
//}
//
//int convertFPFHtoCV()
//{
//	int num_points = fpfh_signature->width;
//	if(num_points > 0){
//		cv_fpfh_signature = cv::Mat(num_points,33,CV_32F);
//		for(int j=0;j<num_points;j++){
//			for(int i=0;i<33;i++){
//				cv_fpfh_signature.at<float>(j,i) = fpfh_signature->points[j].histogram[i];
//				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
//			}
//			//std::cout<<std::endl;
//
//		}
//	}
//
//	int num_points2 = fpfh_signature2->width;
//	if(num_points2 > 0){
//		cv_fpfh_signature2 = cv::Mat(num_points2,33,CV_32F);
//		for(int j=0;j<num_points2;j++){
//			for(int i=0;i<33;i++){
//				cv_fpfh_signature2.at<float>(j,i) = fpfh_signature2->points[j].histogram[i];
//				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
//			}
//			//std::cout<<std::endl;
//
//		}
//	}
//
//	return 1;
//}
//
//
//int convertSHOTtoCV()
//{
//	int num_points = shot_signature->width;
//	if(num_points > 0){
//		cv_shot_signature = cv::Mat(num_points,352,CV_32F);
//		for(int j=0;j<num_points;j++){
//			for(int i=0;i<352;i++){
//				cv_shot_signature.at<float>(j,i) = shot_signature->points[j].descriptor[i];
//				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
//			}
//			//std::cout<<std::endl;
//
//		}
//	}
//
//	int num_points2 = shot_signature2->width;
//	if(num_points2 > 0){
//		cv_shot_signature2 = cv::Mat(num_points2,352,CV_32F);
//		for(int j=0;j<num_points2;j++){
//			for(int i=0;i<352;i++){
//				cv_shot_signature2.at<float>(j,i) = shot_signature2->points[j].descriptor[i];
//				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
//			}
//			//std::cout<<std::endl;
//
//		}
//	}
//
//	return 1;
//}
//
//int convertSHOTcolortoCV()
//{
//	int num_points = shotcolor_signature->width;
//	if(num_points > 0){
//		cv_shotcolor_signature = cv::Mat(num_points,1344,CV_32F);
//		for(int j=0;j<num_points;j++){
//			for(int i=0;i<1344;i++){
//				cv_shotcolor_signature.at<float>(j,i) = shotcolor_signature->points[j].descriptor[i];
//				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
//			}
//			//std::cout<<std::endl;
//
//		}
//	}
//
//	int num_points2 = shotcolor_signature2->width;
//	if(num_points2 > 0){
//		cv_shotcolor_signature2 = cv::Mat(num_points2,1344,CV_32F);
//		for(int j=0;j<num_points2;j++){
//			for(int i=0;i<1344;i++){
//				cv_shotcolor_signature2.at<float>(j,i) = shotcolor_signature2->points[j].descriptor[i];
//				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
//			}
//			//std::cout<<std::endl;
//
//		}
//	}
//
//	return 1;
//}
//
//void writeBOP(std::string filename, pcl::PointCloud<BOP3hist> hist)
//{
//	std::ofstream out;
//	out.open(filename.c_str());
//	int histsize = sizeof(hist[0].histogram) / sizeof(hist[0].histogram[0]) ;
//	//std::cout<<"array size "<<histsize<<std::endl;
//	for(int i=0;i<histsize;i++)
//	{
//		if(i!=histsize)
//			out<<hist[0].histogram[i]<<" ";
//		else
//			out<<hist[0].histogram[i];
//	}
//}
//
//void writeSelectedFPFH()
//{
//	std::ofstream out;
//	out.open("selected_fpfh.txt");
//
//	for(int i=0;i<33;i++)
//	{
//		if(i!=33)
//			out<<fpfh_signature->points[picked_keypt_ind].histogram[i]<<" ";
//		else
//			out<<fpfh_signature->points[picked_keypt_ind].histogram[i];
//	}
//}
//
//void writeSelectedSPFH()
//{
//	std::ofstream out;
//	out.open("selected_spfh.txt");
//	out<<spfh_levels<<std::endl;
//	out<<spfh_bins<<std::endl;
//	int size = SPFH_features[picked_keypt_ind].size();
//	for(int i=0;i<size;i++)
//	{
//		if(i!=size)
//			out<<SPFH_features[picked_keypt_ind][i]<<" ";
//		else
//			out<<SPFH_features[picked_keypt_ind][i];
//	}
//}
//
//void writeSelectedBOP3()
//{
//	std::ofstream out;
//	out.open("selected_bop3.txt");
//	int size = BOP3_features[picked_keypt_ind].size();
//	for(int i=0;i<size;i++)
//	{
//		if(i!=size)
//			out<<BOP3_features[picked_keypt_ind][i]<<" ";
//		else
//			out<<BOP3_features[picked_keypt_ind][i];
//	}
//}
//
//void writeHistograms()
//{
//	std::ofstream out;
//	out.open("anglehists.txt");
//
//	for(int i=0;i<anglehistsize;i++)
//	{
//		if(i!=anglehistsize)
//			out<<angle1hist[i]<<" ";
//		else
//			out<<angle1hist[i];
//	}
//	out<<std::endl;
//	for(int i=0;i<anglehistsize;i++)
//	{
//		if(i!=anglehistsize)
//			out<<angle2hist[i]<<" ";
//		else
//			out<<angle2hist[i];
//	}
//	out<<std::endl;
//	for(int i=0;i<anglehistsize;i++)
//	{
//		if(i!=anglehistsize)
//			out<<angle3hist[i]<<" ";
//		else
//			out<<angle3hist[i];
//	}
//}
//
//void writeSingleHistograms()
//{
//	std::ofstream out;
//	out.open("anglehists_single.txt");
//
//	for(int i=0;i<anglehistsize;i++)
//	{
//		if(i!=anglehistsize)
//			out<<angle1hist_single[i]<<" ";
//		else
//			out<<angle1hist_single[i];
//	}
//	out<<std::endl;
//	for(int i=0;i<anglehistsize;i++)
//	{
//		if(i!=anglehistsize)
//			out<<angle2hist_single[i]<<" ";
//		else
//			out<<angle2hist_single[i];
//	}
//	out<<std::endl;
//	for(int i=0;i<anglehistsize;i++)
//	{
//		if(i!=anglehistsize)
//			out<<angle3hist_single[i]<<" ";
//		else
//			out<<angle3hist_single[i];
//	}
//}
//
//rgb convert_to_heat_map(float minimum, float maximum, float value)
//{
//	rgb rgbval;
//
//    float halfmax = (minimum + maximum) / 2;
//    rgbval.b = int(std::max(float(0), 255*(1 - value/halfmax)));
//    rgbval.r = int(std::max(float(0), 255*(value/halfmax - 1)));
//    rgbval.g = 255 - rgbval.b - rgbval.r;
//
//	return rgbval;
//}
//void pp_callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
//{
//	if (event.getPointIndex () == -1)
//		return;
//
//	std::cout <<"Picked point index: "<< event.getPointIndex() << std::endl;
//	event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
//	std::cout <<"x: "<< picked_pt.x<<"; y: "<< picked_pt.y<<"; z: "<< picked_pt.z<< std::endl;
//	std::vector<float> distances (1);
//	tree->nearestKSearch (picked_pt, 1, picked_pt_indice, distances);
//	std::cout<<picked_pt_indice[0]<<" "<<cloud->points[picked_pt_indice[0]].x<<" "<<cloud->points[picked_pt_indice[0]].y<<" "<<cloud->points[picked_pt_indice[0]].z<<std::endl;
//	picked_pt = cloud->points[picked_pt_indice[0]];
//	std::cout <<"x: "<< picked_pt.x<<"; y: "<< picked_pt.y<<"; z: "<< picked_pt.z<< std::endl;
//
//
//	//-------------------//
//	std::vector<int> pointRadiusSearch;
//	std::vector<float> pointRadiusDistance;
//	double rad = 0.05;
//	tree->radiusSearch(picked_pt,rad,pointRadiusSearch,pointRadiusDistance,0);
//	std::cout<<pointRadiusDistance.size()<<" of points are in the radius of "<< rad<< std::endl;
//	//-------------------//
//
//
//	std::cout <<"Picked keypoint index: "<< event.getPointIndex() << std::endl;
//	event.getPoint (picked_keypt.x, picked_keypt.y, picked_keypt.z);
//	std::cout <<"x: "<< picked_keypt.x<<"; y: "<< picked_keypt.y<<"; z: "<< picked_keypt.z<< std::endl;
//	std::vector<float> distance_to_keypt(1);
//	keypoint_tree->nearestKSearch (picked_keypt, 1, picked_keypt_indice, distance_to_keypt);
//	std::cout<<picked_keypt_indice[0]<<" "<<keypoints_rgb->points[picked_keypt_indice[0]].x<<" "<<keypoints_rgb->points[picked_keypt_indice[0]].y<<" "<<keypoints_rgb->points[picked_keypt_indice[0]].z<<std::endl;
//	picked_keypt = keypoints_rgb->points[picked_keypt_indice[0]];
//	std::cout <<"x: "<< picked_keypt.x<<"; y: "<< picked_keypt.y<<"; z: "<< picked_keypt.z<< std::endl;
//	picked_keypt_ind = picked_keypt_indice[0];
//
//	computeSingleBOP3(false);
//	if(feature_type==1){
//		writeSingleHistograms();
//		std::system("python bop_single_hist_plotter.py &");
//
//		//writeBOP("picked_pt",bop3hist);
//		//std::system("python bop_plotter.py &");
//
//		writeSelectedBOP3();
//		std::system("python selected_bop3_plotter.py &");
//	}
//	pcl::PointCloud<BOP3hist> bop3hist;
//	bop3hist.resize(1);
//	std::cout<<"picked_pt size: "<<picked_pt_BOP3_features.size()<<std::endl;
//	for(int i=0;i<picked_pt_BOP3_features.size();i++)
//	{
//		bop3hist.points[0].histogram[i] = picked_pt_BOP3_features[i];
//		//std::cout<<"hist : "<<bop3hist.points[0].histogram[i]<<std::endl;
//	}
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::copyPointCloud(*keypoints2_rgb_shifted,*color_keypoints);
//
//
//
//
//	//----------------------------------------------------------------------------------------
//	/*
//	std::vector<std::vector<cv::DMatch> > picked_pt_matches ;
//	convertSingleBOP3toCV();
//	int knn = BOP3_features2.size();
//	picked_pt_matches = matchBF_Hamming_KNN(picked_pt_cvBOP_features,cvBOP_features2, knn);
//	std::cout<<"picked pt matches size: "<<picked_pt_matches.size()<<" ; "<<picked_pt_matches[0].size()<<std::endl;
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::copyPointCloud(*keypoints2_shifted,*color_keypoints);
//
//	float maximum_dist, minimum_dist;
//	int picked_pt_ind = picked_pt_indice[0];
//	uint matchsize = picked_pt_matches[0].size();
//	minimum_dist = picked_pt_matches[0][0].distance;
//	maximum_dist = picked_pt_matches[0][matchsize-1].distance;
//
//	for(int j=0;j<matchsize;j++)
//	{
//		rgb rgb_vals;
//		rgb_vals = convert_to_heat_map(minimum_dist, maximum_dist, picked_pt_matches[0][j].distance);
//		int train_ind;
//		train_ind = picked_pt_matches[0][j].trainIdx;
//		color_keypoints->points[train_ind].r = rgb_vals.r;
//		color_keypoints->points[train_ind].g = rgb_vals.g;
//		color_keypoints->points[train_ind].b = rgb_vals.b;
//	}
//	int min_dist_train_id = picked_pt_matches[0][0].trainIdx;
//	viewer.updatePointCloud(color_keypoints,"keypoints2");
//	std::stringstream ss ("line");
//	ss << picked_pt_ind;
//	viewer.addLine(cloud_shifted->points[picked_pt_indice[0]], cloud2_shifted->points[indices2->indices[min_dist_train_id]],255,0,0,ss.str());
//	cout<<"Adding match line from: "<<picked_pt_ind<< " to "<<min_dist_train_id<<std::endl;
//	 */
//	// --------------------------------------------------------------------
//
//
//	float maximum_dist, minimum_dist;
//	picked_keypt_ind = picked_keypt_indice[0];
//	uint matchsize = vv_Matches[picked_keypt_ind].size();
//	minimum_dist = vv_Matches[picked_keypt_ind][0].distance;
//	maximum_dist = vv_Matches[picked_keypt_ind][matchsize-1].distance;
//
//	for(int j=0;j<matchsize;j++)
//	{
//		rgb rgb_vals;
//		rgb_vals = convert_to_heat_map(minimum_dist, maximum_dist, vv_Matches[picked_keypt_ind][j].distance);
//		int train_ind;
//		train_ind = vv_Matches[picked_keypt_ind][j].trainIdx;
//		color_keypoints->points[train_ind].r = rgb_vals.r;
//		color_keypoints->points[train_ind].g = rgb_vals.g;
//		color_keypoints->points[train_ind].b = rgb_vals.b;
//	}
//	int min_dist_train_id = vv_Matches[picked_keypt_ind][0].trainIdx;
//	viewer.updatePointCloud(color_keypoints,"keypoints2");
//    std::stringstream ss ("line");
//    ss << picked_keypt_ind;
//	viewer.addLine(cloud_shifted->points[indices->indices[picked_keypt_ind]], cloud2_shifted->points[indices2->indices[min_dist_train_id]],255,0,0,ss.str());
//	cout<<"Adding match line from: "<<picked_keypt_ind<< " to "<<min_dist_train_id<<std::endl;
//
//
//	if(feature_type==5)
//	{
//		writeSelectedFPFH();
//	}
//	if(feature_type==8)
//	{
//
//		writeSelectedSPFH();
//	}
//
//
//}
//
//int main (int argc, char* argv[])
//{
//
//	memset(angle1hist,0,sizeof (angle1hist));
//	memset(angle2hist,0,sizeof (angle2hist));
//	memset(angle3hist,0,sizeof (angle3hist));
//	memset(angle1hist_single,0,sizeof (angle1hist));
//	memset(angle2hist_single,0,sizeof (angle2hist));
//	memset(angle3hist_single,0,sizeof (angle3hist));
//
//	//read clouds
//	pcl::PCDReader reader;
//	testfile1 = argv[1];
//	if(testfile1 == "sphere"){
//		pcl::simulation::Sphere sphere (0.1);
//		sphere.generate (point_density);
//		sphere.getCloud (*cloud);
//		//my_shape_generator generator;
//		//generator.generate_sphere(cloud,0.05,100000);
//
//	}
//	else if(testfile1 == "cone"){
//		pcl::simulation::Cone cone (0.1,0.2);
//		cone.generate (point_density);
//		cone.getCloud (*cloud);
//
//	}
//	else if(testfile1 == "cylinder"){
//		pcl::simulation::Cylinder cylinder (0.1,0.2);
//		cylinder.generate (point_density);
//		cylinder.getCloud (*cloud);
//
//	}
//	else if(testfile1 == "torus"){
//		pcl::simulation::Torus torus (0.2, 0.1);
//		torus.generate (point_density);
//		torus.getCloud (*cloud);
//
//	}
//	else if(testfile1 == "wedge"){
//		pcl::simulation::Wedge wedge (0.2,0.2,0.1,0.1,0.1);
//		wedge.generate (point_density);
//		wedge.getCloud (*cloud);
//
//	}
//	else
//		reader.read<pcl::PointXYZRGB>(testfile1,*cloud);
//
//	testfile2 = argv[2];
//	if(testfile2 == "sphere"){
//		pcl::simulation::Sphere sphere (0.1);
//		sphere.generate (point_density);
//		sphere.getCloud (*cloud2);
//		//my_shape_generator generator;
//		//generator.generate_sphere(cloud,0.05,100000);
//
//	}
//	else if(testfile2 == "cone"){
//		pcl::simulation::Cone cone (0.1,0.2);
//		cone.generate (point_density);
//		cone.getCloud (*cloud2);
//
//	}
//	else if(testfile2 == "cylinder"){
//		pcl::simulation::Cylinder cylinder (0.1,0.3);
//		cylinder.generate (point_density);
//		cylinder.getCloud (*cloud2);
//
//	}
//	else if(testfile2 == "torus"){
//		pcl::simulation::Torus torus (0.2, 0.1);
//		torus.generate (point_density);
//		torus.getCloud (*cloud2);
//
//	}
//	else if(testfile2 == "wedge"){
//		pcl::simulation::Wedge wedge (0.3,0.2,0.2,0.2,0.2);
//		wedge.generate (point_density);
//		wedge.getCloud (*cloud2);
//
//	}
//	else
//		reader.read<pcl::PointXYZRGB>(testfile2,*cloud2);
//
//
//	bool ShowNormals;
//	bool ShowKeypoints;
//	bool ShowMatches;
//
//	ShowNormals = bool(atoi(argv[3]));
//	ShowKeypoints = bool(atoi(argv[4]));
//	ShowMatches = bool(atoi(argv[5]));
//
//
//	// 1=BOP3 ; 5=FPFH ; 6=SHOT:
//	feature_type = atoi(argv[6]);
//
//	//0-Uniform Sampling 1-Harris3D 2- ISS 3D
//	keypoint_type = atoi(argv[7]);
//
//	//Extract normals
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//	ne.setInputCloud (cloud);
//	ne.setSearchMethod (tree);
//	ne.setRadiusSearch (normal_radius);
//	ne.compute (*cloud_normals);
//
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne2;
//	ne2.setInputCloud (cloud2);
//	ne2.setSearchMethod (tree2);
//	ne2.setRadiusSearch (normal_radius);
//	ne2.compute (*cloud2_normals);
//
//	//Extract keypoints
//	if(keypoint_type == 1)
//	{
//		harris.setInputCloud(cloud);
//		harris.setNormals(cloud_normals);
//		harris.setNonMaxSupression(true);
//		harris.setRefine(false);
//		//harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal>::LOWE);
//		harris.setRadius(harris_radius);
//		harris.compute(*keypoints);
//		for(int j=0;j<keypoints->size();j++)
//		{
//			std::cout<<"Keypoints "<<j<<" x: "<<keypoints->points[j].x<<" y: "<<keypoints->points[j].y<<" z: "<<keypoints->points[j].z<<std::endl;
//		}
//		indices = harris.getKeypointsIndices();
//		pcl::copyPointCloud(*keypoints, *keypoints_rgb);
//
//		//Extract keypoints of cloud2
//		harris2.setInputCloud(cloud2);
//		harris2.setNormals(cloud2_normals);
//		harris2.setNonMaxSupression(true);
//		harris2.setRefine(false);
//		//harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal>::LOWE);
//		harris2.setRadius(harris_radius);
//		harris2.compute(*keypoints2);
//		for(int j=0;j<keypoints2->size();j++)
//		{
//			std::cout<<"Keypoints2 "<<j<<" x: "<<keypoints2->points[j].x<<" y: "<<keypoints2->points[j].y<<" z: "<<keypoints2->points[j].z<<std::endl;
//		}
//		indices2 = harris2.getKeypointsIndices();
//		pcl::copyPointCloud(*keypoints2, *keypoints2_rgb);
//
//
//		keypoint_tree->setInputCloud (keypoints_rgb);
//	}
//	else if(keypoint_type == 2)
//	{
//
//		//
//		//  ISS3D parameters
//		//
//		double iss_normal_radius_;
//		double iss_border_radius_;
//		double iss_salient_radius_;
//		double iss_non_max_radius_;
//		double iss_gamma_21_ (0.975);
//		double iss_gamma_32_ (0.975);
//		double iss_min_neighbors_ (5);
//		int iss_threads_ (4);
//		double model_resolution;
//		model_resolution = computeCloudResolution(cloud);
//		iss_salient_radius_ = 6 * model_resolution;
//		iss_non_max_radius_ = 4 * model_resolution;
//		//with be
//		iss_normal_radius_ = 4 * model_resolution;
//		iss_border_radius_ = 1 * model_resolution;
//		pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
//
//		iss_detector.setSearchMethod (tree);
//		iss_detector.setSalientRadius (iss_salient_radius_);
//		iss_detector.setNonMaxRadius (iss_non_max_radius_);
//		//with be
//		iss_detector.setNormalRadius (iss_normal_radius_);
//		iss_detector.setBorderRadius (iss_border_radius_);
//
//		iss_detector.setThreshold21 (iss_gamma_21_);
//		iss_detector.setThreshold32 (iss_gamma_32_);
//		iss_detector.setMinNeighbors (iss_min_neighbors_);
//		iss_detector.setNumberOfThreads (iss_threads_);
//		iss_detector.setInputCloud (cloud);
//		iss_detector.compute (*keypoints_rgb);
//		indices = iss_detector.getKeypointsIndices();
//		//keypointsize = keypoints_rgb->size();
//
//
//		double model_resolution2;
//		model_resolution2 = computeCloudResolution(cloud2);
//		iss_salient_radius_ = 6 * model_resolution2;
//		iss_non_max_radius_ = 4 * model_resolution2;
//		//with be
//		iss_normal_radius_ = 4 * model_resolution2;
//		iss_border_radius_ = 1 * model_resolution2;
//		pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector2;
//
//		iss_detector2.setSearchMethod (tree2);
//		iss_detector2.setSalientRadius (iss_salient_radius_);
//		iss_detector2.setNonMaxRadius (iss_non_max_radius_);
//		//with be
//		iss_detector2.setNormalRadius (iss_normal_radius_);
//		iss_detector2.setBorderRadius (iss_border_radius_);
//
//		iss_detector2.setThreshold21 (iss_gamma_21_);
//		iss_detector2.setThreshold32 (iss_gamma_32_);
//		iss_detector2.setMinNeighbors (iss_min_neighbors_);
//		iss_detector2.setNumberOfThreads (iss_threads_);
//		iss_detector2.setInputCloud (cloud2);
//		iss_detector2.compute (*keypoints2_rgb);
//		indices2 = iss_detector2.getKeypointsIndices();
//		//keypointsize = keypoints_rgb->size();
//
//
//		keypoint_tree->setInputCloud (keypoints_rgb);
//	}
//
//
//	if(ShowMatches == 1)
//	{
//		double time_A, time_B,time_C,time_D;
//		double calctime, matchtime;
//		double tickfreq= cv::getTickFrequency();
//		if(feature_type == 1)
//		{
//			time_A = cv::getTickCount();
//			computeBOP3(false);
//			time_B = cv::getTickCount();
//			convertBOP3toCV();
//			time_C = cv::getTickCount();
//			//Matches = matchBF_Hamming(cvBOP_features,cvBOP_features2);
//			int k = cvBOP_features2.rows;
//			vv_Matches = matchBF_L2_KNN(cvBOP_features,cvBOP_features2,k);
//			time_D = cv::getTickCount();
//			calctime = (time_B - time_A)/tickfreq;
//			matchtime = (time_D - time_C)/tickfreq;
//			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
//			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
//
//			writeHistograms();
//			std::system("python bop_hist_plotter.py &");
//
//		}
//		else if(feature_type == 2)
//		{
//			time_A = cv::getTickCount();
//			computeBOP3(true);
//			time_B = cv::getTickCount();
//			convertBOP3toCV();
//			time_C = cv::getTickCount();
//			Matches = matchBF_Hamming(cvBOP_features,cvBOP_features2);
//			//int k = cvBOP_features2.rows;
//			//vv_Matches = matchBF_Hamming_KNN(cvBOP_features,cvBOP_features2,k);
//			time_D = cv::getTickCount();
//			calctime = (time_B - time_A)/tickfreq;
//			matchtime = (time_D - time_C)/tickfreq;
//			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
//			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
//		}
//		else if(feature_type == 3)
//		{
//			time_A = cv::getTickCount();
//			computeBOP1(false);
//			time_B = cv::getTickCount();
//			convertBOP1toCV();
//			time_C = cv::getTickCount();
//			Matches = matchBF_Hamming(cvBOP1_features,cvBOP1_features2);
//			time_D = cv::getTickCount();
//			calctime = (time_B - time_A)/tickfreq;
//			matchtime = (time_D - time_C)/tickfreq;
//			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
//			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
//		}
//		else if(feature_type == 4)
//		{
//			time_A = cv::getTickCount();
//			computeBOP1(true);
//			time_B = cv::getTickCount();
//			convertBOP1toCV();
//			time_C = cv::getTickCount();
//			Matches = matchBF_Hamming(cvBOP1_features,cvBOP1_features2);
//			time_D = cv::getTickCount();
//			calctime = (time_B - time_A)/tickfreq;
//			matchtime = (time_D - time_C)/tickfreq;
//			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
//			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
//		}
//		else if(feature_type == 5)
//		{
//			time_A = cv::getTickCount();
//			computeFPFH();
//			time_B = cv::getTickCount();
//			convertFPFHtoCV();
//			time_C = cv::getTickCount();
//			Matches = matchBF_L2(cv_fpfh_signature,cv_fpfh_signature2);
//			int k = cv_fpfh_signature2.rows;
//			vv_Matches = matchBF_L2_KNN(cv_fpfh_signature,cv_fpfh_signature2,k);
//			time_D = cv::getTickCount();
//			calctime = (time_B - time_A)/tickfreq;
//			matchtime = (time_D - time_C)/tickfreq;
//			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
//			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
//		}
//		else if(feature_type == 6)
//		{
//			time_A = cv::getTickCount();
//			computeSHOT();
//			time_B = cv::getTickCount();
//			convertSHOTtoCV();
//			time_C = cv::getTickCount();
//			Matches = matchBF_L2(cv_shot_signature,cv_shot_signature2);
//			int k = cv_shot_signature2.rows;
//			vv_Matches = matchBF_L2_KNN(cv_shot_signature,cv_shot_signature2,k);
//			time_D = cv::getTickCount();
//			calctime = (time_B - time_A)/tickfreq;
//			matchtime = (time_D - time_C)/tickfreq;
//			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
//			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
//		}
//		else if(feature_type == 7)
//		{
//			time_A = cv::getTickCount();
//			computeSHOTcolor();
//			time_B = cv::getTickCount();
//			convertSHOTcolortoCV();
//			time_C = cv::getTickCount();
//			Matches = matchBF_L2(cv_shotcolor_signature,cv_shotcolor_signature2);
//			int k = cv_shotcolor_signature2.rows;
//			vv_Matches = matchBF_L2_KNN(cv_shotcolor_signature,cv_shotcolor_signature2,k);
//			time_D = cv::getTickCount();
//			calctime = (time_B - time_A)/tickfreq;
//			matchtime = (time_D - time_C)/tickfreq;
//			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
//			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
//		}
//		else if(feature_type == 8)
//		{
//			time_A = cv::getTickCount();
//			ComputeFSPFH(spfh_radius,spfh_levels,spfh_bins,0);
//			time_B = cv::getTickCount();
//			ConvertFSPFHtoCV();
//			time_C = cv::getTickCount();
//			Matches = matchBF_L2(cv_SPFH_features,cv_SPFH_features2);
//			int k = cv_SPFH_features2.rows;
//			vv_Matches = matchBF_L2_KNN(cv_SPFH_features,cv_SPFH_features2,k);
//			time_D = cv::getTickCount();
//			calctime = (time_B - time_A)/tickfreq;
//			matchtime = (time_D - time_C)/tickfreq;
//			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
//			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
//		}
//		else if(feature_type == 9)
//		{
//			time_A = cv::getTickCount();
//			ComputeCSSPFH(spfh_radius,spfh_levels,spfh_bins,5);
//			time_B = cv::getTickCount();
//			ConvertFSPFHtoCV();
//			time_C = cv::getTickCount();
//			Matches = matchBF_L2(cv_SPFH_features,cv_SPFH_features2);
//			int k = cv_SPFH_features2.rows;
//			vv_Matches = matchBF_L2_KNN(cv_SPFH_features,cv_SPFH_features2,k);
//			time_D = cv::getTickCount();
//			calctime = (time_B - time_A)/tickfreq;
//			matchtime = (time_D - time_C)/tickfreq;
//			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
//			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
//		}
//
//
//	}
//
//	// VIEWER!
//
//	// Shift the first clouds' points to the left
//	const Eigen::Vector3f translate (0.15, 0.0, 0.0);
//	const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
//	pcl::transformPointCloud (*cloud, *cloud_shifted, -translate, no_rotation);
//	pcl::transformPointCloud (*keypoints_rgb, *keypoints_rgb_shifted, -translate, no_rotation);
//
//	const Eigen::Vector3f translate2 (0.15, 0.0, 0.0);
//	// Shift the second clouds' points to the right
//	pcl::transformPointCloud (*cloud2, *cloud2_shifted, translate2, no_rotation);
//	pcl::transformPointCloud (*keypoints2_rgb, *keypoints2_rgb_shifted, translate2, no_rotation);
//
//
//
//
//	//	viewer.setBackgroundColor (255, 255, 255);
//
//
//
//	pcl::visualization::PointCloudColorHandlerRGBField< pcl:: PointXYZRGB > color (cloud);
//	pcl::visualization::PointCloudColorHandlerRGBField< pcl:: PointXYZRGB > color2 (cloud2);
//
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorhandler (cloud,255,255,0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorhandler2 (cloud2,255,255,0);
//
//
//
//	viewer.addPointCloud<pcl::PointXYZRGB> (cloud_shifted,color, "cloud");
//	viewer.addPointCloud<pcl::PointXYZRGB> (cloud2_shifted,color2, "cloud2");
//
//
//
//	if(ShowKeypoints == 1){
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>  keypointscolorHandler (keypoints_rgb, 0,255,0);
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>  keypointscolorHandler2 (keypoints2_rgb, 0,255,0);
//
//		viewer.addPointCloud<pcl::PointXYZRGB> (keypoints_rgb_shifted,keypointscolorHandler, "keypoints");
//		viewer.setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");
//
//		viewer.addPointCloud<pcl::PointXYZRGB> (keypoints2_rgb_shifted,keypointscolorHandler2, "keypoints2");
//		viewer.setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints2");
//	}
//
//
//	if(ShowNormals == 1){
//		//viewer.setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_COLOR,0,0,0,"normals");
//		viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_shifted, cloud_normals, 100, 0.01, "normals");
//		viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud2_shifted, cloud2_normals, 100, 0.01, "normals2");
//	}
//
//	if(ShowMatches == 1)
//	{
//	int id1, id2;
////		for(int i=0;i<vv_Matches.size();i++)
////		{
////			if(!vv_Matches[i].empty())
////			{
////				id1 = vv_Matches[i][0].queryIdx;
////				id2 = vv_Matches[i][0].trainIdx;
////				std::stringstream ss ("line");
////				ss << i;
////				viewer.addLine(cloud_shifted->points[indices->indices[id1]], cloud2_shifted->points[indices2->indices[id2]],255,0,0,ss.str());
////			}
////		}
//	for(int i=0;i<Matches.size();i++)
//	{
//
//			id1 = Matches[i].queryIdx;
//			id2 = Matches[i].trainIdx;
//			std::stringstream ss ("line");
//			ss << i;
//			viewer.addLine(cloud_shifted->points[indices->indices[id1]], cloud2_shifted->points[indices2->indices[id2]],0,0,255,ss.str());
//
//	}
//
//	}
//
//	viewer.setBackgroundColor (255, 255, 255); //SET FOR WHITE BACKGROUND
//	viewer.initCameraParameters ();
//
//	//-----------
//	viewer.setCameraClipDistances(0.0394826,0.549811);
//	viewer.setCameraPosition(0.00735367,-0.178877,0.684724, 0.00364672,0.0168323,0.816468, 0.0373911,-0.557546,0.829303);
//	viewer.setCameraFieldOfView(0.8575);
//	viewer.setSize(960,540);
//	viewer.setPosition(967,544);
//	//-----------
//
//
//
//	viewer.registerPointPickingCallback (pp_callback, (void*)&viewer);
//
//	while (!viewer.wasStopped ())
//	{
//		viewer.spinOnce ();
//
//	}
//
//
//	return 1;
//}
//
//
//
//
//
