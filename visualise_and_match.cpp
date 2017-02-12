/*
 * visualise_and_extract_on_click.cpp
 *
 *  Created on: Jun 1, 2014
 *      Author: berker
 */

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <ctime>
#include <cmath>

#include "FeatureExtractor.h"
#include "Reader.h"
#include "Matcher.h"
#include "Evaluater.h"


//PCL
#include <boost/thread/thread.hpp>
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/features/pfh.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/PointIndices.h>
#include <pcl/common/angles.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_plotter.h>


//OPENCV
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/ml/ml.hpp>



int BOP_K = 128;
struct BOP3hist
{
  float histogram[384];
};


struct BOP1hist
{
  float histogram[128];
};


//for keypoint visualization!!
pcl::visualization::PCLVisualizer viewer;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shifted (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_shifted (new pcl::PointCloud<pcl::PointXYZI>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr cloud2_normals (new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_shifted (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints2_shifted (new pcl::PointCloud<pcl::PointXYZI>);

pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal> harris;
pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal> harris2;

pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_signature (new pcl::PointCloud<pcl::FPFHSignature33> ());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_signature2 (new pcl::PointCloud<pcl::FPFHSignature33> ());
pcl::PointCloud<pcl::SHOT352>::Ptr shot_signature (new pcl::PointCloud<pcl::SHOT352> ());
pcl::PointCloud<pcl::SHOT352>::Ptr shot_signature2 (new pcl::PointCloud<pcl::SHOT352> ());
pcl::PointCloud<pcl::SHOT1344>::Ptr shotcolor_signature (new pcl::PointCloud<pcl::SHOT1344> ());
pcl::PointCloud<pcl::SHOT1344>::Ptr shotcolor_signature2 (new pcl::PointCloud<pcl::SHOT1344> ());

std::string testfile1, testfile2;

pcl::PointIndicesConstPtr indices ( new pcl::PointIndices);
pcl::PointIndicesConstPtr indices2 ( new pcl::PointIndices);
std::vector<std::vector<bool> >BOP3_features;
std::vector<std::vector<bool> >BOP3_features2;
std::vector<std::vector<bool> >BOP1_features;
std::vector<std::vector<bool> >BOP1_features2;
cv::Mat cvBOP_features;
cv::Mat cvBOP_features2;
cv::Mat cvBOP1_features;
cv::Mat cvBOP1_features2;
cv::Mat cv_fpfh_signature;
cv::Mat cv_fpfh_signature2;
cv::Mat cv_shot_signature;
cv::Mat cv_shot_signature2;
cv::Mat cv_shotcolor_signature;
cv::Mat cv_shotcolor_signature2;
float f1,f2,f3,f4;
bool bf1,bf2,bf3;
std::vector<cv::DMatch> Matches;



float normal_radius = 0.01;
float harris_radius = 0.01;
float support_radius= 0.05;

pcl::PointXYZRGB picked_pt;
std::vector<int> picked_pt_indice(1);
std::vector<float> picked_pt_BOP3_features;





int computeBOP1(bool rgb)
{
	bool r,g,b;
	pcl::PointXYZRGB searchPoint;
	float R = 0.05;

	std::vector<int> pointIdxNKNSearch(BOP_K);
	std::vector<float> pointNKNSquaredDistance(BOP_K);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;


	for(int j=0;j<keypoints->size();j++)
	{
		if(pcl::isFinite(cloud_normals->points[indices->indices[j]]) == true)
		{
			std::vector<bool> features;
			if(rgb==true)
				features.reserve(BOP_K*4);
			else
				features.reserve(BOP_K);

			searchPoint.x = keypoints->points[j].x;
			searchPoint.y = keypoints->points[j].y;
			searchPoint.z = keypoints->points[j].z;

			//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
			int nanpoints = 0;
			int K = BOP_K + 1;
			if ( tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				for (size_t i = 1; i < pointIdxNKNSearch.size(); ++i){

					if(rgb==true)
					{
						if(cloud->points[pointIdxNKNSearch[i]].r > cloud->points[indices->indices[j]].r)
							r=1;
						else
							r=0;
						if(cloud->points[pointIdxNKNSearch[i]].g > cloud->points[indices->indices[j]].g)
							g=1;
						else
							g=0;
						if(cloud->points[pointIdxNKNSearch[i]].b > cloud->points[indices->indices[j]].b)
							b=1;
						else
							b=0;
					}

					pcl::Vector4fMap p = cloud->points[indices->indices[j]].getVector4fMap();
					pcl::Vector4fMap np= cloud_normals->points[indices->indices[j]].getNormalVector4fMap();
					pcl::Vector4fMap q = cloud->points[pointIdxNKNSearch[i] ].getVector4fMap();
					pcl::Vector4fMap nq= cloud_normals->points[pointIdxNKNSearch[i] ].getNormalVector4fMap();
					float convex_test;
					float dot_test ;
					dot_test = np.dot(nq);
					convex_test = (p-q).dot(np-nq);
					//cout<<"convex test :"<<convex_test<<std::endl;
					//cout<<"dot test :"<<pcl::rad2deg(acos(dot_test))<<std::endl;
					if(pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]]) == false)
					{
						bf3 = 0;
					}
					else{
						bf1 = bool(convex_test < 0);
						bf2 = bool(pcl::rad2deg(acos(dot_test)) < 30);
						bf3 = bf1 && bf2;
					}
					//push features for each comparison
					features.push_back(bf3);
					if(rgb==true)
					{
						features.push_back(r); features.push_back(g); features.push_back(b);
					}
				}
			}
			BOP1_features.push_back(features);

		}
		else
			std::cout<<"NaN keypoint!"<<std::endl;
	}
	//cloud 2


	pcl::PointXYZRGB searchPoint2;


	std::vector<int> pointIdxNKNSearch2(BOP_K);
	std::vector<float> pointNKNSquaredDistance2(BOP_K);

	std::vector<int> pointIdxRadiusSearch2;
	std::vector<float> pointRadiusSquaredDistance2;


	for(int j=0;j<keypoints2->size();j++)
	{
		if(pcl::isFinite(cloud2_normals->points[indices2->indices[j]]) == true)
		{
			std::vector<bool> features;
			if(rgb==true)
				features.reserve(BOP_K*4);
			else
				features.reserve(BOP_K);

			searchPoint2.x = keypoints2->points[j].x;
			searchPoint2.y = keypoints2->points[j].y;
			searchPoint2.z = keypoints2->points[j].z;

			//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
			int nanpoints = 0;
			int K = BOP_K + 1;
			if ( tree2->nearestKSearch (searchPoint2, K, pointIdxNKNSearch2, pointNKNSquaredDistance2) > 0 )
			{
				for (size_t i = 1; i < pointIdxNKNSearch2.size(); ++i){

					if(rgb==true)
					{
						if(cloud2->points[pointIdxNKNSearch2[i]].r > cloud2->points[indices2->indices[j]].r)
							r=1;
						else
							r=0;
						if(cloud2->points[pointIdxNKNSearch2[i]].g > cloud2->points[indices2->indices[j]].g)
							g=1;
						else
							g=0;
						if(cloud2->points[pointIdxNKNSearch2[i]].b > cloud2->points[indices2->indices[j]].b)
							b=1;
						else
							b=0;
					}

					pcl::Vector4fMap p = cloud2->points[indices2->indices[j]].getVector4fMap();
					pcl::Vector4fMap np= cloud2_normals->points[indices2->indices[j]].getNormalVector4fMap();
					pcl::Vector4fMap q = cloud2->points[pointIdxNKNSearch2[i] ].getVector4fMap();
					pcl::Vector4fMap nq= cloud2_normals->points[pointIdxNKNSearch2[i] ].getNormalVector4fMap();
					float convex_test;
					float dot_test ;
					dot_test = np.dot(nq);
					convex_test = (p-q).dot(np-nq);
					//cout<<"convex test :"<<convex_test<<std::endl;
					//cout<<"dot test :"<<pcl::rad2deg(acos(dot_test))<<std::endl;

					if(pcl::isFinite(cloud2_normals->points[pointIdxNKNSearch2[i]]) == false)
					{
						bf3 = 0;
					}
					else{
						bf1 = bool(convex_test < 0);
						bf2 = bool(pcl::rad2deg(acos(dot_test)) < 30);
						bf3 = bf1 && bf2;
					}
					//push features for each comparison
					features.push_back(bf3);
					if(rgb==true)
					{
						features.push_back(r); features.push_back(g); features.push_back(b);
					}
				}
			}
			BOP1_features2.push_back(features);

		}
		else
			std::cout<<"NaN keypoint!"<<std::endl;
	}

	return 1;
}

int computeBOP3(bool rgb)
{
	bool r,g,b;
	pcl::PointXYZRGB searchPoint;
	float R = 0.05;

	std::vector<int> pointIdxNKNSearch(BOP_K);
	std::vector<float> pointNKNSquaredDistance(BOP_K);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;


	for(int j=0;j<keypoints->size();j++)
	{
		if(pcl::isFinite(cloud_normals->points[indices->indices[j]]) == true)
		{
			std::vector<bool> features;
			if(rgb==true)
				features.reserve(BOP_K*6);
			else
				features.reserve(BOP_K*3);
			searchPoint.x = keypoints->points[j].x;
			searchPoint.y = keypoints->points[j].y;
			searchPoint.z = keypoints->points[j].z;

			//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
			int nanpoints = 0;
			int K = BOP_K + 1;
			if ( tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				for (size_t i = 1; i < pointIdxNKNSearch.size(); ++i){
					if(rgb==true)
					{
						if(cloud->points[pointIdxNKNSearch[i]].r > cloud->points[indices->indices[j]].r)
							r=1;
						else
							r=0;
						if(cloud->points[pointIdxNKNSearch[i]].g > cloud->points[indices->indices[j]].g)
							g=1;
						else
							g=0;
						if(cloud->points[pointIdxNKNSearch[i]].b > cloud->points[indices->indices[j]].b)
							b=1;
						else
							b=0;
					}
					pcl::computePairFeatures(cloud->points[indices->indices[j]].getVector4fMap(),cloud_normals->points[indices->indices[j]].getNormalVector4fMap(),cloud->points[pointIdxNKNSearch[i] ].getVector4fMap(),cloud_normals->points[pointIdxNKNSearch[i] ].getNormalVector4fMap(),f1,f2,f3,f4);
					//std::cout<<pcl::isFinite(cloud->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud_normals->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud->points[pointIdxNKNSearch[i]])<<" "<<pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]])<< std::endl;
					//std::cout<<"Pair features: "<<f1<<" "<<f2<<" "<<f3<<" "<<f4<<std::endl;
					//std::cout<<"Pair features: "<< pcl::rad2deg(f1)<<" "<<pcl::rad2deg(acos(f2))<<" "<<pcl::rad2deg(acos(f3))<<" "<<f4<<std::endl;

					if(pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]]) == false)
					{
						bf1 = 0;
						bf2 = 0;
						bf3 = 0;
						nanpoints++;
					}
					else{
						if(pcl::rad2deg(f1) > 45)
							bf1 = 1;
						else
							bf1 = 0;

						if(pcl::rad2deg(acos(f2)) > 0)
							bf2 = 1;
						else
							bf2 = 0;

						if(pcl::rad2deg(acos(f3)) > 90)
							bf3 = 1;
						else
							bf3 = 0;
					}
					//push features for each comparison
					features.push_back(bf1); features.push_back(bf2); features.push_back(bf3);
					if(rgb==true)
					{
						features.push_back(r); features.push_back(g); features.push_back(b);
					}
				}
				BOP3_features.push_back(features);
			}


		}
		else
			std::cout<<"NaN keypoint!"<<std::endl;
	}

	//cloud 2


	pcl::PointXYZRGB searchPoint2;


	std::vector<int> pointIdxNKNSearch2(BOP_K);
	std::vector<float> pointNKNSquaredDistance2(BOP_K);

	std::vector<int> pointIdxRadiusSearch2;
	std::vector<float> pointRadiusSquaredDistance2;


	for(int j=0;j<keypoints2->size();j++)
	{
		if(pcl::isFinite(cloud2_normals->points[indices2->indices[j]]) == true)
		{
			std::vector<bool> features;
			if(rgb==true)
				features.reserve(BOP_K*6);
			else
				features.reserve(BOP_K*3);

			searchPoint2.x = keypoints2->points[j].x;
			searchPoint2.y = keypoints2->points[j].y;
			searchPoint2.z = keypoints2->points[j].z;

			//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
			int nanpoints = 0;
			int K = BOP_K + 1;
			if ( tree2->nearestKSearch (searchPoint2, K, pointIdxNKNSearch2, pointNKNSquaredDistance2) > 0 )
			{
				for (size_t i = 1; i < pointIdxNKNSearch2.size(); ++i){
					if(rgb==true)
					{
						if(cloud2->points[pointIdxNKNSearch2[i]].r > cloud2->points[indices2->indices[j]].r)
							r=1;
						else
							r=0;
						if(cloud2->points[pointIdxNKNSearch2[i]].g > cloud2->points[indices2->indices[j]].g)
							g=1;
						else
							g=0;
						if(cloud2->points[pointIdxNKNSearch2[i]].b > cloud2->points[indices2->indices[j]].b)
							b=1;
						else
							b=0;
					}
					pcl::computePairFeatures(cloud2->points[indices2->indices[j]].getVector4fMap(),cloud2_normals->points[indices2->indices[j]].getNormalVector4fMap(),cloud2->points[pointIdxNKNSearch2[i] ].getVector4fMap(),cloud2_normals->points[pointIdxNKNSearch2[i] ].getNormalVector4fMap(),f1,f2,f3,f4);
					//std::cout<<pcl::isFinite(cloud->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud_normals->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud->points[pointIdxNKNSearch[i]])<<" "<<pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]])<< std::endl;
					//std::cout<<"Pair features: "<<f1<<" "<<f2<<" "<<f3<<" "<<f4<<std::endl;
					//std::cout<<"Pair features: "<< pcl::rad2deg(f1)<<" "<<pcl::rad2deg(acos(f2))<<" "<<pcl::rad2deg(acos(f3))<<" "<<f4<<std::endl;

					if(pcl::isFinite(cloud2_normals->points[pointIdxNKNSearch2[i]]) == false)
					{
						bf1 = 0;
						bf2 = 0;
						bf3 = 0;
						nanpoints++;
					}
					else{
						if(pcl::rad2deg(f1) > 45)
							bf1 = 1;
						else
							bf1 = 0;

						if(pcl::rad2deg(acos(f2)) > 0)
							bf2 = 1;
						else
							bf2 = 0;

						if(pcl::rad2deg(acos(f3)) > 90)
							bf3 = 1;
						else
							bf3 = 0;
					}
					//push features for each comparison
					features.push_back(bf1); features.push_back(bf2); features.push_back(bf3);
					if(rgb==true)
					{
						features.push_back(r); features.push_back(g); features.push_back(b);
					}
				}
				BOP3_features2.push_back(features);
			}


		}
		else
			std::cout<<"NaN keypoint!"<<std::endl;
	}


	return 1;
}

int computeSingleBOP3(bool rgb)
{
	picked_pt_BOP3_features.clear();
	bool r,g,b;
	pcl::PointXYZRGB searchPoint;
	float R = 0.05;

	std::vector<int> pointIdxNKNSearch(BOP_K);
	std::vector<float> pointNKNSquaredDistance(BOP_K);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;


	if(rgb==true)
		picked_pt_BOP3_features.reserve(BOP_K*6);
	else
		picked_pt_BOP3_features.reserve(BOP_K*3);
	searchPoint.x = picked_pt.x;
	searchPoint.y = picked_pt.y;
	searchPoint.z = picked_pt.z;

	//std::cout<<"Nearest K points: "<< BOP_K <<std::endl;
	int nanpoints = 0;
	int K = BOP_K + 1;
	if ( tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		for (size_t i = 1; i < pointIdxNKNSearch.size(); ++i){
			if(rgb==true)
			{
				if(cloud->points[pointIdxNKNSearch[i]].r > picked_pt.r)
					r=1;
				else
					r=0;
				if(cloud->points[pointIdxNKNSearch[i]].g > picked_pt.g)
					g=1;
				else
					g=0;
				if(cloud->points[pointIdxNKNSearch[i]].b > picked_pt.b)
					b=1;
				else
					b=0;
			}
			pcl::computePairFeatures(cloud->points[picked_pt_indice[0]].getVector4fMap(),cloud_normals->points[picked_pt_indice[0]].getNormalVector4fMap(),cloud->points[pointIdxNKNSearch[i] ].getVector4fMap(),cloud_normals->points[pointIdxNKNSearch[i] ].getNormalVector4fMap(),f1,f2,f3,f4);
			//std::cout<<pcl::isFinite(cloud->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud_normals->points[indices->indices[j]])<<" "<<pcl::isFinite(cloud->points[pointIdxNKNSearch[i]])<<" "<<pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]])<< std::endl;
			//std::cout<<"Pair features: "<<f1<<" "<<f2<<" "<<f3<<" "<<f4<<std::endl;
			//std::cout<<"Pair features: "<< pcl::rad2deg(f1)<<" "<<pcl::rad2deg(acos(f2))<<" "<<pcl::rad2deg(acos(f3))<<" "<<f4<<std::endl;

			if(pcl::isFinite(cloud_normals->points[pointIdxNKNSearch[i]]) == false)
			{
				bf1 = 0;
				bf2 = 0;
				bf3 = 0;
				nanpoints++;
			}
			else{
				if(pcl::rad2deg(f1) > 45)
					bf1 = 1;
				else
					bf1 = 0;

				if(pcl::rad2deg(acos(f2)) > 0)
					bf2 = 1;
				else
					bf2 = 0;

				if(pcl::rad2deg(acos(f3)) > 90)
					bf3 = 1;
				else
					bf3 = 0;
			}
			//push features for each comparison
			picked_pt_BOP3_features.push_back(bf1); picked_pt_BOP3_features.push_back(bf2); picked_pt_BOP3_features.push_back(bf3);
			if(rgb==true)
			{
				picked_pt_BOP3_features.push_back(r); picked_pt_BOP3_features.push_back(g); picked_pt_BOP3_features.push_back(b);
			}
		}
	}


	else
		std::cout<<"NaN keypoint!"<<std::endl;


	return 1;
}

int convertBOP3toCV()
{
	int r = BOP3_features.size();
	int c = BOP3_features[0].size();
	c = c/8;
	cvBOP_features = cv::Mat(r,c,CV_8U);

	for(int j=0;j<r;j++){
		for(int i=0;i<c;i++){
			uchar val = 0;
			for(int k=0;k<8;k++){
				uchar val2;
				val2 =  BOP3_features[j][i*8+k];
				val2 << 7-k;
				val = val + val2;
			}
			cvBOP_features.at<uchar>(j,i) = val;
		}
	}

	int r2 = BOP3_features2.size();
	int c2 = BOP3_features2[0].size();
	c2 = c2/8;
	cvBOP_features2 = cv::Mat(r2,c2,CV_8U);

	for(int j=0;j<r2;j++){
		for(int i=0;i<c2;i++){
			uchar val = 0;
			for(int k=0;k<8;k++){
				uchar val2;
				val2 =  BOP3_features2[j][i*8+k];
				val2 << 7-k;
				val = val + val2;
			}
			cvBOP_features2.at<uchar>(j,i) = val;
		}
	}

	return 1;
}
int convertBOP1toCV()
{
	int r = BOP1_features.size();
	int c = BOP1_features[0].size();
	c = c/8;
	cvBOP1_features = cv::Mat(r,c,CV_8U);

	for(int j=0;j<r;j++){
		for(int i=0;i<c;i++){
			uchar val = 0;
			for(int k=0;k<8;k++){
				uchar val2;
				val2 =  BOP1_features[j][i*8+k];
				val2 << 7-k;
				val = val + val2;
			}
			cvBOP1_features.at<uchar>(j,i) = val;
		}
	}

	int r2 = BOP1_features2.size();
	int c2 = BOP1_features2[0].size();
	c2 = c2/8;
	cvBOP1_features2 = cv::Mat(r2,c2,CV_8U);

	for(int j=0;j<r2;j++){
		for(int i=0;i<c2;i++){
			uchar val = 0;
			for(int k=0;k<8;k++){
				uchar val2;
				val2 =  BOP1_features2[j][i*8+k];
				val2 << 7-k;
				val = val + val2;
			}
			cvBOP1_features2.at<uchar>(j,i) = val;
		}
	}

	return 1;
}
//int convertBOP3toCV()
//{
//	int r = BOP3_features.size();
//	int c = BOP3_features[0].size();
//	cvBOP_features = cv::Mat(r,c,CV_8U);
//
//	for(int j=0;j<r;j++){
//		for(int i=0;i<c;i++){
//			cvBOP_features.at<uchar>(j,i) = BOP3_features[j][i];
//		}
//	}
//
//	int r2 = BOP3_features2.size();
//	int c2 = BOP3_features2[0].size();
//	cvBOP_features2 = cv::Mat(r2,c2,CV_8U);
//
//	for(int j=0;j<r2;j++){
//		for(int i=0;i<c2;i++){
//			cvBOP_features2.at<uchar>(j,i) = BOP3_features2[j][i];
//		}
//	}
//
//	return 1;
//}

std::vector<cv::DMatch> matchBF_Hamming(cv::Mat &descriptor1, cv::Mat &descriptor2)
{
	cv::BFMatcher matcher(cv::NORM_HAMMING,true);
	std::vector<cv::DMatch> matches;
	matcher.match(descriptor1,descriptor2,matches);
	for(int i=0;i<matches.size();i++)
		std::cout<<"Match "<<i<<": "<<matches[i].queryIdx<<" "<<matches[i].trainIdx<<" "<<matches[i].imgIdx<<" "<<matches[i].distance<<std::endl;
	return matches;
}

std::vector<cv::DMatch> matchBF_Hamming2(cv::Mat &descriptor1, cv::Mat &descriptor2)
{
	cv::BFMatcher matcher(cv::NORM_HAMMING);
	std::vector<std::vector<cv::DMatch> > matches;
	matcher.knnMatch(descriptor1, descriptor2, matches, 2);  // Find two nearest matches
	std::vector<cv::DMatch> good_matches;
	for (int i = 0; i < matches.size(); i++)
	{
	    const float ratio = 0.8; // As in Lowe's paper; can be tuned
	    if (matches[i][0].distance < ratio * matches[i][1].distance)
	    {
	        good_matches.push_back(matches[i][0]);
	    }
	}
	for(int i=0;i<good_matches.size();i++)
		std::cout<<"Match "<<i<<": "<<good_matches[i].queryIdx<<" "<<good_matches[i].trainIdx<<" "<<good_matches[i].imgIdx<<" "<<good_matches[i].distance<<std::endl;
	return good_matches;
}

std::vector<cv::DMatch> matchBF_L2(cv::Mat &descriptor1, cv::Mat &descriptor2)
{
	cv::BFMatcher matcher(cv::NORM_L2,true);
	std::vector<cv::DMatch> matches;
	matcher.match(descriptor1,descriptor2,matches);
	for(int i=0;i<matches.size();i++)
		std::cout<<"Match "<<i<<": "<<matches[i].queryIdx<<" "<<matches[i].trainIdx<<" "<<matches[i].imgIdx<<" "<<matches[i].distance<<std::endl;
	return matches;
}

int computeFPFH()
{
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	fpfh.setSearchMethod(tree);;
	fpfh.setInputNormals(cloud_normals);
	fpfh.setRadiusSearch(support_radius);
	fpfh.setSearchSurface(cloud);
	fpfh.setInputCloud(keypoints_xyzrgb);
	fpfh.compute(*fpfh_signature);

	//CLoud 2
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints2, *keypoints2_xyzrgb);
	fpfh2.setSearchMethod(tree2);;
	fpfh2.setInputNormals(cloud2_normals);
	fpfh2.setRadiusSearch(support_radius);
	fpfh2.setSearchSurface(cloud2);
	fpfh2.setInputCloud(keypoints2_xyzrgb);
	fpfh2.compute(*fpfh_signature2);


	return 1;
}

int computeSHOT()
{
	pcl::SHOTEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	shot.setSearchMethod(tree);;
	shot.setInputNormals(cloud_normals);
	shot.setRadiusSearch(support_radius);
	shot.setSearchSurface(cloud);
	shot.setInputCloud(keypoints_xyzrgb);
	shot.compute(*shot_signature);

	//CLoud 2
	pcl::SHOTEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints2, *keypoints2_xyzrgb);
	shot2.setSearchMethod(tree2);;
	shot2.setInputNormals(cloud2_normals);
	shot2.setRadiusSearch(support_radius);
	shot2.setSearchSurface(cloud2);
	shot2.setInputCloud(keypoints2_xyzrgb);
	shot2.compute(*shot_signature2);

	return 1;
}

int computeSHOTcolor()
{
	pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	shot.setSearchMethod(tree);;
	shot.setInputNormals(cloud_normals);
	shot.setRadiusSearch(support_radius);
	shot.setSearchSurface(cloud);
	shot.setInputCloud(keypoints_xyzrgb);
	shot.compute(*shotcolor_signature);

	//CLoud 2
	pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints2, *keypoints2_xyzrgb);
	shot2.setSearchMethod(tree2);;
	shot2.setInputNormals(cloud2_normals);
	shot2.setRadiusSearch(support_radius);
	shot2.setSearchSurface(cloud2);
	shot2.setInputCloud(keypoints2_xyzrgb);
	shot2.compute(*shotcolor_signature2);

	return 1;
}

int convertFPFHtoCV()
{
	int num_points = fpfh_signature->width;
	if(num_points > 0){
		cv_fpfh_signature = cv::Mat(num_points,33,CV_32F);
		for(int j=0;j<num_points;j++){
			for(int i=0;i<33;i++){
				cv_fpfh_signature.at<float>(j,i) = fpfh_signature->points[j].histogram[i];
				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
			}
			//std::cout<<std::endl;

		}
	}

	int num_points2 = fpfh_signature2->width;
	if(num_points2 > 0){
		cv_fpfh_signature2 = cv::Mat(num_points2,33,CV_32F);
		for(int j=0;j<num_points2;j++){
			for(int i=0;i<33;i++){
				cv_fpfh_signature2.at<float>(j,i) = fpfh_signature2->points[j].histogram[i];
				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
			}
			//std::cout<<std::endl;

		}
	}

	return 1;
}


int convertSHOTtoCV()
{
	int num_points = shot_signature->width;
	if(num_points > 0){
		cv_shot_signature = cv::Mat(num_points,352,CV_32F);
		for(int j=0;j<num_points;j++){
			for(int i=0;i<352;i++){
				cv_shot_signature.at<float>(j,i) = shot_signature->points[j].descriptor[i];
				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
			}
			//std::cout<<std::endl;

		}
	}

	int num_points2 = shot_signature2->width;
	if(num_points2 > 0){
		cv_shot_signature2 = cv::Mat(num_points2,352,CV_32F);
		for(int j=0;j<num_points2;j++){
			for(int i=0;i<352;i++){
				cv_shot_signature2.at<float>(j,i) = shot_signature2->points[j].descriptor[i];
				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
			}
			//std::cout<<std::endl;

		}
	}

	return 1;
}

int convertSHOTcolortoCV()
{
	int num_points = shotcolor_signature->width;
	if(num_points > 0){
		cv_shotcolor_signature = cv::Mat(num_points,1344,CV_32F);
		for(int j=0;j<num_points;j++){
			for(int i=0;i<1344;i++){
				cv_shotcolor_signature.at<float>(j,i) = shotcolor_signature->points[j].descriptor[i];
				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
			}
			//std::cout<<std::endl;

		}
	}

	int num_points2 = shotcolor_signature2->width;
	if(num_points2 > 0){
		cv_shotcolor_signature2 = cv::Mat(num_points2,1344,CV_32F);
		for(int j=0;j<num_points2;j++){
			for(int i=0;i<1344;i++){
				cv_shotcolor_signature2.at<float>(j,i) = shotcolor_signature2->points[j].descriptor[i];
				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
			}
			//std::cout<<std::endl;

		}
	}

	return 1;
}

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	if (event.getPointIndex () == -1)
		return;
	std::cout <<"Picked point index: "<< event.getPointIndex() << std::endl;
	//pcl::PointXYZRGB picked_pt;
	event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
	std::cout <<"x: "<< picked_pt.x<<"; y: "<< picked_pt.y<<"; z: "<< picked_pt.z<< std::endl;
	//std::vector<int> indices (1);
	std::vector<float> distances (1);
	tree->nearestKSearch (picked_pt, 1, picked_pt_indice, distances);
	std::cout<<picked_pt_indice[0]<<" "<<cloud->points[picked_pt_indice[0]].x<<" "<<cloud->points[picked_pt_indice[0]].y<<" "<<cloud->points[picked_pt_indice[0]].z<<std::endl;
	picked_pt = cloud->points[picked_pt_indice[0]];
	std::cout <<"x: "<< picked_pt.x<<"; y: "<< picked_pt.y<<"; z: "<< picked_pt.z<< std::endl;

	computeSingleBOP3(false);
	pcl::PointCloud<BOP3hist> bop3hist;
	bop3hist.resize(1);
	std::cout<<"picked_pt : "<<picked_pt_BOP3_features.size()<<std::endl;
	for(int i=0;i<picked_pt_BOP3_features.size();i++)
	{
		bop3hist.points[0].histogram[i] = picked_pt_BOP3_features[i];
		//std::cout<<"hist : "<<bop3hist.points[0].histogram[i]<<std::endl;
	}

	pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter("Feature");
	plotter->addFeatureHistogram(bop3hist,picked_pt_BOP3_features.size());
	plotter->plot();
	plotter->spinOnce (2000);

	//extract_and_save();
	//viewer.addSphere(picked_pt,0.05,"sphere",0);
}

int main (int argc, char* argv[])
{

	//read clouds
	pcl::PCDReader reader;
	testfile1 = argv[1];
	reader.read<pcl::PointXYZRGB>(testfile1,*cloud);

	testfile2 = argv[2];
	reader.read<pcl::PointXYZRGB>(testfile2,*cloud2);


	bool ShowNormals;
	bool ShowKeypoints;
	bool ShowMatches;

	ShowNormals = bool(atoi(argv[3]));
	ShowKeypoints = bool(atoi(argv[4]));
	ShowMatches = bool(atoi(argv[5]));

	int feature_type;
	// 1=BOP3 ; 2=FPFH ; 3=SHOT:
	feature_type = atoi(argv[6]);

	//Extract normals
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (normal_radius);
	ne.compute (*cloud_normals);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne2;
	ne2.setInputCloud (cloud2);
	ne2.setSearchMethod (tree2);
	ne2.setRadiusSearch (normal_radius);
	ne2.compute (*cloud2_normals);

	//Extract keypoints
	harris.setInputCloud(cloud);
	harris.setNormals(cloud_normals);
	harris.setNonMaxSupression(true);
	harris.setRefine(false);
	//harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal>::LOWE);
	harris.setRadius(harris_radius);
	harris.compute(*keypoints);
	for(int j=0;j<keypoints->size();j++)
	{
		std::cout<<"Keypoints "<<j<<" x: "<<keypoints->points[j].x<<" y: "<<keypoints->points[j].y<<" z: "<<keypoints->points[j].z<<std::endl;
	}
	indices = harris.getKeypointsIndices();

	//Extract keypoints of cloud2
	harris2.setInputCloud(cloud2);
	harris2.setNormals(cloud2_normals);
	harris2.setNonMaxSupression(true);
	harris2.setRefine(false);
	//harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal>::LOWE);
	harris2.setRadius(harris_radius);
	harris2.compute(*keypoints2);
	for(int j=0;j<keypoints2->size();j++)
	{
		std::cout<<"Keypoints2 "<<j<<" x: "<<keypoints2->points[j].x<<" y: "<<keypoints2->points[j].y<<" z: "<<keypoints2->points[j].z<<std::endl;
	}
	indices2 = harris2.getKeypointsIndices();

	if(ShowMatches == 1)
	{
		double time_A, time_B,time_C,time_D;
		double calctime, matchtime;
		double tickfreq= cv::getTickFrequency();
		if(feature_type == 1)
		{
			time_A = cv::getTickCount();
			computeBOP3(false);
			time_B = cv::getTickCount();
			convertBOP3toCV();
			time_C = cv::getTickCount();
			Matches = matchBF_Hamming(cvBOP_features,cvBOP_features2);
			time_D = cv::getTickCount();
			calctime = (time_B - time_A)/tickfreq;
			matchtime = (time_D - time_C)/tickfreq;
			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
		}
		else if(feature_type == 2)
		{
			time_A = cv::getTickCount();
			computeBOP3(true);
			time_B = cv::getTickCount();
			convertBOP3toCV();
			time_C = cv::getTickCount();
			Matches = matchBF_Hamming(cvBOP_features,cvBOP_features2);
			time_D = cv::getTickCount();
			calctime = (time_B - time_A)/tickfreq;
			matchtime = (time_D - time_C)/tickfreq;
			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
		}
		else if(feature_type == 3)
		{
			time_A = cv::getTickCount();
			computeBOP1(false);
			time_B = cv::getTickCount();
			convertBOP1toCV();
			time_C = cv::getTickCount();
			Matches = matchBF_Hamming(cvBOP1_features,cvBOP1_features2);
			time_D = cv::getTickCount();
			calctime = (time_B - time_A)/tickfreq;
			matchtime = (time_D - time_C)/tickfreq;
			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
		}
		else if(feature_type == 4)
		{
			time_A = cv::getTickCount();
			computeBOP1(true);
			time_B = cv::getTickCount();
			convertBOP1toCV();
			time_C = cv::getTickCount();
			Matches = matchBF_Hamming(cvBOP1_features,cvBOP1_features2);
			time_D = cv::getTickCount();
			calctime = (time_B - time_A)/tickfreq;
			matchtime = (time_D - time_C)/tickfreq;
			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
		}
		else if(feature_type == 5)
		{
			time_A = cv::getTickCount();
			computeFPFH();
			time_B = cv::getTickCount();
			convertFPFHtoCV();
			time_C = cv::getTickCount();
			Matches = matchBF_L2(cv_fpfh_signature,cv_fpfh_signature2);
			time_D = cv::getTickCount();
			calctime = (time_B - time_A)/tickfreq;
			matchtime = (time_D - time_C)/tickfreq;
			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
		}
		else if(feature_type == 6)
		{
			time_A = cv::getTickCount();
			computeSHOT();
			time_B = cv::getTickCount();
			convertSHOTtoCV();
			time_C = cv::getTickCount();
			Matches = matchBF_L2(cv_shot_signature,cv_shot_signature2);
			time_D = cv::getTickCount();
			calctime = (time_B - time_A)/tickfreq;
			matchtime = (time_D - time_C)/tickfreq;
			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
		}
		else if(feature_type == 7)
		{
			time_A = cv::getTickCount();
			computeSHOTcolor();
			time_B = cv::getTickCount();
			convertSHOTcolortoCV();
			time_C = cv::getTickCount();
			Matches = matchBF_L2(cv_shotcolor_signature,cv_shotcolor_signature2);
			time_D = cv::getTickCount();
			calctime = (time_B - time_A)/tickfreq;
			matchtime = (time_D - time_C)/tickfreq;
			cout<<"Time to calculate :"<<calctime*1000<<std::endl;
			cout<<"Time to match     :"<<matchtime*1000<<std::endl;
		}

	}

	// VIEWER!

	// Shift the first clouds' points to the left
	const Eigen::Vector3f translate (0.1, 0.0, 0.0);
	const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
	pcl::transformPointCloud (*cloud, *cloud_shifted, translate, no_rotation);
	pcl::transformPointCloud (*keypoints, *keypoints_shifted, translate, no_rotation);

	// Shift the second clouds' points to the right
	pcl::transformPointCloud (*cloud2, *cloud2_shifted, -translate, no_rotation);
	pcl::transformPointCloud (*keypoints2, *keypoints2_shifted, -translate, no_rotation);




	//	viewer.setBackgroundColor (255, 255, 255);



	pcl::visualization::PointCloudColorHandlerRGBField< pcl:: PointXYZRGB > color (cloud);
	pcl::visualization::PointCloudColorHandlerRGBField< pcl:: PointXYZRGB > color2 (cloud2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorhandler (cloud,255,255,0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorhandler2 (cloud2,255,255,0);



	viewer.addPointCloud<pcl::PointXYZRGB> (cloud_shifted,color, "cloud");
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud2_shifted,color2, "cloud2");



	if(ShowKeypoints == 1){
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>  keypointscolorHandler (keypoints, 0,255,0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>  keypointscolorHandler2 (keypoints2, 0,255,0);

		viewer.addPointCloud<pcl::PointXYZI> (keypoints_shifted,keypointscolorHandler, "keypoints");
		viewer.setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");

		viewer.addPointCloud<pcl::PointXYZI> (keypoints2_shifted,keypointscolorHandler2, "keypoints2");
		viewer.setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints2");
	}

	if(ShowNormals == 1){
		//viewer.setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_COLOR,0,0,0,"normals");
		viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_shifted, cloud_normals, 10, 0.01, "normals");
		viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud2_shifted, cloud2_normals, 10, 0.01, "normals2");
	}

	if(ShowMatches == 1)
	{
	int id1, id2;
		for(int i=0;i<Matches.size();i++)
		{
			id1 = Matches[i].queryIdx;
			id2 = Matches[i].trainIdx;
		    std::stringstream ss ("line");
		    ss << i;
			viewer.addLine(cloud_shifted->points[indices->indices[id1]], cloud2_shifted->points[indices2->indices[id2]],255,0,0,ss.str());
		}
	}


	viewer.initCameraParameters ();

	viewer.registerPointPickingCallback (pp_callback, (void*)&viewer);

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();

	}


	return 1;
}



