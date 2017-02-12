/*
 * FeatureExtractor.cpp
 *
 *  Created on: Dec 1, 2013
 *      Author: berker
 */

#include "FeatureExtractor.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

FeatureExtractor::FeatureExtractor() {
	// TODO Auto-generated constructor stub
	Flag_VFH = 0;
	Flag_GFPFH = 0;
	Flag_CVFH = 0;
	Flag_OURCVFH = 0;
	Flag_ESF = 0;
	Flag_PFH = 0;
	Flag_PFHRGB = 0;
	Flag_FPFH = 0;
	Flag_SHOT = 0;
	Flag_SHOTColor = 0;
	Flag_3DSC = 0;
	Flag_USC = 0;
	nn_search_radius = 0.01;
	harris_radius = 0.01;
	support_radius = 0.10;
	config_features_file = "config_features.txt";
	Flag_Normals = 0;
	KeypointsType = 2;
	SamplingDistance = 0.01;

}

FeatureExtractor::~FeatureExtractor() {
	// TODO Auto-generated destructor stub
}
float FeatureExtractor::SafeAcos (float x)
  {
  if (x < -1.0) x = -1.0 ;
  else if (x > 1.0) x = 1.0 ;
  return acos (x) ;
  }

void FeatureExtractor::RGB2HSV(float r, float g, float b, float &h, float &s, float &v)
{
    float K = 0.f;

    if (g < b)
    {
        std::swap(g, b);
        K = -1.f;
    }

    if (r < g)
    {
        std::swap(r, g);
        K = -2.f / 6.f - K;
    }

    float chroma = r - std::min(g, b);
    h = fabs(K + (g - b) / (6.f * chroma + 1e-20f));
    s = chroma / (r + 1e-20f);
    v = r;
}

void FeatureExtractor::ConfigFeatures()
{
	flags.clear();
	feature_names.clear();

	std::string feature_name;
	bool flag;

	std::ifstream file;
	file.open(config_features_file.c_str());

	while(!file.eof())
	{
		file>>feature_name>>flag;
		std::cout<<feature_name<<" "<<flag<<std::endl;
		feature_names.push_back(feature_name);
		flags.push_back(flag);
	}
	file.close();

	//Check VFH
	std::vector<std::string>::iterator it;
	it = std::find(feature_names.begin(), feature_names.end(), "VFH");
	int i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_VFH = flags[i];
		if(flags[i]==1)
			std::cout<<"VFH will be extracted!"<<std::endl;
	}

	//Check GFPFH
	it = std::find(feature_names.begin(), feature_names.end(), "GFPFH");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_GFPFH = flags[i];
		if(flags[i]==1)
			std::cout<<"GFPFH will be extracted!"<<std::endl;
	}

	//Check CVFH
	it = std::find(feature_names.begin(), feature_names.end(), "CVFH");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_CVFH = flags[i];
		if(flags[i]==1)
			std::cout<<"CVFH will be extracted!"<<std::endl;
	}

	//Check OURCVFH
	it = std::find(feature_names.begin(), feature_names.end(), "OURCVFH");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_OURCVFH = flags[i];
		if(flags[i]==1)
			std::cout<<"OURCVFH will be extracted!"<<std::endl;
	}

	//Check ESF
	it = std::find(feature_names.begin(), feature_names.end(), "ESF");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_ESF = flags[i];
		if(flags[i]==1)
			std::cout<<"ESF will be extracted!"<<std::endl;
	}

	//Check PFH
	it = std::find(feature_names.begin(), feature_names.end(), "PFH");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_PFH = flags[i];
		if(flags[i]==1)
			std::cout<<"PFH will be extracted!"<<std::endl;
	}

	//Check PFHRGB
	it = std::find(feature_names.begin(), feature_names.end(), "PFHRGB");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_PFHRGB = flags[i];
		if(flags[i]==1)
			std::cout<<"PFHRGB will be extracted!"<<std::endl;
	}

	//Check FPFH
	it = std::find(feature_names.begin(), feature_names.end(), "FPFH");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_FPFH = flags[i];
		if(flags[i]==1)
			std::cout<<"FPFH will be extracted!"<<std::endl;
	}

	//Check SHOT
	it = std::find(feature_names.begin(), feature_names.end(), "SHOT");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_SHOT = flags[i];
		if(flags[i]==1)
			std::cout<<"SHOT will be extracted!"<<std::endl;
	}

	//Check SHOTColor
	it = std::find(feature_names.begin(), feature_names.end(), "SHOTColor");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_SHOTColor = flags[i];
		if(flags[i]==1)
			std::cout<<"SHOTColor will be extracted!"<<std::endl;
	}

	//Check 3DSC
	it = std::find(feature_names.begin(), feature_names.end(), "3DSC");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_3DSC = flags[i];
		if(flags[i]==1)
			std::cout<<"3DSC will be extracted!"<<std::endl;
	}

	//Check USC
	it = std::find(feature_names.begin(), feature_names.end(), "USC");
	i = std::distance( feature_names.begin(), it );
	if (it!=feature_names.end())
	{
		Flag_USC = flags[i];
		if(flags[i]==1)
			std::cout<<"USC will be extracted!"<<std::endl;
	}


}

int FeatureExtractor::ExtractFeatures(std::string pointcloud_list, bool write)
{
	ConfigFeatures();
	int sum=0;
	for(int i=0;i<flags.size();i++)
		sum = sum + int(flags[i]);
	if(sum == 0)
	{
		std::cout<<"No feature is configured for extracting!"<<std::endl;
		return 0;
	}

	std::ifstream file;
	file.open(pointcloud_list.c_str());
	std::string filename;

	pcl::PCDWriter writer;

	while(!file.eof())
	{
		LOOP:
		file>>filename;
		std::size_t found = filename.find("pcd");
		if (found!=std::string::npos){
			std::cout<<"Extracting: "<<filename<<std::endl;

			std::string path,pathlessname;
			unsigned found = filename.find_last_of("/");
			//unsigned found2 = filename.find_first_of(".");
			path = filename.substr(0,found);
			pathlessname = filename.substr(found+1);

			//std::cout<<path<<" + "<<pathlessname<<"\n";


			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::PointCloud<int> sample_indices ;

			pcl::PointIndicesConstPtr indices ( new pcl::PointIndices);



			try{
				pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename.c_str(), *cloud);
			}
			catch(pcl::PCLException& e)
			{
				std::cout<<"error reading!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;

			}

			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

			int cloud_size  = cloud->size();
			std::cout<<"cloud.size: "<<cloud_size<<std::endl;
			if(Flag_ESF == 1 && (cloud_size > 2))
			{
				pcl::ESFEstimation<pcl::PointXYZRGB, pcl::ESFSignature640> esf;
				esf.setInputCloud(cloud);
				pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_signature (new pcl::PointCloud<pcl::ESFSignature640> ());
				esf.compute(*esf_signature);
				std::stringstream ss_savefilename;
				ss_savefilename << pathlessname <<"_ESF";
				std::string savefilename = ss_savefilename.str();

				try{
					writer.writeASCII(savefilename.c_str(),*esf_signature);
				}
				catch(pcl::PCLException& e)
				{
					std::cout<<"error writing ESF signature!!!"<<std::endl;
					goto LOOP;
				}

			}
			//Check if any of the features to be extracted needs calculation of surface normals!
			if(Flag_VFH || Flag_CVFH || Flag_PFH || Flag_PFHRGB || Flag_FPFH || Flag_SHOT || Flag_SHOTColor == 1)
			{
				pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
				ne.setInputCloud (cloud);
				ne.setSearchMethod (tree);
				ne.setRadiusSearch (nn_search_radius);
				ne.compute (*cloud_normals);
				Flag_Normals = 1;
			}
			//Compute descriptor/sampling indices
			if(Flag_FPFH || Flag_PFH || Flag_PFHRGB || Flag_SHOT || Flag_SHOTColor == 1)
			{
				if(KeypointsType == 1){
					pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
					pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal> harris;
					harris.setInputCloud(cloud);
					harris.setNormals(cloud_normals);
					harris.setNonMaxSupression(true);
					harris.setRefine(false);
					//harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal>::CURVATURE);
					harris.setRadius(harris_radius);
					harris.compute(*keypoints);
					pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
					indices = harris.getKeypointsIndices();

				}
				else if(KeypointsType == 2){
					//
					//  ISS3D parameters
					//
					double iss_normal_radius_;
					double iss_border_radius_;
					double iss_salient_radius_;
					double iss_non_max_radius_;
					double iss_gamma_21_ (0.975);
					double iss_gamma_32_ (0.975);
					double iss_min_neighbors_ (5);
					int iss_threads_ (4);
					double model_resolution;
					model_resolution = computeCloudResolution(cloud);
					iss_salient_radius_ = 6 * model_resolution;
					iss_non_max_radius_ = 4 * model_resolution;
					//with be
					iss_normal_radius_ = 4 * model_resolution;
					iss_border_radius_ = 1 * model_resolution;
					pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;

					iss_detector.setSearchMethod (tree);
					iss_detector.setSalientRadius (iss_salient_radius_);
					iss_detector.setNonMaxRadius (iss_non_max_radius_);
					//with be
					iss_detector.setNormalRadius (iss_normal_radius_);
					iss_detector.setBorderRadius (iss_border_radius_);

					iss_detector.setThreshold21 (iss_gamma_21_);
					iss_detector.setThreshold32 (iss_gamma_32_);
					iss_detector.setMinNeighbors (iss_min_neighbors_);
					iss_detector.setNumberOfThreads (iss_threads_);
					iss_detector.setInputCloud (cloud);
					iss_detector.compute (*keypoints_xyzrgb);
					indices = iss_detector.getKeypointsIndices();

				}

				else{
					pcl::UniformSampling<pcl::PointXYZRGB> sampler;
					sampler.setInputCloud(cloud);
					sampler.setRadiusSearch(SamplingDistance);
					sampler.compute(sample_indices);
					pcl::copyPointCloud(*cloud, sample_indices.points, *sampled_cloud);
				}
			}
			if(Flag_VFH == 1)
			{
				pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
				vfh.setSearchMethod(tree);
				vfh.setInputCloud(cloud);
				vfh.setInputNormals(cloud_normals);
				pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_signature (new pcl::PointCloud<pcl::VFHSignature308> ());
				vfh.compute (*vfh_signature);
				std::stringstream ss_savefilename;
				ss_savefilename << pathlessname <<"_VFH_"<<nn_search_radius;
				std::string savefilename = ss_savefilename.str();

				try{
					writer.writeASCII(savefilename.c_str(),*vfh_signature);
				}
				catch(pcl::PCLException& e)
				{
					std::cout<<"error writing VFH signature!!!"<<std::endl;
					goto LOOP;
				}
			}
			if(Flag_FPFH == 1)
			{
				std::stringstream ss_savefilename;
				pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
				pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_signature (new pcl::PointCloud<pcl::FPFHSignature33> ());
				fpfh.setSearchMethod(tree);
				//fpfh.setIndices(sample_indices);
				fpfh.setInputNormals(cloud_normals);
				fpfh.setRadiusSearch(support_radius);
				fpfh.setSearchSurface(cloud);

				if(KeypointsType == 1){
					fpfh.setInputCloud(keypoints_xyzrgb);
					fpfh.compute(*fpfh_signature);

					ss_savefilename << pathlessname <<"_FPFH_sr_"<<support_radius<<"_h3d";
				}
				else if(KeypointsType == 2){
					fpfh.setInputCloud(keypoints_xyzrgb);
					fpfh.compute(*fpfh_signature);

					ss_savefilename << pathlessname <<"_FPFH_sr_"<<support_radius<<"_iss";
				}
				else{
					fpfh.setInputCloud(sampled_cloud);
					fpfh.compute(*fpfh_signature);

					ss_savefilename << pathlessname <<"_FPFH_sr_"<<support_radius<<"_ds_"<<SamplingDistance;
				}

				std::string savefilename = ss_savefilename.str();

				try{
					writer.writeASCII(savefilename.c_str(),*fpfh_signature);
				}
				catch(pcl::PCLException& e)
				{
					std::cout<<"error writing FPFH signature!!!"<<std::endl;
					goto LOOP;
				}

			}
			if(Flag_PFH == 1)
			{
				std::stringstream ss_savefilename;
				pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
				pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_signature (new pcl::PointCloud<pcl::PFHSignature125> ());
				pfh.setSearchMethod(tree);
				//fpfh.setIndices(sample_indices);
				pfh.setInputNormals(cloud_normals);
				pfh.setRadiusSearch(support_radius);
				pfh.setSearchSurface(cloud);

				if(KeypointsType == 1){
					pfh.setInputCloud(keypoints_xyzrgb);
					pfh.compute(*pfh_signature);

					ss_savefilename << pathlessname <<"_PFH_sr_"<<support_radius<<"_h3d";
				}
				else if(KeypointsType == 2){
					pfh.setInputCloud(keypoints_xyzrgb);
					pfh.compute(*pfh_signature);

					ss_savefilename << pathlessname <<"_PFH_sr_"<<support_radius<<"_iss";
				}
				else{
					pfh.setInputCloud(sampled_cloud);
					pfh.compute(*pfh_signature);

					ss_savefilename << pathlessname <<"_PFH_sr_"<<support_radius<<"_ds_"<<SamplingDistance;
				}

				std::string savefilename = ss_savefilename.str();

				try{
					writer.writeASCII(savefilename.c_str(),*pfh_signature);
				}
				catch(pcl::PCLException& e)
				{
					std::cout<<"error writing PFH signature!!!"<<std::endl;
					goto LOOP;
				}

			}
			if(Flag_PFHRGB == 1)
			{
				std::stringstream ss_savefilename;
				pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> pfh;
				pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfh_signature (new pcl::PointCloud<pcl::PFHRGBSignature250> ());
				pfh.setSearchMethod(tree);
				//fpfh.setIndices(sample_indices);
				pfh.setInputNormals(cloud_normals);
				pfh.setRadiusSearch(support_radius);
				pfh.setSearchSurface(cloud);

				if(KeypointsType == 1){
					pfh.setInputCloud(keypoints_xyzrgb);
					pfh.compute(*pfh_signature);

					ss_savefilename << pathlessname <<"_PFHRGB_sr_"<<support_radius<<"_h3d";
				}
				else if(KeypointsType == 2){
					pfh.setInputCloud(keypoints_xyzrgb);
					pfh.compute(*pfh_signature);

					ss_savefilename << pathlessname <<"_PFHRGB_sr_"<<support_radius<<"_iss";
				}
				else{
					pfh.setInputCloud(sampled_cloud);
					pfh.compute(*pfh_signature);

					ss_savefilename << pathlessname <<"_PFHRGB_sr_"<<support_radius<<"_ds_"<<SamplingDistance;
				}

				std::string savefilename = ss_savefilename.str();

				try{
					writer.writeASCII(savefilename.c_str(),*pfh_signature);
				}
				catch(pcl::PCLException& e)
				{
					std::cout<<"error writing PFH signature!!!"<<std::endl;
					goto LOOP;
				}

			}
			if(Flag_SHOT == 1)
			{

				std::stringstream ss_savefilename;
				pcl::SHOTEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
				pcl::PointCloud<pcl::SHOT352>::Ptr shot_signature (new pcl::PointCloud<pcl::SHOT352> ());
				shot.setSearchMethod(tree);
				//fpfh.setIndices(sample_indices);
				shot.setInputNormals(cloud_normals);
				shot.setRadiusSearch(support_radius);
				shot.setSearchSurface(cloud);

				if(KeypointsType == 1){
					shot.setInputCloud(keypoints_xyzrgb);
					shot.compute(*shot_signature);

					ss_savefilename << pathlessname <<"_SHOT_sr_"<<support_radius<<"_h3d";
				}
				else if(KeypointsType == 2){
					shot.setInputCloud(keypoints_xyzrgb);
					shot.compute(*shot_signature);

					ss_savefilename << pathlessname <<"_SHOT_sr_"<<support_radius<<"_iss";
				}
				else{
					shot.setInputCloud(sampled_cloud);
					shot.compute(*shot_signature);

					ss_savefilename << pathlessname <<"_SHOT_sr_"<<support_radius<<"_ds_"<<SamplingDistance;
				}

//				std::stringstream ss_savefilename;
//				pcl::SHOTEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
//				pcl::PointCloud<pcl::SHOT352>::Ptr shot_signature (new pcl::PointCloud<pcl::SHOT352> ());
//				shot.setSearchMethod(tree);
//				//shot.setInputCloud(cloud);
//				//fpfh.setIndices(sample_indices);
//				shot.setInputNormals(cloud_normals);
//				shot.setRadiusSearch(support_radius);
//				shot.setSearchSurface(cloud);
//
//				if(KeypointsType == 1){
//					shot.setInputCloud(keypoints_xyzrgb);
//
//					shot.setIndices(indices);
//					shot.compute(*shot_signature);
//
//					ss_savefilename << pathlessname <<"_SHOT_sr_"<<support_radius<<"_h3d";
//				}
//				else if(KeypointsType == 2){
//					shot.setInputCloud(keypoints_xyzrgb);
//
//					shot.setIndices(indices);
//					shot.compute(*shot_signature);
//
//					ss_savefilename << pathlessname <<"_SHOT_sr_"<<support_radius<<"_iss";
//				}
//				else{
//					shot.setInputCloud(sampled_cloud);
//					//shot.setSearchSurface(sampled_cloud);
//					shot.compute(*shot_signature);
//
//					ss_savefilename << pathlessname <<"_SHOT_sr_"<<support_radius<<"_ds_"<<SamplingDistance;
//				}

				std::string savefilename = ss_savefilename.str();

				try{
					writer.writeASCII(savefilename.c_str(),*shot_signature);
				}
				catch(pcl::PCLException& e)
				{
					std::cout<<"error writing SHOT signature!!!"<<std::endl;
					goto LOOP;
				}

			}
			if(Flag_SHOTColor == 1)
			{
				std::stringstream ss_savefilename;
				pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot;
				pcl::PointCloud<pcl::SHOT1344>::Ptr shotcolor_signature (new pcl::PointCloud<pcl::SHOT1344> ());
				shot.setSearchMethod(tree);
				//fpfh.setIndices(sample_indices);
				shot.setInputNormals(cloud_normals);
				shot.setRadiusSearch(support_radius);
				shot.setSearchSurface(cloud);

				if(KeypointsType == 1){
					shot.setInputCloud(keypoints_xyzrgb);
					shot.compute(*shotcolor_signature);

					ss_savefilename << pathlessname <<"_SHOTColor_sr_"<<support_radius<<"_h3d";
				}
				else if(KeypointsType == 2){
					shot.setInputCloud(keypoints_xyzrgb);
					shot.compute(*shotcolor_signature);

					ss_savefilename << pathlessname <<"_SHOTColor_sr_"<<support_radius<<"_iss";
				}
				else{
					shot.setInputCloud(sampled_cloud);
					shot.compute(*shotcolor_signature);

					ss_savefilename << pathlessname <<"_SHOTColor_sr_"<<support_radius<<"_ds_"<<SamplingDistance;
				}

				std::string savefilename = ss_savefilename.str();

				try{
					writer.writeASCII(savefilename.c_str(),*shotcolor_signature);
				}
				catch(pcl::PCLException& e)
				{
					std::cout<<"error writing SHOTColor signature!!!"<<std::endl;
					goto LOOP;
				}

			}
		}

	}

	return 1;
}



cv::Mat FeatureExtractor::TrainBOW(int numofcenters, const cv::Mat &descriptors)
{
	cv::BOWKMeansTrainer bowtrainer(numofcenters);
	bowtrainer.add(descriptors);
	cv::Mat vocabulary = bowtrainer.cluster();

	return vocabulary;
}


cv::Mat FeatureExtractor::ExtractBOWHistograms(std::string vocabulary_name, std::vector<cv::Mat> &descriptors)
{
	cv::FileStorage fs(vocabulary_name,cv::FileStorage::READ);
	cv::Mat vocabulary;
	fs["vocabulary"]>>vocabulary;
	fs.release();
	std::cout<<"vocabulary size: "<<vocabulary.rows<<" "<<vocabulary.cols<<std::endl;


//	for(int i=0;i<vocabulary.rows;i++){
//		for(int j=0;j<vocabulary.cols;j++)
//			std::cout<<vocabulary.at<float>(i,j)<<" ";
//		std::cout<<std::endl;
//	}


	cv::BFMatcher matcher;
	std::vector<cv::Mat> Vocabulary;
	Vocabulary.push_back(vocabulary);
	matcher.add(Vocabulary);

	std::vector<std::vector<cv::DMatch> > allmatches;

	for(int i=0;i<descriptors.size();i++)
	{
		std::vector<cv::DMatch> matches;
		matcher.match(descriptors[i],matches);
		allmatches.push_back(matches);
	}
	int colsize = vocabulary.rows;
	int rowsize = allmatches.size();
	std::cout<<"allmatches size: "<<rowsize<<std::endl;
	int matchword;
	cv::Mat bowhistograms = cv::Mat::zeros(rowsize,colsize,CV_32F);
	for(int i=0;i<rowsize;i++){
		for(int j=0;j<allmatches[i].size();j++){
			matchword = allmatches[i][j].trainIdx;
			bowhistograms.at<float>(i,matchword) = bowhistograms.at<float>(i,matchword) + 1;
			//std::cout<<allmatches[i][j].queryIdx<<" "<<allmatches[i][j].trainIdx<<" "<<allmatches[i][j].imgIdx<<std::endl;
		}
		bowhistograms.row(i) = bowhistograms.row(i) / allmatches[i].size();
	}


//	for(int i=0;i<bowhistograms.rows;i++){
//		for(int j=0;j<bowhistograms.cols;j++){
//			std::cout<<bowhistograms.at<float>(i,j)<<" ";
//		}
//		std::cout<<std::endl;
//		std::cout<<"------"<<std::endl;
//	}

	return bowhistograms;
}


int FeatureExtractor::writeSPFH(std::string filename, std::vector<std::vector<float> > features)
{
	std::ofstream out;
	out.open(filename.c_str());
	int linesize = features[0].size();
	out<<linesize<<"\n";
	out<<features.size()<<"\n";
	for(int i=0;i<features.size();i++)
	{
		for(int j=0;j<features[i].size();j++){
			if(j != features[i].size()-1)
				out<<features[i][j]<<" ";
			else
				out<<features[i][j];
		}
		if(i != features.size()-1)
			out<<"\n";

	}
	out.close();
	return 1;
}



int FeatureExtractor:: ExtractCOSPAIR(std::string pointcloud_list, double R, int levels, int bins, int rgb_type, int rgb_bins, int keypointType, float sampling) // FOR RGB-D
{
	COSPAIR cospair;

	std::ifstream file;
	file.open(pointcloud_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::size_t found = filename.find("pcd");
		if (found!=std::string::npos){
			std::cout<<"Extracting: "<<filename<<std::endl;

			std::string path,pathlessname;
			unsigned found = filename.find_last_of("/");
			//unsigned found2 = filename.find_first_of(".");
			path = filename.substr(0,found);
			pathlessname = filename.substr(found+1);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

			try{
				pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename.c_str(), *cloud);
			}
			catch(pcl::PCLException& e)
			{
				std::cout<<"error reading!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;

			}


			std::vector<std::vector<float> >SPFH_features;
			SPFH_features = cospair.ComputeCOSPAIR(cloud,R, levels, bins, rgb_type, rgb_bins, keypointType, sampling);

			//std::cout<<"feature size: "<<SPFH_features.size()<<" "<<SPFH_features[0].size()<<std::endl;

			std::stringstream ss_savefilename;
			//ss_savefilename << pathlessname <<"_FSPFH";
			//std::cout<<"keypoint type: "<<keypointType<<std::endl;
			ss_savefilename << pathlessname <<"_SPAIR_sp"<<R*100<<"_l"<<levels<<"_b"<<bins;
			if(keypointType == 1)
				ss_savefilename <<"_h3d";
			else if(keypointType == 2)
				ss_savefilename <<"_iss";
			else if(keypointType == 0 && sampling != 0.0)
				ss_savefilename <<"_ds"<<sampling*100;
			if(rgb_type!=0)
				ss_savefilename <<"_RGB"<<rgb_type<<"_b"<<rgb_bins;
			//if(rgb_type==1)
			//	ss_savefilename <<"_RGB1"<<"_b"<<rgb_bins;
			//if(rgb_type==2)
			//	ss_savefilename <<"_RGB2"<<"_b"<<rgb_bins;
			//ss_savefilename <<"_v4";
			std::string savefilename = ss_savefilename.str();
			if(SPFH_features.size()>0)
				writeSPFH(savefilename, SPFH_features);
		}
	}

	return 1;
}

// FOR BigBird
//int FeatureExtractor:: ExtractCOSPAIR(std::string pointcloud_list, double R, int levels, int bins, int rgb_type, int rgb_bins, int keypointType, float sampling) // FOR BigBird
//{
//
//
//	std::ifstream file;
//	file.open(pointcloud_list.c_str());
//	std::string filename;
//	while(!file.eof())
//	{
//		file>>filename;
//		std::size_t found = filename.find("pcd");
//		std::size_t found2,found3;
//		if (found!=std::string::npos){
//			std::cout<<"Extracting: "<<filename<<std::endl;
//
//			std::string path,pathlessname,instance_name;
//			unsigned found = filename.find_last_of("/");
//			//unsigned found2 = filename.find_first_of(".");
//			path = filename.substr(0,found);
//			pathlessname = filename.substr(found+1);
//
//			found2 = filename.find("bigbird");
//			found3 = filename.find("clouds");
//			instance_name = filename.substr(found2+8,found3-found2-9);
//			pathlessname = instance_name + "_" + pathlessname;
//
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//			try{
//				pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename.c_str(), *cloud);
//			}
//			catch(pcl::PCLException& e)
//			{
//				std::cout<<"error reading!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
//
//			}
//
//
//			std::vector<std::vector<float> >SPFH_features;
//			SPFH_features = ComputeSPAIR_v4(cloud,R, levels, bins, rgb_type, rgb_bins, keypointType, sampling);
//
//			//std::cout<<"feature size: "<<SPFH_features.size()<<" "<<SPFH_features[0].size()<<std::endl;
//
//			std::stringstream ss_savefilename;
//			//ss_savefilename << pathlessname <<"_FSPFH";
//			//std::cout<<"keypoint type: "<<keypointType<<std::endl;
//			ss_savefilename << pathlessname <<"_SPAIR_sp"<<R*100<<"_l"<<levels<<"_b"<<bins;
//			if(keypointType == 1)
//				ss_savefilename <<"_h3d";
//			else if(keypointType == 2)
//				ss_savefilename <<"_iss";
//			else if(keypointType == 0 && sampling != 0.0)
//				ss_savefilename <<"_ds"<<sampling*100;
//			if(rgb_type!=0)
//				ss_savefilename <<"_RGB"<<rgb_type<<"_b"<<rgb_bins;
//			//if(rgb_type==1)
//			//	ss_savefilename <<"_RGB1"<<"_b"<<rgb_bins;
//			//if(rgb_type==2)
//			//	ss_savefilename <<"_RGB2"<<"_b"<<rgb_bins;
//			ss_savefilename <<"_v4";
//			std::string savefilename = ss_savefilename.str();
//			if(SPFH_features.size()>0)
//				writeSPFH(savefilename, SPFH_features);
//		}
//	}
//
//	return 1;
//}

// FOR Amazon
//int FeatureExtractor:: ExtractCOSPAIR(std::string pointcloud_list, double R, int levels, int bins, int rgb_type, int rgb_bins, int keypointType, float sampling)
//{
//
//
//	std::ifstream file;
//	file.open(pointcloud_list.c_str());
//	std::string filename;
//	while(!file.eof())
//	{
//		file>>filename;
//		std::size_t found = filename.find("pcd");
//		std::size_t found2,found3;
//		if (found!=std::string::npos){
//			std::cout<<"Extracting: "<<filename<<std::endl;
//
//			std::string path,pathlessname,instance_name;
//			unsigned found = filename.find_last_of("/");
//			//unsigned found2 = filename.find_first_of(".");
//			path = filename.substr(0,found);
//			pathlessname = filename.substr(found+1);
//
//			found2 = filename.find("amazon");
//			found3 = filename.find("clouds");
//			instance_name = filename.substr(found2+7,found3-found2-8);
//			pathlessname = instance_name + "_" + pathlessname;
//
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//			try{
//				pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename.c_str(), *cloud);
//			}
//			catch(pcl::PCLException& e)
//			{
//				std::cout<<"error reading!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
//
//			}
//
//
//			std::vector<std::vector<float> >SPFH_features;
//			SPFH_features = ComputeSPAIR_v4(cloud,R, levels, bins, rgb_type, rgb_bins, keypointType, sampling);
//
//			//std::cout<<"feature size: "<<SPFH_features.size()<<" "<<SPFH_features[0].size()<<std::endl;
//
//			std::stringstream ss_savefilename;
//			//ss_savefilename << pathlessname <<"_FSPFH";
//			//std::cout<<"keypoint type: "<<keypointType<<std::endl;
//			ss_savefilename << pathlessname <<"_SPAIR_sp"<<R*100<<"_l"<<levels<<"_b"<<bins;
//			if(keypointType == 1)
//				ss_savefilename <<"_h3d";
//			else if(keypointType == 2)
//				ss_savefilename <<"_iss";
//			else if(keypointType == 0 && sampling != 0.0)
//				ss_savefilename <<"_ds"<<sampling*100;
//			if(rgb_type!=0)
//				ss_savefilename <<"_RGB"<<rgb_type<<"_b"<<rgb_bins;
//			//if(rgb_type==1)
//			//	ss_savefilename <<"_RGB1"<<"_b"<<rgb_bins;
//			//if(rgb_type==2)
//			//	ss_savefilename <<"_RGB2"<<"_b"<<rgb_bins;
//			ss_savefilename <<"_v4";
//			std::string savefilename = ss_savefilename.str();
//			if(SPFH_features.size()>0)
//				writeSPFH(savefilename, SPFH_features);
//		}
//	}
//
//	return 1;
//}




double FeatureExtractor::computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size(); i++)
  {
    if (pcl::isFinite(cloud->points[i]) == false)
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

double FeatureExtractor::computeCloudSize (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	double cloudsize;
	long int numofpoints = cloud->size();
	std::vector<int> indices;//(numofpoints);
	std::vector<float> sqr_distances;//(numofpoints);
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud(cloud);
	double maxsize = 0;
	double localmax;

	for (size_t i = 0; i < cloud->size(); i++)
	{
		if (pcl::isFinite(cloud->points[i]) == true)
		{

			//tree.radiusSearch(i,0.02,indices,sqr_distances);
			tree.nearestKSearch (i, numofpoints, indices, sqr_distances);
			std::vector<float>::const_iterator it;
			it = std::max_element(sqr_distances.begin(), sqr_distances.end());
			//it = sqr_distances.end();
			localmax = sqrt(*it);
			//std::cout << " the max is " << localmax << std::endl;
			if(localmax > maxsize){
				maxsize = localmax;
			}
		}
	}
	cloudsize = maxsize;
	return cloudsize;
}

//std::vector<std::vector<double> > FeatureExtractor::ComputeOPFH(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double R, int levels, int bins, bool rgb, float sampling)
//{
//	std::vector<std::vector<double> > OPFH_features;
//
//
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//	ne.setInputCloud (cloud);
//	ne.setSearchMethod (tree);
//	ne.setRadiusSearch (0.01);
//	ne.compute (*cloud_normals);
//
//	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//	pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal> harris;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<int> sample_indices ;
//	pcl::UniformSampling<pcl::PointXYZRGB> sampler;
//	pcl::PointIndicesConstPtr indices ( new pcl::PointIndices);
//
//	if (sampling == 0)
//	{
//		harris.setInputCloud(cloud);
//		harris.setNormals(cloud_normals);
//		harris.setNonMaxSupression(true);
//		//harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal>::LOWE);
//		harris.setRadius(harris_radius);
//		harris.setRefine(false);
//		harris.compute(*keypoints);
//
//		indices = harris.getKeypointsIndices();
//	}
//	else
//	{
//		sampler.setInputCloud(cloud);
//		sampler.setRadiusSearch(sampling);
//		sampler.compute(sample_indices);
//		pcl::copyPointCloud(*cloud, sample_indices.points, *sampled_cloud);
//	}
//
//
//
//
//	//std::map<int, int>  pointMap;
//
//	int histsize = levels * bins * 3;
//	int levelsize = bins * 3;
//
//	for(int j=0;j<keypoints->size();j++)
//	{
//		std::map<std::pair<int,int>, double>  pairMap;
//		double paircount = 0;
//		std::vector<double> opfh_features(histsize,0.0);
//		if(pcl::isFinite(cloud_normals->points[indices->indices[j]]) == true)
//		{
//			std::vector<int> pointIdxRadiusSearch;
//			std::vector<float> pointRadiusSquaredDistance;
//			float f1,f2,f3,f4;
//			float deg_f1,deg_f2,deg_f3;
//			int bin_f1,bin_f2,bin_f3;
//
//			for(int l=1;l<=levels;l++)
//			{
//				std::cout<<" in level "<<l<<std::endl;
//				double r;
//				int searchsize;
//				double levelpaircount = 0;
//				r = ((l*1.0)/levels)*R;
//				searchsize = tree->radiusSearch (*cloud, indices->indices[j], r, pointIdxRadiusSearch, pointRadiusSquaredDistance,0);
//
//				std::pair<int, int> key;
//				// Iterate over all the points in the neighborhood
//				for (int i_idx = 0; i_idx < searchsize; ++i_idx)
//				{
//					for (int j_idx = 0; j_idx < i_idx; ++j_idx)
//					{
//						// If the 3D points are invalid, don't bother estimating, just continue
//						if (!isFinite (cloud->points[pointIdxRadiusSearch[i_idx]]) || !isFinite (cloud->points[pointIdxRadiusSearch[j_idx]]))
//							continue;
//
//						int p1, p2;
//						p1 = pointIdxRadiusSearch[i_idx];
//						p2 = pointIdxRadiusSearch[j_idx];
//						key = std::pair<int, int> (p1, p2);
//						std::map<std::pair<int,int>, double>::iterator  pairMap_it;
//						//std::cout<<i_idx<<" "<<j_idx<<" "<<p1<<" "<<p2<<" "<<pointRadiusSquaredDistance[i_idx]<<" "<<pointRadiusSquaredDistance[j_idx]<<std::endl;
//						pairMap_it = pairMap.find(key);
//						if(pairMap_it == pairMap.end())
//						{
//							paircount = paircount +1;
//							levelpaircount = levelpaircount +1;
//							pairMap.insert(pairMap_it,std::pair<std::pair<int,int>, double> (key,paircount));
//							pcl::computePairFeatures(cloud->points[pointIdxRadiusSearch[i_idx] ].getVector4fMap(),cloud_normals->points[pointIdxRadiusSearch[i_idx] ].getNormalVector4fMap(),cloud->points[pointIdxRadiusSearch[j_idx] ].getVector4fMap(),cloud_normals->points[pointIdxRadiusSearch[j_idx] ].getNormalVector4fMap(),f1,f2,f3,f4);
//							deg_f1 = pcl::rad2deg(f1+1);
//							deg_f2 = pcl::rad2deg(acos(f2)) ;
//							deg_f3 = pcl::rad2deg(acos(f3));
//							bin_f1 = int(float(deg_f1 / (360.0/bins)));
//							bin_f2 = int(float(deg_f2 / (180.0/bins)));
//							bin_f3 = int(float(deg_f3 / (180.0/bins)));
//							opfh_features[(levelsize*(l-1)) + bin_f1] = opfh_features[(levelsize*(l-1)) + bin_f1] + 1;
//							opfh_features[(levelsize*(l-1)) + 1*bins + bin_f2] = opfh_features[(levelsize*(l-1)) + 1*bins + bin_f2] + 1;
//							opfh_features[(levelsize*(l-1)) + 2*bins + bin_f3] = opfh_features[(levelsize*(l-1)) + 2*bins + bin_f3] + 1;
//							//std::cout<<f1<<" "<<f2<<" "<<f3<<" ; "<<deg_f1<<" "<<deg_f2<<" "<<deg_f3<<" ; "<<bin_f1<<" "<<bin_f2<<" "<<bin_f3<<std::endl;
//							//std::cout<<" in level "<<l<<std::endl;
//							//std::cout<<(levelsize*(l-1)) + bin_f1<<" "<<(levelsize*(l-1)) + 1*bins + bin_f2<<" "<<(levelsize*(l-1)) + 2*bins + bin_f3<<std::endl;
//						}
//
//					}
//				}
//				std::cout<<"# of pairs in level "<<l<<" is: "<<levelpaircount<< std::endl;
//				for(int n=0;n<opfh_features.size();n++)
//					opfh_features[n] = opfh_features[n] / levelpaircount;
//			}
//		}
//		std::cout<<"# of pairs in all levels is: "<<paircount<< std::endl;
//		OPFH_features.push_back(opfh_features);
//		pairMap.clear();
//	}
//
//
//	return OPFH_features;
//}

//void pp_callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
//{
//  if (event.getPointIndex () == -1)
//    return;
//  std::cout <<"Picked point index: "<< event.getPointIndex() << std::endl;
//  pcl::PointXYZ picked_pt;
//  event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
//  std::cout <<"x: "<< picked_pt.x<<"; y: "<< picked_pt.y<<"; z: "<< picked_pt.z<< std::endl;
//
//}
//
//int FeatureExtractor::viewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints)
//{
//	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//
//
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorhandler (cloud,255,255,0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>  keypointscolorHandler (keypoints, 0,255,0);
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> samplecolorhandler (sampled_cloud,255,0,0);
//
//	pcl::visualization::PointCloudColorHandlerRGBField< pcl:: PointXYZRGB > color (cloud);
//
//
//	viewer.addPointCloud<pcl::PointXYZRGB> (cloud,color, "original point cloud ");
//	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 100, 0.02, "normals");
//
//
//	viewer.addPointCloud<pcl::PointXYZI> (keypoints,keypointscolorHandler, "keypoints");
//	//viewer.addPointCloud<pcl::PointXYZRGB> (sampled_cloud,samplecolorhandler, "sampled_points");
//	viewer.setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");
//	//viewer.setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sampled_points");
//	viewer.initCameraParameters ();
//
//	viewer.registerPointPickingCallback (pp_callback, (void*)&viewer);
//
//	while (!viewer.wasStopped ())
//	{
//		viewer.spinOnce ();
//
//	}
//
//	return 1;
//}
