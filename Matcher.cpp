/*
 * Matcher.cpp
 *
 *  Created on: Dec 7, 2013
 *      Author: berker
 */

#include "Matcher.h"

Matcher::Matcher() {
	// TODO Auto-generated constructor stub

}

Matcher::~Matcher() {
	// TODO Auto-generated destructor stub
}


std::vector<cv::DMatch> Matcher::FlannMatcher(cv::Mat &queryDescriptors, std::vector<cv::Mat>& trainDescriptors)
{
	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.add( trainDescriptors );
	matcher.match( queryDescriptors, matches );
	return matches;
}

std::vector<cv::DMatch> Matcher::FlannMatcher(cv::Mat &queryDescriptors, cv::Mat &trainDescriptors)
{
	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match( queryDescriptors, trainDescriptors, matches );
	return matches;
}

//----->FLANNMATCHER - original, old
//std::vector<std::vector<cv::DMatch> > Matcher::FlannMatcher(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat>& trainDescriptors)
//{
//
//	cv::FlannBasedMatcher matcher;
//	std::vector<std::vector<cv::DMatch> > allmatches;
//	matcher.add( trainDescriptors );
//	for(int i=0;i<queryDescriptors.size();i++){
//		std::vector<cv::DMatch> matches;
//		matcher.match( queryDescriptors[i], matches );
//		allmatches.push_back(matches);
//	}
//	return allmatches;
//}

//----->FLANNMATCHER - new
std::vector<std::vector<cv::DMatch> > Matcher::FlannMatcher(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat>& trainDescriptors)
{

	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2)); //çalışmıyor
	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::KMeansIndexParams>(64,11,cvflann::CENTERS_KMEANSPP,0.2)); // çalışıyor
	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::CompositeIndexParams>()); // çalışıyor
	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::KDTreeIndexParams>(2)); //çok iyi çalışıyor; 16 en iyi
	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::AutotunedIndexParams>()); //çalışmıyor

	//cv::flann::IndexParams index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5);
	//cv::flann::SearchParams search_params = dict(checks=50) ;
	//cv::Ptr<cv::flann::IndexParams> index_params = cv::Ptr<cv::flann::LshIndexParams>();
	//cv::Ptr<cv::flann::SearchParams> search_params = cv::Ptr<cv::flann::SearchParams>();
	//cv::FlannBasedMatcher matcher(index_params,search_params);


	cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::KDTreeIndexParams>(16), cv::makePtr<cv::flann::SearchParams>(64));//16-64


	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>();
	//cv::FlannBasedMatcher matcher;


	std::vector<std::vector<cv::DMatch> > allmatches;
	matcher->add( trainDescriptors );
	for(int i=0;i<queryDescriptors.size();i++){
		std::vector<cv::DMatch> matches;
		matcher->match( queryDescriptors[i], matches );
		allmatches.push_back(matches);
	}
	return allmatches;
}




//Only used for single descriptor-to-descriptor matching
std::vector<std::vector<cv::DMatch> > Matcher::BruteForceMatcherL2KNN(const cv::Mat &queryDescriptors, const cv::Mat& trainDescriptors, int k)
{
	cv::BFMatcher matcher(cv::NORM_L2);
	std::vector<std::vector<cv::DMatch> > allmatches;
	matcher.knnMatch(queryDescriptors,trainDescriptors, allmatches,k);
	return allmatches;
}

std::vector<std::vector<cv::DMatch> > Matcher::FlannMatcherBinary(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat>& trainDescriptors)
{
	 cv::Ptr<cv::FlannBasedMatcher> matcher;
	 cv::flann::Index flannIndex(trainDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
	// cv::flann::Index flannIndex(trainDescriptors, cv::flann::LshIndexParams(12, 20, 2));
	//cv::FlannBasedMatcher matcher(cv::flann::LshIndexParams(12, 20, 2));
	std::vector<std::vector<cv::DMatch> > allmatches;
	matcher->add( trainDescriptors );
	for(int i=0;i<queryDescriptors.size();i++){
		std::vector<cv::DMatch> matches;
		matcher->match( queryDescriptors[i], matches );
		allmatches.push_back(matches);
	}
	return allmatches;
}

std::vector<std::vector<cv::DMatch> > Matcher::BruteforceMatcherHamming(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat> &trainDescriptors)
{
	cv::BFMatcher matcher(cv::NORM_HAMMING2);
	std::vector<std::vector<cv::DMatch> > allmatches;
	matcher.add( trainDescriptors );
	for(int i=0;i<queryDescriptors.size();i++){
		std::vector<cv::DMatch> matches;
		matcher.match( queryDescriptors[i], matches );
		allmatches.push_back(matches);
	}
	return allmatches;

}

std::vector<std::vector<cv::DMatch> > Matcher::BruteforceMatcherL2(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat> &trainDescriptors)
{
	cv::BFMatcher matcher(cv::NORM_L2);
	std::vector<std::vector<cv::DMatch> > allmatches;
	matcher.add( trainDescriptors );
	for(int i=0;i<queryDescriptors.size();i++){
		std::vector<cv::DMatch> matches;
		matcher.match( queryDescriptors[i], matches );
		allmatches.push_back(matches);
	}
	return allmatches;

}

std::vector<std::vector<cv::DMatch> > Matcher::HistogramMatcher(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat> &trainDescriptors, int type, bool RGB)
{
	cv::BFMatcher matcher(cv::NORM_L2);
	std::vector<std::vector<cv::DMatch> > allmatches;
	for(int i=0;i<queryDescriptors.size();i++){
		std::vector<cv::DMatch> matches;
		matches = CompareHistogram( queryDescriptors[i], trainDescriptors, type, RGB);
		allmatches.push_back(matches);
	}
	return allmatches;

}
std::vector<cv::DMatch> Matcher::CompareHistogram(cv::Mat &queryDescriptor, std::vector<cv::Mat> &trainDescriptors, int type, bool RGB)
{
	std::vector<cv::DMatch> matches;


	for(uint q=0;q<queryDescriptor.rows;q++){
		cv::DMatch dmatch;
		double dist;

		double max = -1000000;
		double min =  1000000;
		uint i_max = 0;
		uint j_max = 0;
		uint i_min = 0;
		uint j_min = 0;
		for(uint i=0; i<trainDescriptors.size();i++ ){
			for(uint j=0;j<trainDescriptors[i].rows;j++){

//				dist = cv::compareHist(queryDescriptor.row(q), trainDescriptors[i].row(j),type);
//				//std::cout<<"Histogram Matcher: "<<i<<" "<<j<<" "<<dist<<std::endl;
//				if(dist<min){
//					min=dist;
//					i_min = i;
//					j_min = j;
//				}
//				if(dist>max){
//					max=dist;
//					i_max = i;
//					j_max = j;
//				}

				//std::cout<<queryDescriptor.row(q)<<std::endl;
				//std::cout<<trainDescriptors[i].row(j)<<std::endl;

				double total_dist = 0;
				int level;
				if (RGB==0)
					level = 7;
				else
					level = 14;
				for(int l=1;l<=level;l++){
					for(int f=0;f<3;f++){
						cv::Mat query = queryDescriptor(cv::Rect (((l-1)*27 + f*9),q, 9, 1));
						cv::Mat train = trainDescriptors[i](cv::Rect (((l-1)*27 + f*9),j, 9, 1));
						//std::cout<<"QD: "<<query.rows<<" "<<query.cols<<std::endl;
						//std::cout<<"TD: "<<train.rows<<" "<<train.cols<<std::endl;
						//std::cout<<query<<std::endl;
						//std::cout<<train<<std::endl;
						dist = cv::compareHist(query, train,type);
						total_dist = total_dist + dist;
						//std::cout<<l<<" "<<f<<" ; "<<"dist: "<<dist<< " ; total dist: "<<total_dist<<std::endl;
					}
				}
				if(total_dist<min){
					min=total_dist;
					i_min = i;
					j_min = j;
				}
				if(total_dist>max){
					max=total_dist;
					i_max = i;
					j_max = j;
				}



			}
		}
		if(type == 0 || type == 2)
		{
			dmatch.distance = max;
			dmatch.queryIdx = q;
			dmatch.trainIdx = j_max;
			dmatch.imgIdx = i_max;

		}
		else if (type == 1 || type ==3)
		{
			dmatch.distance = min;
			dmatch.queryIdx = q;
			dmatch.trainIdx = j_min;
			dmatch.imgIdx = i_min;
		}
		matches.push_back(dmatch);
	}

	return matches;
}
