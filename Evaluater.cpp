/*
 * Evaluater.cpp
 *
 *  Created on: Dec 9, 2013
 *      Author: berker
 */

#include "Evaluater.h"
#include "Reader.h"
#include "Matcher.h"
#include "FeatureExtractor.h"

using namespace cv;
using namespace cv::ml;

struct sort_pair {
    bool operator()(const std::pair<int,float> &left, const std::pair<int,float> &right) {
        return left.second < right.second;
    }
};



Evaluater::Evaluater() {
	// TODO Auto-generated constructor stub
	Temp_Test = "temp_test.txt";
	Temp_Train = "temp_train.txt";
	//numberofCategories = 51;
	//numberofInstances = 300;

	//std::vector< std::vector<int> > categoryScores(52, std::vector<int>(52, 0));
	//std::vector< std::vector<int> > instanceScores(301, std::vector<int>(301, 0));


}

Evaluater::~Evaluater() {
	// TODO Auto-generated destructor stub
}

//struct PCDEval
//{
//	std::string fileName;
//	int categoryNum;
//	std::string categoryName;
//	int instanceNum;
//	std::string instanceName;
//};

float Evaluater::getCategoryAccuracy()
{
	return categoryAccuracy;
}

float Evaluater::getInstanceAccuracy()
{
	return instanceAccuracy;
}

std::vector< std::vector<int> > Evaluater::getCategoryScores()
{
	return categoryScores;
}

std::vector< std::vector<int> > Evaluater::getInstanceScores()
{
	return instanceScores;
}

std::vector< std::pair<int,int> > Evaluater::getCategoryPairs()
{
	return categoryPairs;
}

std::vector< std::pair<int,int> > Evaluater::getInstancePairs()
{
	return instancePairs;
}

int Evaluater::EvaluateVFH_flann(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;


	Reader reader2;
	std::vector<cv::Mat> cvVFHtest;
	reader2.ReadVFH(Temp_Test,cvVFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvVFHtrain;
	reader3.ReadVFH(Temp_Train,cvVFHtrain);

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;


	matches = matcher.FlannMatcher(cvVFHtest,cvVFHtrain);


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;

	//std::vector< std::vector<int> > categoryScores(52, std::vector<int>(52, 0));
	//std::vector< std::vector<int> > instanceScores(301, std::vector<int>(301, 0));


	for(int i=0;i<matches.size();i++){
		imgID = matches[i][0].imgIdx;
		if(imgID < trainEvalList.size())
		{

//			std::cout<<i<<std::endl;
//			std::cout<<imgID<<std::endl;
//			std::cout<<testEvalList[i].categoryNum<<std::endl;
//			std::cout<<trainEvalList[imgID].categoryNum<<std::endl;
//			std::cout<<testEvalList[i].instanceNum<<std::endl;
//			std::cout<<trainEvalList[imgID].instanceNum<<std::endl;
//			std::cout<<std::endl;

			//categoryScores[testEvalList[i].categoryNum][trainEvalList[imgID].categoryNum]++;
			//instanceScores[testEvalList[i].instanceNum][trainEvalList[imgID].instanceNum]++;

			if(testEvalList[i].categoryNum == trainEvalList[imgID].categoryNum)
			{
				categoryMatch++;
			}
			else
				categoryNonMatch++;

			if(testEvalList[i].instanceNum == trainEvalList[imgID].instanceNum)
			{
				instanceMatch++;
			}
			else
				instanceNonMatch++;

		}
	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluateVFH_bf(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;


	Reader reader2;
	std::vector<cv::Mat> cvVFHtest;
	reader2.ReadVFH(Temp_Test,cvVFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvVFHtrain;
	reader3.ReadVFH(Temp_Train,cvVFHtrain);

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;


	matches = matcher.BruteforceMatcherL2(cvVFHtest,cvVFHtrain);


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;

	//std::vector< std::vector<int> > categoryScores(52, std::vector<int>(52, 0));
	//std::vector< std::vector<int> > instanceScores(301, std::vector<int>(301, 0));


	for(int i=0;i<matches.size();i++){
		imgID = matches[i][0].imgIdx;
		if(imgID < trainEvalList.size())
		{

			//categoryScores[testEvalList[i].categoryNum][trainEvalList[imgID].categoryNum]++;
			//instanceScores[testEvalList[i].instanceNum][trainEvalList[imgID].instanceNum]++;

			if(testEvalList[i].categoryNum == trainEvalList[imgID].categoryNum)
			{
				categoryMatch++;
			}
			else
				categoryNonMatch++;

			if(testEvalList[i].instanceNum == trainEvalList[imgID].instanceNum)
			{
				instanceMatch++;
			}
			else
				instanceNonMatch++;

		}
	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}


int Evaluater::EvaluateESF_flann(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;


	Reader reader2;
	std::vector<cv::Mat> cvESFtest;
	reader2.ReadESF(Temp_Test,cvESFtest);

	Reader reader3;
	std::vector<cv::Mat> cvESFtrain;
	reader3.ReadESF(Temp_Train,cvESFtrain);

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;


	matches = matcher.FlannMatcher(cvESFtest,cvESFtrain);


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;

	//std::vector< std::vector<int> > categoryScores(52, std::vector<int>(52, 0));
	//std::vector< std::vector<int> > instanceScores(301, std::vector<int>(301, 0));


	for(int i=0;i<matches.size();i++){
		imgID = matches[i][0].imgIdx;
		//std::cout<<i<<" "<<matches[i][0].queryIdx<<" "<<matches[i][0].trainIdx<<" "<<matches[i][0].imgIdx<<std::endl;
		if(imgID < trainEvalList.size())
		{

//			std::cout<<i<<std::endl;
//			std::cout<<imgID<<std::endl;
//			std::cout<<testEvalList[i].categoryNum<<std::endl;
//			std::cout<<trainEvalList[imgID].categoryNum<<std::endl;
//			std::cout<<testEvalList[i].instanceNum<<std::endl;
//			std::cout<<trainEvalList[imgID].instanceNum<<std::endl;
//			std::cout<<std::endl;

			//categoryScores[testEvalList[i].categoryNum][trainEvalList[imgID].categoryNum]++;
			//instanceScores[testEvalList[i].instanceNum][trainEvalList[imgID].instanceNum]++;

			if(testEvalList[i].categoryNum == trainEvalList[imgID].categoryNum)
			{
				categoryMatch++;
			}
			else
				categoryNonMatch++;

			if(testEvalList[i].instanceNum == trainEvalList[imgID].instanceNum)
			{
				instanceMatch++;
			}
			else
				instanceNonMatch++;

		}
	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluateESF_bf(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;


	Reader reader2;
	std::vector<cv::Mat> cvESFtest;
	reader2.ReadESF(Temp_Test,cvESFtest);

	Reader reader3;
	std::vector<cv::Mat> cvESFtrain;
	reader3.ReadESF(Temp_Train,cvESFtrain);

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;


	matches = matcher.BruteforceMatcherL2(cvESFtest,cvESFtrain);


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;

	//std::vector< std::vector<int> > categoryScores(52, std::vector<int>(52, 0));
	//std::vector< std::vector<int> > instanceScores(301, std::vector<int>(301, 0));


	for(int i=0;i<matches.size();i++){
		imgID = matches[i][0].imgIdx;
		//std::cout<<i<<" "<<matches[i][0].queryIdx<<" "<<matches[i][0].trainIdx<<" "<<matches[i][0].imgIdx<<std::endl;
		if(imgID < trainEvalList.size())
		{

			//categoryScores[testEvalList[i].categoryNum][trainEvalList[imgID].categoryNum]++;
			//instanceScores[testEvalList[i].instanceNum][trainEvalList[imgID].instanceNum]++;

			if(testEvalList[i].categoryNum == trainEvalList[imgID].categoryNum)
			{
				categoryMatch++;
			}
			else
				categoryNonMatch++;

			if(testEvalList[i].instanceNum == trainEvalList[imgID].instanceNum)
			{
				instanceMatch++;
			}
			else
				instanceNonMatch++;

		}
	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}


int Evaluater::EvaluateFPFH_flann(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvFPFHtest;
	reader2.ReadFPFH(Temp_Test,cvFPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvFPFHtrain;
	reader3.ReadFPFH(Temp_Train,cvFPFHtrain);

	std::cout<<"# of test files: "<<cvFPFHtest.size()<<"; # of train files: "<<cvFPFHtrain.size()<<std::endl;

	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	matches = matcher.FlannMatcher(cvFPFHtest,cvFPFHtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvFPFHtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;

	//std::vector< std::vector<int> > categoryScores(52, std::vector<int>(52, 0));
	//std::vector< std::vector<int> > instanceScores(301, std::vector<int>(301, 0));

//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluateFPFH_bf(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvFPFHtest;
	reader2.ReadFPFH(Temp_Test,cvFPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvFPFHtrain;
	reader3.ReadFPFH(Temp_Train,cvFPFHtrain);

	std::cout<<"# of test files: "<<cvFPFHtest.size()<<"; # of train files: "<<cvFPFHtrain.size()<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;


	//matches = matcher.FlannMatcher(cvFPFHtest,cvFPFHtrain);
	std::time_t start_match = std::time(NULL);
	matches = matcher.BruteforceMatcherL2(cvFPFHtest,cvFPFHtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvFPFHtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;

	//std::vector< std::vector<int> > categoryScores(52, std::vector<int>(52, 0));
	//std::vector< std::vector<int> > instanceScores(301, std::vector<int>(301, 0));

//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;

		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluateSHOT_flann(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvSHOTtest;
	reader2.ReadSHOT(Temp_Test,cvSHOTtest);

	Reader reader3;
	std::vector<cv::Mat> cvSHOTtrain;
	reader3.ReadSHOT(Temp_Train,cvSHOTtrain);

	std::cout<<"# of test files: "<<cvSHOTtest.size()<<"; # of train files: "<<cvSHOTtrain.size()<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	matches = matcher.FlannMatcher(cvSHOTtest,cvSHOTtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvSHOTtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();



		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}

	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);






	return 1;
}

int Evaluater::EvaluateSHOT_bf(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvSHOTtest;
	reader2.ReadSHOT(Temp_Test,cvSHOTtest);

	Reader reader3;
	std::vector<cv::Mat> cvSHOTtrain;
	reader3.ReadSHOT(Temp_Train,cvSHOTtrain);

	std::cout<<"# of test files: "<<cvSHOTtest.size()<<"; # of train files: "<<cvSHOTtrain.size()<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	//matches = matcher.FlannMatcher(cvSHOTtest,cvSHOTtrain);
	matches = matcher.BruteforceMatcherL2(cvSHOTtest,cvSHOTtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvSHOTtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;

		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluateSHOTCOLOR_bf(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvSHOTtest;
	reader2.ReadSHOTColor(Temp_Test,cvSHOTtest);

	Reader reader3;
	std::vector<cv::Mat> cvSHOTtrain;
	reader3.ReadSHOTColor(Temp_Train,cvSHOTtrain);

	std::cout<<"# of test files: "<<cvSHOTtest.size()<<"; # of train files: "<<cvSHOTtrain.size()<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	//matches = matcher.FlannMatcher(cvSHOTtest,cvSHOTtrain);
	matches = matcher.BruteforceMatcherL2(cvSHOTtest,cvSHOTtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvSHOTtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;

		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluateSHOTCOLOR_flann(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvSHOTtest;
	reader2.ReadSHOTColor(Temp_Test,cvSHOTtest);

	Reader reader3;
	std::vector<cv::Mat> cvSHOTtrain;
	reader3.ReadSHOTColor(Temp_Train,cvSHOTtrain);

	std::cout<<"# of test files: "<<cvSHOTtest.size()<<"; # of train files: "<<cvSHOTtrain.size()<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	matches = matcher.FlannMatcher(cvSHOTtest,cvSHOTtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvSHOTtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;

		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluatePFH_flann(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvPFHtest;
	reader2.ReadPFH(Temp_Test,cvPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvPFHtrain;
	reader3.ReadPFH(Temp_Train,cvPFHtrain);

	std::cout<<"# of test files: "<<cvPFHtest.size()<<"; # of train files: "<<cvPFHtrain.size()<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	matches = matcher.FlannMatcher(cvPFHtest,cvPFHtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvPFHtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluatePFH_bf(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvPFHtest;
	reader2.ReadPFH(Temp_Test,cvPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvPFHtrain;
	reader3.ReadPFH(Temp_Train,cvPFHtrain);

	std::cout<<"# of test files: "<<cvPFHtest.size()<<"; # of train files: "<<cvPFHtrain.size()<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	matches = matcher.BruteforceMatcherL2(cvPFHtest,cvPFHtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvPFHtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluatePFHRGB_flann(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvPFHtest;
	reader2.ReadPFHRGB(Temp_Test,cvPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvPFHtrain;
	reader3.ReadPFHRGB(Temp_Train,cvPFHtrain);

	std::cout<<"# of test files: "<<cvPFHtest.size()<<"; # of train files: "<<cvPFHtrain.size()<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	matches = matcher.FlannMatcher(cvPFHtest,cvPFHtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvPFHtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluatePFHRGB_bf(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvPFHtest;
	reader2.ReadPFHRGB(Temp_Test,cvPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvPFHtrain;
	reader3.ReadPFHRGB(Temp_Train,cvPFHtrain);

	std::cout<<"# of test files: "<<cvPFHtest.size()<<"; # of train files: "<<cvPFHtrain.size()<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;


	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	matches = matcher.BruteforceMatcherL2(cvPFHtest,cvPFHtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvPFHtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}


int Evaluater::EvaluateVFH_SVM(std::string testList, std::string trainList,int kernelType, bool evalType)
{

	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;
	testEvalList = reader.ReadTestEvalTypes(testList);
	trainEvalList = reader.ReadTrainEvalTypes(trainList);

	Reader reader2;
	cv::Mat cvVFHtest;
	reader2.ReadVFH(Temp_Test,cvVFHtest);
	cv::normalize(cvVFHtest,cvVFHtest,0,1,cv::NORM_MINMAX,CV_32F);

	Reader reader3;
	cv::Mat cvVFHtrain;
	reader3.ReadVFH(Temp_Train,cvVFHtrain);
	cv::normalize(cvVFHtrain,cvVFHtrain,0,1,cv::NORM_MINMAX,CV_32F);


	int testsize = cvVFHtest.rows;
	int trainsize = cvVFHtrain.rows;

	std::cout<<"EVAL Test size: "<<testEvalList.size()<<" ; EVAL Train size: "<<trainEvalList.size()<<std::endl;
	std::cout<<"Test size: "<<testsize<<" ; Train size: "<<trainsize<<std::endl;

	cv::Mat testLabels(testsize,1,CV_32FC1);
	cv::Mat trainLabels(trainsize,1,CV_32FC1);


	for(int i=0;i<trainsize;i++)
	{
		if(trainEvalList[i].categoryNum == 2)
			trainLabels.at<float>(i) = 1;
		else
			trainLabels.at<float>(i) = 0;
	}
	for(int i=0;i<testsize;i++)
	{
		if(testEvalList[i].categoryNum == 2)
			testLabels.at<float>(i) = 1;
		else
			testLabels.at<float>(i) = 0;
	}

//	int label = 1;
//	for(int i=0;i<testsize;i++)
//	{
//		if(i==0){
//			testLabels.at<float>(i) = label;
//		}
//		else{
//			if(evalType == 0){
//				if(testEvalList[i].categoryNum == testEvalList[i-1].categoryNum)
//				{}
//				else
//					label++;
//			}
//			else{
//				if(testEvalList[i].instanceNum == testEvalList[i-1].instanceNum)
//				{}
//				else
//					label++;
//
//			}
//			testLabels.at<float>(i) = label;
//		}
//		//std::cout<<"Label: "<<label<<std::endl;
//	}
//
//	label = 1;
//	for(int i=0;i<trainsize;i++)
//	{
//		if(i==0){
//			trainLabels.at<float>(i) = label;
//		}
//		else{
//			if(evalType == 0){
//				if(trainEvalList[i].categoryNum == trainEvalList[i-1].categoryNum)
//				{}
//				else
//					label++;
//			}
//			else{
//				if(trainEvalList[i].instanceNum == trainEvalList[i-1].instanceNum)
//				{}
//				else
//					label++;
//
//			}
//			trainLabels.at<float>(i) = label;
//		}
//	}


	Ptr<SVM> svm = SVM::create();
	if(kernelType == 0)
		svm->setKernel(SVM::LINEAR);
	else if(kernelType == 1)
		svm->setKernel(SVM::POLY);
	else if (kernelType == 2)
		svm->setKernel(SVM::RBF);
	else if (kernelType == 3)
		svm->setKernel(SVM::SIGMOID);
	else
		svm->setKernel(SVM::RBF);

	svm->setType(SVM::C_SVC);
	svm->setNu(0.5);
	svm->setC(0.1);
	svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 1000, 1e-6));
	//params.term_crit = cv::TermCriteria(CV_TERMCRIT_ITER, 1000, 1e-6);

    //------------------------ Train the SVM ----------------------------------------------------
    std::cout << "Starting training process" << std::endl;
    //CvSVM svm;
    //svm.train(cvVFHtrain, trainLabels, cv::Mat(), cv::Mat(), params);
    Ptr<TrainData> trainData = TrainData::create(cvVFHtrain, ROW_SAMPLE, trainLabels);
    svm->train(trainData,0);
    std::cout << "Finished training process" << std::endl;

    //-------------------------TEST----------------------------------------------------------


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;

    cv::Mat results;
    svm->predict(cvVFHtest,results);
    std::cout<<"Results size: "<<results.rows<<std::endl;
    for(int i=0;i<results.rows;i++){
    	float result = results.at<float>(i);
    	std::cout<<result<<" ";
    	if(evalType == 0){
			//if(testEvalList[i].categoryNum == result)
			if(testLabels.at<float>(i) == result)
			{
				categoryMatch++;
			}
			else
				categoryNonMatch++;
    	}
    	else{
			if(testEvalList[i].instanceNum == result)
			{
				instanceMatch++;
			}
			else
				instanceNonMatch++;
    	}
    }

	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);

    std::cout<<"Match #:"<<categoryMatch<<" ; Nonmatch: "<<categoryNonMatch<<std::endl;
	return 1;
}

int Evaluater::EvaluateBOW_flann(std::string vocabulary_name, int feature_type, std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	//Reading features
	std::vector<cv::Mat> cvFPFHtest;
	std::vector<cv::Mat> cvFPFHtrain;
	if(feature_type == 1)
	{
		Reader reader2;
		reader2.ReadFPFH(Temp_Test,cvFPFHtest);

		Reader reader3;
		reader3.ReadFPFH(Temp_Train,cvFPFHtrain);

		std::cout<<"# of FPFH test files: "<<cvFPFHtest.size()<<"; # of FPFH train files: "<<cvFPFHtrain.size()<<std::endl;
	}


	//BOW Histogram Extraction
	cv::Mat TestBOWHistograms;
	cv::Mat TrainBOWHistograms;
	FeatureExtractor fe;

	if(feature_type == 1)
	{
		TestBOWHistograms = fe.ExtractBOWHistograms(vocabulary_name, cvFPFHtest);
		TrainBOWHistograms = fe.ExtractBOWHistograms(vocabulary_name, cvFPFHtrain);
	}


	//matching
	Matcher matcher;
	std::vector<cv::DMatch> matches;
	matches = matcher.FlannMatcher(TestBOWHistograms,TrainBOWHistograms);


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;

	int imgID;
	for(int i=0;i<matches.size();i++){
		imgID = matches[i].trainIdx;
		std::cout<<i<<" "<<matches[i].queryIdx<<" "<<matches[i].trainIdx<<" "<<matches[i].imgIdx<<std::endl;
		//std::cout<<testEvalList[i].categoryNum<<" "<<trainEvalList[imgID].categoryNum<<" "<<testEvalList[i].instanceNum<<" "<<trainEvalList[imgID].instanceNum<<std::endl;
		if(testEvalList[i].categoryNum == trainEvalList[imgID].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[imgID].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;
	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);

	return 1;
}

int Evaluater::EvaluateBOP_bf(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvBOPtest;
	reader2.ReadBOPList(Temp_Test,cvBOPtest);

	Reader reader3;
	std::vector<cv::Mat> cvBOPtrain;
	reader3.ReadBOPList(Temp_Train,cvBOPtrain);

	std::cout<<"# of test files: "<<cvBOPtest.size()<<"; # of train files: "<<cvBOPtrain.size()<<std::endl;

	std::cout<<cvBOPtest[0].type()<<" "<<cvBOPtest[0].cols<<" "<<cvBOPtrain[0].cols<<std::endl;
	std::cout<<cvBOPtest[0].type()<<" "<<cvBOPtest[0].rows<<" "<<cvBOPtrain[0].rows<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;


	//matches = matcher.BruteforceMatcherHamming(cvBOPtest,cvBOPtrain);
	matches = matcher.BruteforceMatcherL2(cvBOPtest,cvBOPtrain);
	//matches = matcher.FlannMatcher(cvBOPtest,cvBOPtrain);


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvBOPtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		std::cout<<"Query image: "<<i<<std::endl;
		std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluateBOP_hamming(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvBOPtest;
	reader2.ReadBOPListuchar(Temp_Test,cvBOPtest);

	Reader reader3;
	std::vector<cv::Mat> cvBOPtrain;
	reader3.ReadBOPListuchar(Temp_Train,cvBOPtrain);

	std::cout<<"# of test files: "<<cvBOPtest.size()<<"; # of train files: "<<cvBOPtrain.size()<<std::endl;

	std::cout<<cvBOPtest[0].type()<<" "<<cvBOPtest[0].cols<<" "<<cvBOPtrain[0].cols<<std::endl;
	std::cout<<cvBOPtest[0].type()<<" "<<cvBOPtest[0].rows<<" "<<cvBOPtrain[0].rows<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	matches = matcher.BruteforceMatcherHamming(cvBOPtest,cvBOPtrain);


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvBOPtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		std::cout<<"Query image: "<<i<<std::endl;
		std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}


int Evaluater::EvaluateBOP_flann(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvBOPtest;
	reader2.ReadBOPList(Temp_Test,cvBOPtest);

	Reader reader3;
	std::vector<cv::Mat> cvBOPtrain;
	reader3.ReadBOPList(Temp_Train,cvBOPtrain);

	std::cout<<"# of test files: "<<cvBOPtest.size()<<"; # of train files: "<<cvBOPtrain.size()<<std::endl;

	std::cout<<cvBOPtest[0].type()<<" "<<cvBOPtest[0].cols<<" "<<cvBOPtrain[0].cols<<std::endl;
	std::cout<<cvBOPtest[0].type()<<" "<<cvBOPtest[0].rows<<" "<<cvBOPtrain[0].rows<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;


	//matches = matcher.FlannMatcherBinary(cvBOPtest,cvBOPtrain);
	matches = matcher.FlannMatcher(cvBOPtest,cvBOPtrain);


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvBOPtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		std::cout<<"Query image: "<<i<<std::endl;
		std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}


int Evaluater::EvaluateSPFH_bf(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvSPFHtest;
	reader2.ReadSPFHList(Temp_Test,cvSPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvSPFHtrain;
	reader3.ReadSPFHList(Temp_Train,cvSPFHtrain);

	std::cout<<"# of test files: "<<cvSPFHtest.size()<<"; # of train files: "<<cvSPFHtrain.size()<<std::endl;

	std::cout<<cvSPFHtest[0].type()<<" "<<cvSPFHtest[0].cols<<" "<<cvSPFHtrain[0].cols<<std::endl;
	std::cout<<cvSPFHtest[0].type()<<" "<<cvSPFHtest[0].rows<<" "<<cvSPFHtrain[0].rows<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	//matches = matcher.BruteforceMatcherHamming(cvBOPtest,cvBOPtrain);
	matches = matcher.BruteforceMatcherL2(cvSPFHtest,cvSPFHtrain);
	//matches = matcher.FlannMatcher(cvBOPtest,cvBOPtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvSPFHtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;

		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluateSPFH_flann(std::string testList, std::string trainList)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvSPFHtest;
	reader2.ReadSPFHList(Temp_Test,cvSPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvSPFHtrain;
	reader3.ReadSPFHList(Temp_Train,cvSPFHtrain);

	std::cout<<"# of test files: "<<cvSPFHtest.size()<<"; # of train files: "<<cvSPFHtrain.size()<<std::endl;

	std::cout<<cvSPFHtest[0].type()<<" "<<cvSPFHtest[0].cols<<" "<<cvSPFHtrain[0].cols<<std::endl;
	std::cout<<cvSPFHtest[0].type()<<" "<<cvSPFHtest[0].rows<<" "<<cvSPFHtrain[0].rows<<std::endl;
	std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;

	std::time_t start_match = std::time(NULL);
	matches = matcher.FlannMatcher(cvSPFHtest,cvSPFHtrain);
	std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvSPFHtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;

		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();


		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}


int Evaluater::EvaluateSPFH_shrec_flann(std::string testList, int run_number)
{


	std::time_t start_read = std::time(NULL);

	Reader reader;

	std::vector<int> instance_numbers;
	reader.ReadShrecInstanceNumbers(testList, instance_numbers);


	std::vector<cv::Mat> cvSPFHtest;
	reader.ReadSPFHList(testList,cvSPFHtest);

	std::cout<<"instance numbers size: "<<instance_numbers.size()<<std::endl;
	std::cout<<"# of test scans: "<<cvSPFHtest.size()<<std::endl;
	int numberofobjects;
	numberofobjects = *std::max_element(instance_numbers.begin(),instance_numbers.end());
	std::cout<<"# of test objects: "<<numberofobjects<<std::endl;


	Matcher matcher;


	std::time_t start_match = std::time(NULL);


	float instance_distances[numberofobjects][numberofobjects];
	float max_distance = 5.5;

	for(int m=1;m<1+numberofobjects;m++)
	{

		std::vector<cv::Mat> cvQuery;
		for(int n=0;n<cvSPFHtest.size();n++)
		{
			if(instance_numbers[n]==m)
				cvQuery.push_back(cvSPFHtest[n]);

		}

		std::cout<<"Query size: "<<cvQuery.size()<<std::endl;

		for(int mm=1;mm<1+numberofobjects;mm++)
		{

			std::vector<cv::Mat> cvTest;
			for(int nn=0;nn<cvSPFHtest.size();nn++)
			{
				if(instance_numbers[nn] == mm)
					cvTest.push_back(cvSPFHtest[nn]);

			}
			std::cout<<"Test size: "<<cvTest.size()<<std::endl;

			std::vector<std::vector<cv::DMatch> > matches;
			matches = matcher.FlannMatcher(cvQuery,cvTest);

			float average_distance = 0;
			float sum=0;
			for(int ii=0;ii<matches.size();ii++){
				for(int jj=0;jj<matches[ii].size();jj++)
				{
					average_distance = average_distance + matches[ii][jj].distance;
					sum = sum + 1;
				}
			}
			average_distance = average_distance / sum;
			instance_distances[m-1][mm-1] = average_distance;
			if(average_distance > max_distance)
				max_distance = average_distance;

			std::cout<<" Query Object: "<<m<<"  ;  Test Object: "<<mm<<"  ;  Distance: "<<average_distance<<std::endl;
		}

		std::stringstream ss;
		ss<<"rankedlists_flann/"<<run_number<<"-"<<m<<".res";
		ofstream of;
		of.open(ss.str().c_str());

		std::vector< std::pair<int,float> > distance_map;
		for(int mm=1;mm<1+numberofobjects;mm++)
		{
			distance_map.push_back( std::pair<int,float>(mm,instance_distances[m-1][mm-1]) );
		}

		std::sort(distance_map.begin(),distance_map.end(),sort_pair());

		for(int mm=1;mm<1+numberofobjects;mm++)
		{
			std::cout<<distance_map[mm-1].first<<" "<<distance_map[mm-1].second<<" "<< 1 - ((distance_map[mm-1].second) / max_distance) << std::endl;
			of<<distance_map[mm-1].first<<" "<<distance_map[mm-1].second<<" "<< 1 - ((distance_map[mm-1].second) / max_distance) <<" "<< exp (-(2*((distance_map[mm-1].second) / max_distance))*(2*((distance_map[mm-1].second) / max_distance))) <<std::endl;
		}
		of.close();
	}



	std::cout<<"Max distance: "<<max_distance<<std::endl;
	std::cout<<"Total Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;



	return 1;
}

int Evaluater::EvaluateSPFH_shrec_bf(std::string testList, int run_number)
{


	std::time_t start_read = std::time(NULL);

	Reader reader;

	std::vector<int> instance_numbers;
	reader.ReadShrecInstanceNumbers(testList, instance_numbers);


	std::vector<cv::Mat> cvSPFHtest;
	reader.ReadSPFHList(testList,cvSPFHtest);

	std::cout<<"instance numbers size: "<<instance_numbers.size()<<std::endl;
	std::cout<<"# of test scans: "<<cvSPFHtest.size()<<std::endl;
	int numberofobjects;
	numberofobjects = *std::max_element(instance_numbers.begin(),instance_numbers.end());
	std::cout<<"# of test objects: "<<numberofobjects<<std::endl;


	Matcher matcher;


	std::time_t start_match = std::time(NULL);


	float instance_distances[numberofobjects][numberofobjects];

	for(int m=1;m<1+numberofobjects;m++)
	{

		std::vector<cv::Mat> cvQuery;
		for(int n=0;n<cvSPFHtest.size();n++)
		{
			if(instance_numbers[n]==m)
				cvQuery.push_back(cvSPFHtest[n]);

		}

		std::cout<<"Query size: "<<cvQuery.size()<<std::endl;

		for(int mm=1;mm<1+numberofobjects;mm++)
		{

			std::vector<cv::Mat> cvTest;
			for(int nn=0;nn<cvSPFHtest.size();nn++)
			{
				if(instance_numbers[nn] == mm)
					cvTest.push_back(cvSPFHtest[nn]);

			}
			std::cout<<"Test size: "<<cvTest.size()<<std::endl;

			std::vector<std::vector<cv::DMatch> > matches;
			matches = matcher.BruteforceMatcherL2(cvQuery,cvTest);

			float average_distance = 0;
			float sum=0;
			for(int ii=0;ii<matches.size();ii++){
				for(int jj=0;jj<matches[ii].size();jj++)
				{
					average_distance = average_distance + matches[ii][jj].distance;
					sum = sum + 1;
				}
			}
			average_distance = average_distance / sum;
			instance_distances[m-1][mm-1] = average_distance;

			std::cout<<" Query Object: "<<m<<"  ;  Test Object: "<<mm<<"  ;  Distance: "<<average_distance<<std::endl;
		}

		std::stringstream ss;
		ss<<"rankedlists/"<<run_number<<"-"<<m<<".res";
		ofstream of;
		of.open(ss.str().c_str());
		for(int mm=1;mm<1+numberofobjects;mm++)
		{
			of<<mm<<" "<<instance_distances[m-1][mm-1]<<std::endl;
		}
		of.close();
	}



	std::cout<<"Total Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;



	return 1;
}


int Evaluater::EvaluateSPFH_comphist(std::string testList, std::string trainList, int type, bool rgb)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;

	trainEvalList = reader.ReadTrainEvalTypes(trainList);
	std::cout<<trainEvalList.size()<<std::endl;

	std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList.size()<<std::endl;

	Reader reader2;
	std::vector<cv::Mat> cvSPFHtest;
	reader2.ReadSPFHList(Temp_Test,cvSPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvSPFHtrain;
	reader3.ReadSPFHList(Temp_Train,cvSPFHtrain);

	std::cout<<"# of test files: "<<cvSPFHtest.size()<<"; # of train files: "<<cvSPFHtrain.size()<<std::endl;

	std::cout<<cvSPFHtest[0].type()<<" "<<cvSPFHtest[0].cols<<" "<<cvSPFHtrain[0].cols<<std::endl;
	std::cout<<cvSPFHtest[0].type()<<" "<<cvSPFHtest[0].rows<<" "<<cvSPFHtrain[0].rows<<std::endl;

	Matcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;


	matches = matcher.HistogramMatcher(cvSPFHtest,cvSPFHtrain,type, rgb);


	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;


	int train_size = cvSPFHtrain.size();

	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;
		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);

	std::cout<<"categoryAccuracy: "<<categoryAccuracy<<" ; instanceAccuracy: "<<instanceAccuracy<<std::endl;

	return 1;
}

void Evaluater::calculateRecallPrecision()
{

	//CATEGORY
	v_categoryPrecision.clear();
	v_categoryRecall.clear();
	v_categoryFscore.clear();

	int numofpairs = categoryPairs.size();
	for (int i=1; i<=numberofCategories;i++)
	{
		int tp = 0;
		int tn = 0;
		int fp = 0;
		int fn = 0;
		int left = 0;
		int right = 0;
		float recall;
		float precision;
		float fscore;

		for (int j =0; j<categoryPairs.size();j++)
		{
			if((categoryPairs[j].first == i) && (categoryPairs[j].second == i))
				tp++;
			if((categoryPairs[j].first == i))
				left++;
			if((categoryPairs[j].second == i))
				right++;
		}
		fp = right - tp;
		fn = left - tp;
		tn = numofpairs - tp - fp - fn;

		if((tp+fn) == 0)
			recall = 0;
		else
			recall = tp*1.0 / ((tp + fn)*1.0);
		if((tp+fp) ==0)
			precision = 0;
		else
			precision = tp*1.0 / ((tp + fp)*1.0);
		if((tp + fp + fn) == 0)
			fscore =0;
		else
			fscore = tp*2.0 / ((2*tp + fp + fn)*1.0);
		v_categoryPrecision.push_back(precision);
		v_categoryRecall.push_back(recall);
		v_categoryFscore.push_back(fscore);
		//std::cout<<"Precision: "<<precision<<" ; Recall: "<<recall<<" ; FScore: "<<fscore<<std::endl;
	}
	categoryPrecision = std::accumulate(v_categoryPrecision.begin(), v_categoryPrecision.end(), 0.0) / v_categoryPrecision.size();
	categoryRecall = std::accumulate(v_categoryRecall.begin(), v_categoryRecall.end(), 0.0) / v_categoryRecall.size();
	categoryFscore = std::accumulate(v_categoryFscore.begin(), v_categoryFscore.end(), 0.0) / v_categoryFscore.size();
	//std::cout<<"Av. Cat. Precision: "<<categoryPrecision<<" ; Av. Cat. Recall: "<<categoryRecall<<" ; Av. Cat. FScore: "<<categoryFscore<<std::endl;

	//INSTANCE

	v_instancePrecision.clear();
	v_instanceRecall.clear();
	v_instanceFscore.clear();

	numofpairs = instancePairs.size();
	for (int i=1; i<=numberofInstances;i++)
	{
		int tp = 0;
		int tn = 0;
		int fp = 0;
		int fn = 0;
		int left = 0;
		int right = 0;
		float recall;
		float precision;
		float fscore;

		for (int j =0; j<instancePairs.size();j++)
		{
			if((instancePairs[j].first == i) && (instancePairs[j].second == i))
				tp++;
			if((instancePairs[j].first == i))
				left++;
			if((instancePairs[j].second == i))
				right++;
		}
		fp = right - tp;
		fn = left - tp;
		tn = numofpairs - tp - fp - fn;

		if((tp+fn) == 0)
			recall = 0;
		else
			recall = tp*1.0 / ((tp + fn)*1.0);
		if((tp+fp) ==0)
			precision = 0;
		else
			precision = tp*1.0 / ((tp + fp)*1.0);

		if((tp + fp + fn) == 0)
			fscore =0;
		else
			fscore = tp*2.0 / ((2*tp + fp + fn)*1.0);
		v_instancePrecision.push_back(precision);
		v_instanceRecall.push_back(recall);
		v_instanceFscore.push_back(fscore);
		//std::cout<<"Precision: "<<precision<<" ; Recall: "<<recall<<" ; FScore: "<<fscore<<std::endl;
	}
	instancePrecision = std::accumulate(v_instancePrecision.begin(), v_instancePrecision.end(), 0.0) / v_instancePrecision.size();
	instanceRecall = std::accumulate(v_instanceRecall.begin(), v_instanceRecall.end(), 0.0) / v_instanceRecall.size();
	instanceFscore = std::accumulate(v_instanceFscore.begin(), v_instanceFscore.end(), 0.0) / v_instanceFscore.size();
	//std::cout<<"Av. Ins. Precision: "<<instancePrecision<<" ; Av. Ins. Recall: "<<instanceRecall<<" ; Av. Ins. FScore: "<<instanceFscore<<std::endl;


}


int Evaluater::EvaluateSHOTCOLOR_flann(std::string testList, std::vector<std::string> trainLists)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;
	Reader reader2;
	std::vector<cv::Mat> cvSHOTtest;
	reader2.ReadSHOTColor(Temp_Test,cvSHOTtest);


	std::vector<std::vector<cv::DMatch> > matches;
	std::vector<std::vector<cv::DMatch> > matches_temp;
	int train_size = 0;
	for(int t=0;t<trainLists.size();t++){
		std::vector<PCDEval> trainEvalList_local;
		trainEvalList_local = reader.ReadTrainEvalTypes(trainLists[t]);
		std::cout<<trainEvalList_local.size()<<std::endl;
		std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList_local.size()<<std::endl;
		Reader reader3;
		std::vector<cv::Mat> cvSHOTtrain;
		reader3.ReadSHOTColor(Temp_Train,cvSHOTtrain);
		std::cout<<"# of test files: "<<cvSHOTtest.size()<<"; # of train files: "<<cvSHOTtrain.size()<<std::endl;
		train_size = train_size + cvSHOTtrain.size();

		//std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;
		Matcher matcher;
		std::time_t start_match = std::time(NULL);
		matches_temp = matcher.FlannMatcher(cvSHOTtest,cvSHOTtrain);
		std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

		if(t==0)
		{
			matches = matches_temp;
		}
		else{
			for(int i=0;i<matches_temp.size();i++){
				for(int j=0;j<matches_temp[i].size();j++){
					if(matches_temp[i][j].distance < matches[i][j].distance){
						matches[i][j].imgIdx = (matches_temp[i][j].imgIdx) + (train_size - cvSHOTtrain.size() );
						matches[i][j].distance = matches_temp[i][j].distance;
					}
				}
			}
		}
		trainEvalList.insert(trainEvalList.end(),trainEvalList_local.begin(),trainEvalList_local.end());
	}

	std::cout<<"trainEvalList size :"<<trainEvalList.size()<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;



	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;

		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}

int Evaluater::EvaluateSHOT_flann(std::string testList, std::vector<std::string> trainLists)
{
	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;
	std::time_t start_read = std::time(NULL);
	Reader reader;

	testEvalList = reader.ReadTestEvalTypes(testList);
	std::cout<<testEvalList.size()<<std::endl;
	Reader reader2;
	std::vector<cv::Mat> cvSHOTtest;
	reader2.ReadSHOT(Temp_Test,cvSHOTtest);


	std::vector<std::vector<cv::DMatch> > matches;
	std::vector<std::vector<cv::DMatch> > matches_temp;
	int train_size = 0;
	for(int t=0;t<trainLists.size();t++){
		std::vector<PCDEval> trainEvalList_local;
		trainEvalList_local = reader.ReadTrainEvalTypes(trainLists[t]);
		std::cout<<trainEvalList_local.size()<<std::endl;
		std::cout<<"# of test files: "<<testEvalList.size()<<"; # of train files: "<<trainEvalList_local.size()<<std::endl;
		Reader reader3;
		std::vector<cv::Mat> cvSHOTtrain;
		reader3.ReadSHOT(Temp_Train,cvSHOTtrain);
		std::cout<<"# of test files: "<<cvSHOTtest.size()<<"; # of train files: "<<cvSHOTtrain.size()<<std::endl;
		train_size = train_size + cvSHOTtrain.size();

		//std::cout<<"Time Elapsed to Read: "<<std::difftime(std::time(NULL), start_read)<<" seconds!"<<std::endl;
		Matcher matcher;
		std::time_t start_match = std::time(NULL);
		matches_temp = matcher.FlannMatcher(cvSHOTtest,cvSHOTtrain);
		std::cout<<"Time Elapsed to Match: "<<std::difftime(std::time(NULL), start_match)<<" seconds!"<<std::endl;

		if(t==0)
		{
			matches = matches_temp;
		}
		else{
			for(int i=0;i<matches_temp.size();i++){
				for(int j=0;j<matches_temp[i].size();j++){
					if(matches_temp[i][j].distance < matches[i][j].distance){
						matches[i][j].imgIdx = (matches_temp[i][j].imgIdx) + (train_size - cvSHOTtrain.size() );
						matches[i][j].distance = matches_temp[i][j].distance;
					}
				}
			}
		}
		trainEvalList.insert(trainEvalList.end(),trainEvalList_local.begin(),trainEvalList_local.end());
	}

	std::cout<<"trainEvalList size :"<<trainEvalList.size()<<std::endl;

	int instanceMatch = 0;
	int instanceNonMatch = 0;
	int categoryMatch = 0;
	int categoryNonMatch = 0;



	int imgID;
	std::cout<<"matches size: "<<matches.size()<<std::endl;


//	cv::Mat match_votes(1,train_size,CV_32S);
	int match_votes[train_size];
	for(int i=0;i<matches.size();i++){
		//majority voting
		int msize = matches[i].size();
		//std::cout<<"matched descriptors: "<<msize<<std::endl;
		for(int k=0;k<train_size;k++)
			match_votes[k] = 0;
//			match_votes.at<int>(1,k) = 0;
		for(int j=0;j<matches[i].size();j++){
			imgID = matches[i][j].imgIdx;
			//std::cout<<i<<" "<<j<<" "<<matches[i][j].queryIdx<<" "<<matches[i][j].trainIdx<<" "<<matches[i][j].imgIdx<<" "<<matches[i][j].distance<<std::endl;
			match_votes[imgID] = match_votes[imgID] + 1;
			//std::cout<<"current total vote: "<<match_votes[imgID] <<std::endl;

		}
		int max_vote = 0;
		int max_voted_img = 0;
		int vote;
		for(int k=0;k<train_size;k++){
			vote = match_votes[k];
//			vote = match_votes.at<int>(1,k);
			if(vote > max_vote){
				max_vote = vote;
				max_voted_img = k;
			}
		}
		//std::cout<<"Query image: "<<i<<std::endl;
		//std::cout<<"max voted image: "<<max_voted_img<<std::endl;
		//std::cout<<trainEvalList[max_voted_img].categoryNum<<" "<<trainEvalList[max_voted_img].instanceNum<<std::endl;

		std::pair<int,int> categorypair(testEvalList[i].categoryNum,trainEvalList[max_voted_img].categoryNum);
		std::pair<int,int> instancepair(testEvalList[i].instanceNum,trainEvalList[max_voted_img].instanceNum);
		categoryPairs.push_back(categorypair);
		instancePairs.push_back(instancepair);

		numberofCategories = testEvalList.back().categoryNum;
		numberofInstances = testEvalList.back().instanceNum;
		calculateRecallPrecision();

		if(testEvalList[i].categoryNum == trainEvalList[max_voted_img].categoryNum)
		{
			categoryMatch++;
		}
		else
			categoryNonMatch++;

		if(testEvalList[i].instanceNum == trainEvalList[max_voted_img].instanceNum)
		{
			instanceMatch++;
		}
		else
			instanceNonMatch++;

	}


	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);


	return 1;
}


int Evaluater::EvaluateSPAIR_SVM(std::string testList, std::string trainList,int kernelType, bool evalType)
{

	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;
	testEvalList = reader.ReadTestEvalTypes(testList);
	trainEvalList = reader.ReadTrainEvalTypes(trainList);

	Reader reader2;
	std::vector<cv::Mat> cvSPFHtest;
	reader2.ReadSPFHList(Temp_Test,cvSPFHtest);

	Reader reader3;
	std::vector<cv::Mat> cvSPFHtrain;
	reader3.ReadSPFHList(Temp_Train,cvSPFHtrain);



	//make the train and test single mat files

	int width = cvSPFHtrain[0].cols;
	//int height = cvSPFHtrain[0].rows;
	int numoftrainfiles = cvSPFHtrain.size();
	int numoftestfiles = cvSPFHtest.size();

	int totaltrainrows = 0;
	for(int k=0;k<numoftrainfiles;k++){
		int rownum = cvSPFHtrain[k].rows;
		totaltrainrows = totaltrainrows + rownum;
	}

	std::cout<<"Total rows in the train mat is = "<<totaltrainrows<<std::endl;
	cv::Mat trainMat(totaltrainrows,width,CV_32F);
	cv::Mat trainLabels(totaltrainrows,1,CV_32FC1);

	int currentrow =0;
	for(int k=0;k<numoftrainfiles;k++){
		for(int i=0;i<cvSPFHtrain[k].rows;i++){
			if(evalType==0)
				trainLabels.at<float>(currentrow,1) = float(trainEvalList[k].categoryNum);
			else
				trainLabels.at<float>(currentrow,1) = float(trainEvalList[k].instanceNum);
			for(int j=0;j<width;j++){
				trainMat.at<float>(currentrow,j) = cvSPFHtrain[k].at<float>(i,j);
			}
			currentrow++;
		}

	}

	int totaltestrows = 0;
	for(int k=0;k<numoftestfiles;k++){
		int rownum = cvSPFHtest[k].rows;
		totaltestrows = totaltestrows + rownum;
	}

	std::cout<<"Total rows in the test mat is = "<<totaltestrows<<std::endl;
	cv::Mat testMat(totaltestrows,width,CV_32F);
	cv::Mat testLabels(totaltestrows,1,CV_32FC1);

	currentrow =0;
	for(int k=0;k<numoftestfiles;k++){
		for(int i=0;i<cvSPFHtest[k].rows;i++){
			if(evalType==0)
				testLabels.at<float>(currentrow,1) = float(testEvalList[k].categoryNum);
			else
				testLabels.at<float>(currentrow,1) = float(testEvalList[k].instanceNum);
			for(int j=0;j<width;j++){
				testMat.at<float>(currentrow,j) = cvSPFHtest[k].at<float>(i,j);
			}
			currentrow++;
		}

	}

	//normalize features
	cv::normalize(testMat,testMat,0,1,cv::NORM_MINMAX,CV_32F);
	cv::normalize(trainMat,trainMat,0,1,cv::NORM_MINMAX,CV_32F);


	Ptr<SVM> svm = SVM::create();
	if(kernelType == 0)
		svm->setKernel(SVM::LINEAR);
	else if(kernelType == 1)
		svm->setKernel(SVM::POLY);
	else if (kernelType == 2)
		svm->setKernel(SVM::RBF);
	else if (kernelType == 3)
		svm->setKernel(SVM::SIGMOID);
	else
		svm->setKernel(SVM::RBF);

	svm->setType(SVM::C_SVC);
	svm->setNu(0.5);
	svm->setC(0.1);
	svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 1000, 1e-6));

    //------------------------ Train the SVM ----------------------------------------------------
    std::cout << "Starting training process" << std::endl;
    //CvSVM svm;
    Ptr<TrainData> trainData = TrainData::create(trainMat, ROW_SAMPLE, trainLabels);
    svm->train(trainData,0);
    //svm.train(trainMat, trainLabels, cv::Mat(), cv::Mat(), params);
    //svm.train_auto(trainMat, trainLabels, cv::Mat(), cv::Mat(), params,5);
    std::cout << "Finished training process" << std::endl;

    //-------------------------TEST----------------------------------------------------------


    	int instanceMatch = 0;
    	int instanceNonMatch = 0;
    	int categoryMatch = 0;
    	int categoryNonMatch = 0;

        cv::Mat results;
        svm->predict(testMat,results);
        std::cout<<"Results size: "<<results.rows<<std::endl;

        for(int i=0;i<results.rows;i++){
        	float result = results.at<float>(i);
        	//std::cout<<result<<" "<<int(result);
        	if(evalType == 0){
        		std::pair<int,int> categorypair(int(testLabels.at<float>(i,1)),int(result));
        		categoryPairs.push_back(categorypair);
    			if(testLabels.at<float>(i) == result)
    			{
    				categoryMatch++;
    			}
    			else
    				categoryNonMatch++;
        	}
        	else{
        		std::pair<int,int> instancepair(int(testLabels.at<float>(i,1)),int(result));
        		instancePairs.push_back(instancepair);
    			if(testLabels.at<float>(i) == result)
    			{
    				instanceMatch++;
    			}
    			else
    				instanceNonMatch++;
        	}
        }

        numberofCategories = testEvalList.back().categoryNum;
        numberofInstances = testEvalList.back().instanceNum;
        calculateRecallPrecision();

    	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
    	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);

	return 1;
}

int Evaluater::Evaluate_SVM(std::string testList, std::string trainList,int descriptorType, int kernelType, bool evalType, double c, double gamma)
{

	std::vector<PCDEval> testEvalList;
	std::vector<PCDEval> trainEvalList;

	Reader reader;
	testEvalList = reader.ReadTestEvalTypes(testList);
	trainEvalList = reader.ReadTrainEvalTypes(trainList);

	Reader reader2;
	Reader reader3;

	std::vector<cv::Mat> cvTest;
	std::vector<cv::Mat> cvTrain;
	if(descriptorType==0){
		reader2.ReadSPFHList(Temp_Test,cvTest);
		reader3.ReadSPFHList(Temp_Train,cvTrain);
	}
	else if (descriptorType==1){
		reader2.ReadSHOT(Temp_Test,cvTest);
		reader3.ReadSHOT(Temp_Train,cvTrain);
	}
	else if (descriptorType==2){
		reader2.ReadSHOTColor(Temp_Test,cvTest);
		reader3.ReadSHOTColor(Temp_Train,cvTrain);
	}
	else if (descriptorType==3){
		reader2.ReadFPFH(Temp_Test,cvTest);
		reader3.ReadFPFH(Temp_Train,cvTrain);
	}
	else if (descriptorType==4){
		reader2.ReadPFH(Temp_Test,cvTest);
		reader3.ReadPFH(Temp_Train,cvTrain);
	}
	else if (descriptorType==5){
		reader2.ReadPFHRGB(Temp_Test,cvTest);
		reader3.ReadPFHRGB(Temp_Train,cvTrain);
	}
	else{
		std::cout<<" Wrong descriptor type!"<<std::endl;
	}



	//make the train and test single mat files

	int width = cvTrain[0].cols;
	//int height = cvSPFHtrain[0].rows;
	int numoftrainfiles = cvTrain.size();
	int numoftestfiles = cvTest.size();

	int totaltrainrows = 0;
	for(int k=0;k<numoftrainfiles;k++){
		int rownum = cvTrain[k].rows;
		totaltrainrows = totaltrainrows + rownum;
	}

	std::cout<<"Total rows in the train mat is = "<<totaltrainrows<<std::endl;
	cv::Mat trainMat(totaltrainrows,width,CV_32F);
	cv::Mat trainLabels(totaltrainrows,1,CV_32SC1);

	int currentrow =0;
	for(int k=0;k<numoftrainfiles;k++){
		for(int i=0;i<cvTrain[k].rows;i++){
			if(evalType==0)
				//trainLabels.at<float>(currentrow,1) = float(trainEvalList[k].categoryNum);
				trainLabels.at<int>(currentrow,1) = trainEvalList[k].categoryNum;
			else
				//trainLabels.at<float>(currentrow,1) = float(trainEvalList[k].instanceNum);
				trainLabels.at<int>(currentrow,1) = trainEvalList[k].instanceNum;
			for(int j=0;j<width;j++){
				trainMat.at<float>(currentrow,j) = cvTrain[k].at<float>(i,j);
			}
			currentrow++;
		}

	}

	int totaltestrows = 0;
	for(int k=0;k<numoftestfiles;k++){
		int rownum = cvTest[k].rows;
		totaltestrows = totaltestrows + rownum;
	}

	std::cout<<"Total rows in the test mat is = "<<totaltestrows<<std::endl;
	cv::Mat testMat(totaltestrows,width,CV_32F);
	cv::Mat testLabels(totaltestrows,1,CV_32SC1);

	currentrow =0;
	for(int k=0;k<numoftestfiles;k++){
		for(int i=0;i<cvTest[k].rows;i++){
			if(evalType==0)
				//testLabels.at<float>(currentrow,1) = float(testEvalList[k].categoryNum);
				testLabels.at<int>(currentrow,1) = testEvalList[k].categoryNum;
			else
				//testLabels.at<float>(currentrow,1) = float(testEvalList[k].instanceNum);
				testLabels.at<int>(currentrow,1) = testEvalList[k].instanceNum;
			for(int j=0;j<width;j++){
				testMat.at<float>(currentrow,j) = cvTest[k].at<float>(i,j);
			}
			currentrow++;
		}

	}

	//normalize features
	cv::normalize(testMat,testMat,0,1,cv::NORM_MINMAX,CV_32F);
	cv::normalize(trainMat,trainMat,0,1,cv::NORM_MINMAX,CV_32F);


	Ptr<SVM> svm = SVM::create();
	if(kernelType == 0)
		svm->setKernel(SVM::LINEAR);
	else if(kernelType == 1)
		svm->setKernel(SVM::POLY);
	else if (kernelType == 2)
		svm->setKernel(SVM::RBF);
	else if (kernelType == 3)
		svm->setKernel(SVM::SIGMOID);
	else
		svm->setKernel(SVM::RBF);

	svm->setType(SVM::C_SVC);
	svm->setNu(0.5);
	svm->setC(0.1);
	svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 1000, 1e-6));

    //------------------------ Train the SVM ----------------------------------------------------
    std::cout << "Starting training process" << std::endl;
    //CvSVM svm;
    //svm.train(trainMat, trainLabels, cv::Mat(), cv::Mat(), params);
    Ptr<TrainData> trainData = TrainData::create(trainMat, ROW_SAMPLE, trainLabels);
    svm->train(trainData,0);

    //svm.train_auto(trainMat, trainLabels, cv::Mat(), cv::Mat(), params);
    std::cout << "Finished training process" << std::endl;

    //-------------------------TEST----------------------------------------------------------


    	int instanceMatch = 0;
    	int instanceNonMatch = 0;
    	int categoryMatch = 0;
    	int categoryNonMatch = 0;

        cv::Mat results;
        svm->predict(testMat,results);
        std::cout<<"Results size: "<<results.rows<<std::endl;

        for(int i=0;i<results.rows;i++){
        	float result = results.at<float>(i);
        	//std::cout<<result<<" "<<int(result);
        	if(evalType == 0){
        		//std::pair<int,int> categorypair(int(testLabels.at<float>(i,1)),int(result));
        		std::pair<int,int> categorypair(testLabels.at<int>(i,1),int(result));
        		categoryPairs.push_back(categorypair);
    			if(testLabels.at<int>(i) == result)
    			{
    				categoryMatch++;
    			}
    			else
    				categoryNonMatch++;
        	}
        	else{
        		//std::pair<int,int> instancepair(int(testLabels.at<float>(i,1)),int(result));
        		std::pair<int,int> instancepair(testLabels.at<int>(i,1),int(result));
        		instancePairs.push_back(instancepair);
    			if(testLabels.at<int>(i) == result)
    			{
    				instanceMatch++;
    			}
    			else
    				instanceNonMatch++;
        	}
        }

        numberofCategories = testEvalList.back().categoryNum;
        numberofInstances = testEvalList.back().instanceNum;
        calculateRecallPrecision();

    	categoryAccuracy = (1.0 * categoryMatch) / (categoryMatch + categoryNonMatch);
    	instanceAccuracy = (1.0 * instanceMatch) / (instanceMatch + instanceNonMatch);

	return 1;
}


