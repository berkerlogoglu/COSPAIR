/*
 * Evaluater.h
 *
 *  Created on: Dec 9, 2013
 *      Author: berker
 */

#ifndef EVALUATER_H_
#define EVALUATER_H_

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <utility>
#include <ctime>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

struct PCDEval;

class Evaluater {
public:
	Evaluater();
	virtual ~Evaluater();
	int EvaluateSPAIR_SVM(std::string testList, std::string trainList,int kernelType, bool evalType);
	int Evaluate_SVM(std::string testList, std::string trainList,int descriptorType, int kernelType, bool evalType, double c, double gamma);

	int EvaluateVFH_flann(std::string testList, std::string trainList);
	int EvaluateVFH_bf(std::string testList, std::string trainList);
	int EvaluateVFH_SVM(std::string testList, std::string trainList,int kernelType, bool evalType);
	int EvaluateESF_flann(std::string testList, std::string trainList);
	int EvaluateESF_bf(std::string testList, std::string trainList);
	int EvaluateSHOT_flann(std::string testList, std::string trainList);
	int EvaluateSHOT_flann(std::string testList, std::vector<std::string> trainList);
	int EvaluateSHOT_bf(std::string testList, std::string trainList);
	int EvaluateSHOTCOLOR_flann(std::string testList, std::string trainList);
	int EvaluateSHOTCOLOR_flann(std::string testList, std::vector<std::string> trainList);

	int EvaluateSHOTCOLOR_bf(std::string testList, std::string trainList);

	int EvaluatePFH_flann(std::string testList, std::string trainList);
	int EvaluatePFH_bf(std::string testList, std::string trainList);
	int EvaluatePFHRGB_flann(std::string testList, std::string trainList);
	int EvaluatePFHRGB_bf(std::string testList, std::string trainList);


	int EvaluateFPFH_flann(std::string testList, std::string trainList);
	int EvaluateFPFH_bf(std::string testList, std::string trainList);
	int EvaluateBOP_bf(std::string testList, std::string trainList);
	int EvaluateBOP_hamming(std::string testList, std::string trainList);
	int EvaluateBOP_flann(std::string testList, std::string trainList);
	int EvaluateSPFH_bf(std::string testList, std::string trainList);
	int EvaluateSPFH_flann(std::string testList, std::string trainList);

	int EvaluateSPFH_shrec_bf(std::string testList, int run_number);
	int EvaluateSPFH_shrec_flann(std::string testList, int run_number);

	int EvaluateSPFH_comphist(std::string testList, std::string trainList, int type, bool rgb);
	//feature type =  1 for FPFH, ...
	int EvaluateBOW_flann(std::string vocabulary_name, int feature_type, std::string testList, std::string trainList);
	std::vector< std::vector<int> > getCategoryScores();
	std::vector< std::vector<int> >  getInstanceScores();
	float getCategoryAccuracy();
	float getInstanceAccuracy();
	std::vector< std::pair<int,int> > getCategoryPairs();
	std::vector< std::pair<int,int> > getInstancePairs();
	void calculateRecallPrecision();
	int numberofCategories;
	int numberofInstances;

	float categoryRecall;
	float instanceRecall;
	float categoryPrecision;
	float instancePrecision;
	float categoryFscore;
	float instanceFscore;

	std::vector<float> v_categoryRecall;
	std::vector<float> v_instanceRecall;
	std::vector<float> v_categoryPrecision;
	std::vector<float> v_instancePrecision;
	std::vector<float> v_categoryFscore;
	std::vector<float> v_instanceFscore;

private:
	std::vector< std::vector<int> > categoryScores;
	std::vector< std::vector<int> > instanceScores;
	float categoryAccuracy;
	float instanceAccuracy;



	std::string Temp_Test;
	std::string Temp_Train;

	std::vector< std::pair<int,int> > categoryPairs;
	std::vector< std::pair<int,int> > instancePairs;
};

#endif /* EVALUATER_H_ */
