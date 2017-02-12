/*
 * Matcher.h
 *
 *  Created on: Dec 7, 2013
 *      Author: berker
 */

#ifndef MATCHER_H_
#define MATCHER_H_

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc.hpp"




class Matcher {
public:
	Matcher();
	virtual ~Matcher();
	std::vector<cv::DMatch> FlannMatcher(cv::Mat &queryDescriptors, std::vector<cv::Mat> &trainDescriptors);
	std::vector<cv::DMatch> FlannMatcher(cv::Mat &queryDescriptors, cv::Mat &trainDescriptors);
	std::vector<std::vector<cv::DMatch> > FlannMatcher(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat> &trainDescriptors);
	std::vector<std::vector<cv::DMatch> > BruteForceMatcherL2KNN(const cv::Mat &queryDescriptors, const cv::Mat& trainDescriptors, int k);
	std::vector<std::vector<cv::DMatch> > FlannMatcherBinary(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat> &trainDescriptors);
	std::vector<std::vector<cv::DMatch> > BruteforceMatcherHamming(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat> &trainDescriptors);
	std::vector<std::vector<cv::DMatch> > BruteforceMatcherL2(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat> &trainDescriptors);
	std::vector<std::vector<cv::DMatch> > HistogramMatcher(std::vector<cv::Mat> &queryDescriptors, std::vector<cv::Mat> &trainDescriptors, int type, bool RGB);
	std::vector<cv::DMatch>  CompareHistogram(cv::Mat &queryDescriptor, std::vector<cv::Mat> &trainDescriptors, int type, bool RGB);
};

#endif /* MATCHER_H_ */
