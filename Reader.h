/*
 * Reader.h
 *
 *  Created on: Dec 4, 2013
 *      Author: berker
 */

#ifndef READER_H_
#define READER_H_

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>


//OPENCV
#include <opencv2/core/core.hpp>


struct PCDEval
{
	std::string fileName;
	int categoryNum;
	std::string categoryName;
	int instanceNum;
	std::string instanceName;
};

class Reader {
public:
	Reader();
	virtual ~Reader();
	int ReadVFH(std::string pcd_list,std::vector<pcl::VFHSignature308> &VFHdescriptors);
	int ReadVFH(std::string pcd_list,std::vector<cv::Mat> &VFHdescriptors);
	int ReadVFH(std::string pcd_list, cv::Mat &VFHdescriptors);

	int ReadESF(std::string pcd_list,std::vector<cv::Mat> &ESFdescriptors);
	int ReadESF(std::string pcd_list, cv::Mat &ESFdescriptors);

	int ReadFPFH(std::string pcd_list,std::vector<cv::Mat> &FPFHdescriptors);
	int ReadFPFH(std::string pcd_list,cv::Mat &FPFHdescriptors);

	int ReadSHOT(std::string pcd_list,std::vector<cv::Mat> &SHOTdescriptors);
	int ReadSHOTColor(std::string pcd_list,std::vector<cv::Mat> &SHOTColordescriptors);

	int ReadPFH(std::string pcd_list,std::vector<cv::Mat> &PFHdescriptors);
	int ReadPFHRGB(std::string pcd_list,std::vector<cv::Mat> &PFHRGBdescriptors);

	int CovertFromListToCategory(std::string input, std::string output);
	int CovertFromListToCategory(std::string input, int sampleRate, int testposnum, std::string testfilename, std::string trainfilename);

	int CovertFromListToCategory_bigbird(std::string input, int sampleRate, std::vector<int> testposnum, std::string testfilename, std::string trainfilename);
	int CovertFromListToCategory_amazon(std::string input, int sampleRate, std::vector<int> testposnum, std::string testfilename, std::string trainfilename);
	int CovertFromListToCategory_amazon2(std::string input, int sampleRate, std::vector<int> testposnum, std::string testfilename, std::string trainfilename);
	int CovertFromListToCategory_microsoft(std::string input, int sampleRate, int testposnum, std::string testfilename, std::string trainfilename);

	int BuildMasterCategoryInstanceMap(std::string master_category_list, std::map<std::string, int> &category_map, std::map<std::string, int> &instance_map);

	std::vector<PCDEval>  ReadTestEvalTypes(std::string list);
	std::vector<PCDEval>  ReadTrainEvalTypes(std::string list);

	int ReadBOPList(std::string pcd_list,std::vector<cv::Mat> &BOPdescriptors);
	int ReadBOPListuchar(std::string pcd_list,std::vector<cv::Mat> &BOPdescriptors);
	int ReadBOP(std::string pcd_file,cv::Mat &BOPdescriptors);
	int ReadBOPuchar(std::string pcd_file,cv::Mat &BOPdescriptors);
	int ReadBOPfast(std::string pcd_file,cv::Mat &BOPdescriptors);

	int ReadSPFH(std::string pcd_file,cv::Mat &SPFHdescriptors);
	int ReadSPFHList(std::string pcd_list,std::vector<cv::Mat> &SPFHdescriptors);

	int ReadShrecInstanceNumbers(std::string list, std::vector<int> &instance_numbers);



private:
	std::string Temp_Test;
	std::string Temp_Train;
};

#endif /* READER_H_ */
