/*
 * Reader.cpp
 *
 *  Created on: Dec 4, 2013
 *      Author: berker
 */

#include "Reader.h"



Reader::Reader() {
	// TODO Auto-generated constructor stub
	Temp_Test = "temp_test.txt";
	Temp_Train = "temp_train.txt";
}

Reader::~Reader() {
	// TODO Auto-generated destructor stub
}



std::vector<PCDEval>  Reader::ReadTestEvalTypes(std::string list)
{
	std::vector<PCDEval> EvalList;

	std::string filename,instancename,categoryname;

	std::ifstream file;
	std::ofstream ofile;

	file.open(list.c_str());
	ofile.open(Temp_Test.c_str());
	int categorynum,instancenum;

	bool firstline = true;
	while(!file.eof())
	{
		PCDEval pcdeval;

		file>>filename>>categorynum>>categoryname>>instancenum>>instancename;
		if(!firstline)
			ofile<<"\n";
		firstline = false;
		ofile<<filename;

		pcdeval.fileName = filename;
		pcdeval.categoryNum = categorynum;
		pcdeval.categoryName = categoryname;
		pcdeval.instanceNum = instancenum;
		pcdeval.instanceName = instancename;

		EvalList.push_back(pcdeval);
	}
	file.close();
	ofile.close();



	return EvalList;
}

std::vector<PCDEval>  Reader::ReadTrainEvalTypes(std::string list)
{
	std::vector<PCDEval> EvalList;

	std::string filename,instancename,categoryname;

	std::ifstream file;
	std::ofstream ofile;

	file.open(list.c_str());
	ofile.open(Temp_Train.c_str());
	int categorynum,instancenum;

	bool firstline = true;
	while(!file.eof())
	{
		PCDEval pcdeval;

		file>>filename>>categorynum>>categoryname>>instancenum>>instancename;
		if(!firstline)
			ofile<<"\n";
		firstline = false;
		ofile<<filename;

		pcdeval.fileName = filename;
		pcdeval.categoryNum = categorynum;
		pcdeval.categoryName = categoryname;
		pcdeval.instanceNum = instancenum;
		pcdeval.instanceName = instancename;

		EvalList.push_back(pcdeval);
	}
	file.close();
	ofile.close();



	return EvalList;
}

int Reader::ReadVFH(std::string pcd_list,std::vector<pcl::VFHSignature308> &VFHdescriptors)
{
//	pcl::PCDReader pcdreader;
//
//	std::ifstream file;
//	file.open(pcd_list.c_str());
//	std::string filename;
//	while(!file.eof())
//	{
//		file>>filename;
//		std::cout<<"Reading: "<<filename<<std::endl;
//
//		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_read (new pcl::PointCloud<pcl::VFHSignature308> ());
//		pcdreader.read(filename,*vfh_read);
//
//		//VFHdescriptors.push_back(*vfh_read);
//
//	}
	return 1;
}


int Reader::ReadVFH(std::string pcd_list,std::vector<cv::Mat> &VFHdescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_signature (new pcl::PointCloud<pcl::VFHSignature308> ());
		pcdreader.read(filename,*vfh_signature);
		cv::Mat cv_vfh_descriptor(1,308,CV_32F);
		for(int i=0;i<308;i++){
			cv_vfh_descriptor.at<float>(i) = vfh_signature->points[0].histogram[i];
			//std::cout<<cv_vfh_descriptor.at<float>(i)<<" ";
		}
		VFHdescriptors.push_back(cv_vfh_descriptor);

	}
	return 1;
}

int Reader::ReadVFH(std::string pcd_list, cv::Mat &VFHdescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;

	int nlines = std::count(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>(), '\n');
	nlines = nlines +1;
	std::cout<<" Number of test files: "<<nlines<<std::endl;
	VFHdescriptors = cv::Mat(nlines,308,CV_32F);
	file.close();

	file.open(pcd_list.c_str());

	while(!file.eof())
	{
		int i = 0;
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_signature (new pcl::PointCloud<pcl::VFHSignature308> ());
		pcdreader.read(filename,*vfh_signature);

		for(int j=0;j<308;j++){
			VFHdescriptors.at<float>(i,j) = vfh_signature->points[0].histogram[j];
			//std::cout<<VFHdescriptors.at<float>(i,j)<<" ";
		}
		std::cout<<std::endl;
		i++;
	}


	return 1;
}

int Reader::ReadESF(std::string pcd_list,std::vector<cv::Mat> &ESFdescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_signature (new pcl::PointCloud<pcl::ESFSignature640> ());
		pcdreader.read(filename,*esf_signature);
		cv::Mat cv_esf_descriptor(1,640,CV_32F);
		for(int i=0;i<640;i++){
			cv_esf_descriptor.at<float>(i) = esf_signature->points[0].histogram[i];
			//std::cout<<cv_esf_descriptor.at<float>(i)<<" ";
		}
		ESFdescriptors.push_back(cv_esf_descriptor);

	}
	return 1;
}

int Reader::ReadESF(std::string pcd_list, cv::Mat &ESFdescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;

	int nlines = std::count(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>(), '\n');
	nlines = nlines +1;
	std::cout<<" Number of test files: "<<nlines<<std::endl;
	ESFdescriptors = cv::Mat(nlines,640,CV_32F);
	file.close();

	file.open(pcd_list.c_str());

	while(!file.eof())
	{
		int i = 0;
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_signature (new pcl::PointCloud<pcl::ESFSignature640> ());
		pcdreader.read(filename,*esf_signature);

		for(int j=0;j<640;j++){
			ESFdescriptors.at<float>(i,j) = esf_signature->points[0].histogram[j];
			//std::cout<<ESFdescriptors.at<float>(i,j)<<" ";
		}
		std::cout<<std::endl;
		i++;
	}


	return 1;
}

int Reader::ReadFPFH(std::string pcd_list,std::vector<cv::Mat> &FPFHdescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_signature (new pcl::PointCloud<pcl::FPFHSignature33> ());
		pcdreader.read(filename,*fpfh_signature);
		int num_points = fpfh_signature->width;
		cv::Mat cv_fpfh_signature(num_points,33,CV_32F);
		for(int j=0;j<num_points;j++){
			for(int i=0;i<33;i++){
				cv_fpfh_signature.at<float>(j,i) = fpfh_signature->points[j].histogram[i];
				//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
			}
			//std::cout<<std::endl;

		}
		FPFHdescriptors.push_back(cv_fpfh_signature);

	}
	file.close();
	return 1;
}

int Reader::ReadFPFH(std::string pcd_list,cv::Mat &FPFHdescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;

	std::vector<cv::Mat> vFPFHdescriptors;

	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_signature (new pcl::PointCloud<pcl::FPFHSignature33> ());
		pcdreader.read(filename,*fpfh_signature);
		int num_points = fpfh_signature->width;
		if(num_points > 0){
			std::cout<<"Reading "<<num_points<<" descriptor(s)"<<std::endl;
			cv::Mat cv_fpfh_signature(num_points,33,CV_32F);

			for(int j=0;j<num_points;j++){
				for(int i=0;i<33;i++){
					cv_fpfh_signature.at<float>(j,i) = fpfh_signature->points[j].histogram[i];
					//std::cout<<cv_fpfh_signature.at<float>(j,i)<<" ";
				}
				//std::cout<<std::endl;

			}
			vFPFHdescriptors.push_back(cv_fpfh_signature);
		}

	}

	int totalsize = 0;
	std::vector<int>filesizes;
	for(int i=0;i<vFPFHdescriptors.size();i++)
	{
		totalsize = totalsize + vFPFHdescriptors[i].rows;
	}
	FPFHdescriptors = cv::Mat(totalsize, 33, CV_32F);

	int p = 0;
	for(int i=0;i<vFPFHdescriptors.size();i++){
		int num_points = vFPFHdescriptors[i].rows;
		for(int j=0;j<num_points;j++){
			for(int k=0;k<33;k++){
				FPFHdescriptors.at<float>(p,k) = vFPFHdescriptors[i].at<float>(j,k);
				//std::cout<<FPFHdescriptors.at<float>(p,k)<<" ";
			}
			//std::cout<<std::endl;
			p++;
		}
	}
//	std::cout<<"totalsize: "<<totalsize<<"; p = "<<p<<std::endl;
	return 1;
}
int Reader::ReadSHOT(std::string pcd_list,std::vector<cv::Mat> &SHOTdescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::SHOT352>::Ptr shot_signature (new pcl::PointCloud<pcl::SHOT352> ());
		pcdreader.read(filename,*shot_signature);
		int num_points = shot_signature->width;
		std::cout<<"signature width: "<<num_points<<std::endl;
		cv::Mat cv_shot_signature(num_points,352,CV_32F);
		for(int j=0;j<num_points;j++){
			for(int i=0;i<352;i++){
				cv_shot_signature.at<float>(j,i) = shot_signature->points[j].descriptor[i];
				//std::cout<<cv_shot_signature.at<float>(j,i)<<" ";
				//std::cout<<shot_signature->points[j].descriptor[i]<<" ";
			}
			//std::cout<<std::endl;

		}
		SHOTdescriptors.push_back(cv_shot_signature);

	}
	return 1;
}

int Reader::ReadSHOTColor(std::string pcd_list,std::vector<cv::Mat> &SHOTColordescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::SHOT1344>::Ptr shotcolor_signature (new pcl::PointCloud<pcl::SHOT1344> ());
		pcdreader.read(filename,*shotcolor_signature);
		int num_points = shotcolor_signature->width;
		cv::Mat cv_shotcolor_signature(num_points,1344,CV_32F);
		for(int j=0;j<num_points;j++){
			for(int i=0;i<1344;i++){
				cv_shotcolor_signature.at<float>(j,i) = shotcolor_signature->points[j].descriptor[i];
				//std::cout<<cv_shotcolor_signature.at<float>(j,i)<<" ";
			}
			//std::cout<<std::endl;

		}
		SHOTColordescriptors.push_back(cv_shotcolor_signature);

	}
	return 1;
}

int Reader::ReadPFH(std::string pcd_list,std::vector<cv::Mat> &PFHdescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_signature (new pcl::PointCloud<pcl::PFHSignature125> ());
		pcdreader.read(filename,*pfh_signature);
		int num_points = pfh_signature->width;
		std::cout<<"signature width: "<<num_points<<std::endl;
		cv::Mat cv_pfh_signature(num_points,125,CV_32F);
		for(int j=0;j<num_points;j++){
			for(int i=0;i<125;i++){
				cv_pfh_signature.at<float>(j,i) = pfh_signature->points[j].histogram[i];
				//std::cout<<cv_shot_signature.at<float>(j,i)<<" ";
				//std::cout<<shot_signature->points[j].descriptor[i]<<" ";
			}
			//std::cout<<std::endl;

		}
		PFHdescriptors.push_back(cv_pfh_signature);

	}
	return 1;
}

int Reader::ReadPFHRGB(std::string pcd_list,std::vector<cv::Mat> &PFHRGBdescriptors)
{
	pcl::PCDReader pcdreader;

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;

		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfh_signature (new pcl::PointCloud<pcl::PFHRGBSignature250> ());
		pcdreader.read(filename,*pfh_signature);
		int num_points = pfh_signature->width;
		std::cout<<"signature width: "<<num_points<<std::endl;
		cv::Mat cv_pfh_signature(num_points,250,CV_32F);
		for(int j=0;j<num_points;j++){
			for(int i=0;i<250;i++){
				cv_pfh_signature.at<float>(j,i) = pfh_signature->points[j].histogram[i];
				//std::cout<<cv_shot_signature.at<float>(j,i)<<" ";
				//std::cout<<shot_signature->points[j].descriptor[i]<<" ";
			}
			//std::cout<<std::endl;

		}
		PFHRGBdescriptors.push_back(cv_pfh_signature);

	}
	return 1;
}


int Reader::CovertFromListToCategory(std::string input, std::string output)
{
	std::fstream file;
	std::ofstream out(output.c_str());

	file.open(input.c_str());

	std::string line, filename;
	int category = 0;
	int instance = 0;

	int linecount=0;

	std::string instance_name;
	std::string prev_instance_name = "";

	std::string category_name;
	std::string prev_category_name = "";

	while(!file.eof())
	{
		unsigned found2, found3, found4;

		file>>filename;
		unsigned found1 = filename.find_last_of("/");
		std::cout<<filename<<std::endl;
		line = filename.substr(found1+1);
		//std::cout<<line<<std::endl;
		std::size_t found = line.find(".pcd");
		line = line.substr(0,found);
		std::cout<<line<<std::endl;
		if (found!=std::string::npos){
			unsigned t = std::count(line.begin(), line.end(), '_');
			//std::cout<<"# of _: "<<t<<std::endl;
			if(t==3)
			{
				found2 = line.find("_");
				category_name = line.substr(0,found2);
				found3 = line.find("_",found2+1);
				instance_name = line.substr(0,found3);
			}
			else if (t==4)
			{
				found2 = line.find("_");
				found3 = line.find("_",found2+1);
				category_name = line.substr(0,found3);
				found4 = line.find("_",found3+1);
				instance_name = line.substr(0,found4);

			}
			//std::cout<<category_name<<" "<<instance_name<<std::endl;

			if(instance_name!=prev_instance_name){
				instance++;
				prev_instance_name = instance_name;

			}
			if(category_name!=prev_category_name){
				category++;
				prev_category_name = category_name;
			}
			out<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
		}
	}
	out.close();




	return 1;
}

int Reader::CovertFromListToCategory(std::string input, int sampleRate, int testposnum, std::string testfilename, std::string trainfilename)
{
	std::fstream file;


	std::ofstream out_test(testfilename.c_str());
	std::ofstream out_train(trainfilename.c_str());

	file.open(input.c_str());

	std::string line,filename;
	int category = 0;
	int instance = 0;

	int linecount=0;

	std::string instance_name;
	std::string prev_instance_name = "";

	std::string category_name;
	std::string prev_category_name = "";

	bool testpose;
	std::string found_pose, found_samplenum;

	int posnum;

	while(!file.eof())
	{
		testpose = 0;
		int samplenum;
		unsigned found2, found3, found4, found5;
		file>>filename;
		unsigned found1 = filename.find_last_of("/");
		line = filename.substr(found1+1);
		//std::cout<<line<<std::endl;

		std::size_t found = line.find(".pcd");
		line = line.substr(0,found);
		//std::cout<<line<<std::endl;
		if (found!=std::string::npos){
			unsigned t = std::count(line.begin(), line.end(), '_');
			if(t==3)
			{
				found2 = line.find("_");
				category_name = line.substr(0,found2);
				found3 = line.find("_",found2+1);
				instance_name = line.substr(0,found3);
				found_pose = line[found3+1];
				posnum = std::atoi(found_pose.c_str());
				found5 = line.find(".pcd",found3+1);
				found_samplenum = line.substr(found3+3, found5-found3-3);
				samplenum = std::atoi(found_samplenum.c_str());

			}
			else if (t==4)
			{
				found2 = line.find("_");
				found3 = line.find("_",found2+1);
				category_name = line.substr(0,found3);
				found4 = line.find("_",found3+1);
				instance_name = line.substr(0,found4);
				found_pose = line[found4+1];
				posnum = std::atoi(found_pose.c_str());
				found5 = line.find(".pcd",found4+1);
				found_samplenum = line.substr(found4+3, found5-found4-3);
				samplenum = std::atoi(found_samplenum.c_str());


			}
			std::cout<<"pose number: "<<posnum<<" "<<"sample number: "<<samplenum<<" "<<category_name<<" "<<instance_name<<std::endl;

			if(instance_name!=prev_instance_name){
				//std::cout<<instance<<std::endl;
				instance++;
				//std::cout<<instance<<std::endl;
				prev_instance_name = instance_name;
			}
			if(category_name!=prev_category_name){
				category++;
				prev_category_name = category_name;
			}
			if((samplenum % sampleRate) == 0)
			{

				if(posnum == testposnum)
					out_test<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
				else
					out_train<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
			}

		}
	}

	out_test.close();
	out_train.close();




	return 1;
}



int Reader::CovertFromListToCategory_bigbird(std::string input, int sampleRate, std::vector<int> testposnum, std::string testfilename, std::string trainfilename)
{
	std::fstream file;


	std::ofstream out_test(testfilename.c_str());
	std::ofstream out_train(trainfilename.c_str());

	file.open(input.c_str());

	std::string line,filename;
	int category = 0;
	int instance = 0;

	int linecount=0;

	std::string instance_name;
	std::string prev_instance_name = "";

	std::string category_name;
	std::string prev_category_name = "";

	bool testpose;
	std::string found_pose, found_samplenum;

	int posnum;

	while(!file.eof())
	{
		testpose = 0;
		int samplenum;
		unsigned found2, found3, found4, found5;
		file>>filename;
		unsigned found1 = filename.find_last_of("/");
		line = filename.substr(found1+1);
		std::cout<<filename<<std::endl;
		//std::cout<<line<<std::endl;

		std::size_t found = line.find(".pcd");
		line = line.substr(0,found);
		std::cout<<line<<std::endl;
		if (found!=std::string::npos){
			//std::cout<<line.substr(4)<<std::endl;
			//posnum = atoi(line.substr(2,1).c_str());
			//samplenum = atoi(line.substr(4).c_str());

			//found2 = filename.find("bigbird");
			//found3 = filename.find("clouds");
			//std::cout<<filename.substr(found2+8,found3-found2-9)<<std::endl;
			//instance_name = filename.substr(found2+8,found3-found2-9);
			//found4 = instance_name.find("_");
			//category_name = instance_name.substr(0,found4);

			found2 = line.find("_NP");
			//std::cout<<found2<<" "<<line.substr(found2+3,1).c_str()<<" "<<line.substr(found2+5,1).c_str()<<std::endl;
			posnum = atoi(line.substr(found2+3,1).c_str());
			samplenum = atoi(line.substr(found2+5).c_str());
			instance_name = line.substr(0,found2);
			found3 = instance_name.find_first_of("_");
			category_name = instance_name.substr(0,found3);
			std::cout<<"pose number: "<<posnum<<" "<<"sample number: "<<samplenum<<" "<<category_name<<" "<<instance_name<<std::endl;

			if(instance_name!=prev_instance_name){
				instance++;
				prev_instance_name = instance_name;
			}
			if(category_name!=prev_category_name){
				category++;
				prev_category_name = category_name;
			}
			if((samplenum % sampleRate) == 0)
			{
				//out<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
				if(std::find(testposnum.begin(), testposnum.end(), posnum) != testposnum.end())
					out_test<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
				else
					out_train<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
			}

		}
	}

	out_test.close();
	out_train.close();

	return 1;

}

int Reader::CovertFromListToCategory_amazon(std::string input, int sampleRate, std::vector<int> testposnum, std::string testfilename, std::string trainfilename)
{
	std::fstream file;


	std::ofstream out_test(testfilename.c_str());
	std::ofstream out_train(trainfilename.c_str());

	file.open(input.c_str());

	std::string line,filename;
	int category = 0;
	int instance = 0;

	int linecount=0;

	std::string instance_name;
	std::string prev_instance_name = "";

	std::string category_name;
	std::string prev_category_name = "";

	bool testpose;
	std::string found_pose, found_samplenum;

	int posnum;

	while(!file.eof())
	{
		testpose = 0;
		int samplenum;
		unsigned found2, found3, found4, found5;
		file>>filename;
		unsigned found1 = filename.find_last_of("/");
		line = filename.substr(found1+1);
		std::cout<<filename<<std::endl;
		//std::cout<<line<<std::endl;

		std::size_t found = line.find(".pcd");
		line = line.substr(0,found);
		std::cout<<line<<std::endl;
		if (found!=std::string::npos){
			//std::cout<<line.substr(4)<<std::endl;
			//posnum = atoi(line.substr(2,1).c_str());
			//samplenum = atoi(line.substr(4).c_str());

			//found2 = filename.find("bigbird");
			//found3 = filename.find("clouds");
			//std::cout<<filename.substr(found2+8,found3-found2-9)<<std::endl;
			//instance_name = filename.substr(found2+8,found3-found2-9);
			//found4 = instance_name.find("_");
			//category_name = instance_name.substr(0,found4);

			found2 = line.find("_NP");
			//std::cout<<found2<<" "<<line.substr(found2+3,1).c_str()<<" "<<line.substr(found2+5,1).c_str()<<std::endl;
			posnum = atoi(line.substr(found2+3,1).c_str());
			samplenum = atoi(line.substr(found2+5).c_str());


			found3 = filename.find("amazon");
			found4 = filename.find("clouds");

			instance_name = filename.substr(found3+7,found4-1-found3-7);
			//found3 = instance_name.find_first_of("_");
			category_name = instance_name;


			std::cout<<"pose number: "<<posnum<<" "<<"sample number: "<<samplenum<<" "<<category_name<<" "<<instance_name<<std::endl;

			if(instance_name!=prev_instance_name){
				instance++;
				prev_instance_name = instance_name;
			}
			if(category_name!=prev_category_name){
				category++;
				prev_category_name = category_name;
			}
			if((samplenum % sampleRate) == 0)
			{
				//out<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
				if(std::find(testposnum.begin(), testposnum.end(), posnum) != testposnum.end())
					out_test<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
				else
					out_train<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
			}

		}
	}

	out_test.close();
	out_train.close();

	return 1;

}

int Reader::CovertFromListToCategory_amazon2(std::string input, int sampleRate, std::vector<int> testposnum, std::string testfilename, std::string trainfilename)
{
	std::fstream file;


	std::ofstream out_test(testfilename.c_str());
	std::ofstream out_train(trainfilename.c_str());

	file.open(input.c_str());

	std::string line,filename;
	int category = 0;
	int instance = 0;

	int linecount=0;

	std::string instance_name;
	std::string prev_instance_name = "";

	std::string category_name;
	std::string prev_category_name = "";

	bool testpose;
	std::string found_pose, found_samplenum;

	int posnum;

	while(!file.eof())
	{
		testpose = 0;
		int samplenum;
		unsigned found2, found3, found4, found5;
		file>>filename;
		unsigned found1 = filename.find_last_of("/");
		line = filename.substr(found1+1);
		std::cout<<filename<<std::endl;
		//std::cout<<line<<std::endl;

		std::size_t found = line.find(".pcd");
		line = line.substr(0,found);
		std::cout<<line<<std::endl;
		if (found!=std::string::npos){
			//std::cout<<line.substr(4)<<std::endl;
			//posnum = atoi(line.substr(2,1).c_str());
			//samplenum = atoi(line.substr(4).c_str());

			//found2 = filename.find("bigbird");
			//found3 = filename.find("clouds");
			//std::cout<<filename.substr(found2+8,found3-found2-9)<<std::endl;
			//instance_name = filename.substr(found2+8,found3-found2-9);
			//found4 = instance_name.find("_");
			//category_name = instance_name.substr(0,found4);

			found2 = line.find("_NP");
			//std::cout<<found2<<" "<<line.substr(found2+3,1).c_str()<<" "<<line.substr(found2+5,1).c_str()<<std::endl;
			posnum = atoi(line.substr(found2+3,1).c_str());
			samplenum = atoi(line.substr(found2+5).c_str());


			found3 = filename.find("amazon");
			found4 = filename.find("clouds");

			//instance_name = filename.substr(found3+7,found4-1-found3-7);
			instance_name = line.substr(0,found2);
			//found3 = instance_name.find_first_of("_");
			category_name = instance_name;


			std::cout<<"pose number: "<<posnum<<" "<<"sample number: "<<samplenum<<" "<<category_name<<" "<<instance_name<<std::endl;

			if(instance_name!=prev_instance_name){
				instance++;
				prev_instance_name = instance_name;
			}
			if(category_name!=prev_category_name){
				category++;
				prev_category_name = category_name;
			}
			if((samplenum % sampleRate) == 0)
			{
				//out<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
				if(std::find(testposnum.begin(), testposnum.end(), posnum) != testposnum.end())
					out_test<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
				else
					out_train<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
			}

		}
	}

	out_test.close();
	out_train.close();

	return 1;

}

int Reader::CovertFromListToCategory_microsoft(std::string input, int sampleRate, int testposnum, std::string testfilename, std::string trainfilename)
{
	std::fstream file;


	std::ofstream out_test(testfilename.c_str());
	std::ofstream out_train(trainfilename.c_str());

	file.open(input.c_str());

	std::string line,filename;
	int category = 0;
	int instance = 0;

	int linecount=0;

	std::string instance_name;
	std::string prev_instance_name = "";

	std::string category_name;
	std::string prev_category_name = "";

	bool testpose;
	//std::string found_pose, found_samplenum;

	int posenum;

	while(!file.eof())
	{
		testpose = 0;
		int samplenum;
		unsigned found2, found3, found4, found5;
		file>>filename;
		unsigned found1 = filename.find_last_of("/");
		line = filename.substr(found1+1);
		std::cout<<line<<std::endl;

		std::size_t found = line.find(".pcd");
		line = line.substr(0,found);
		std::cout<<line<<std::endl;
		if (found!=std::string::npos){
			unsigned t = std::count(line.begin(), line.end(), '_');
			//std::cout<<t<<std::endl;
			if(t==3)
			{
				found2 = line.find("_");
				category_name = line.substr(0,found2);
				found3 = line.find("_",found2+1);
				instance_name = line.substr(0,found3);
				found4 = line.find("_",found3+1);
				samplenum = atoi(line.substr(found3+1,found4).c_str());
				posenum = atoi(line.substr(found4+1,2).c_str());

			}
			else if (t==4)
			{
				found2 = line.find("_");
				found3 = line.find("_",found2+1);
				category_name = line.substr(0,found3);
				found4 = line.find("_",found3+1);
				instance_name = line.substr(0,found4);
				found5 = line.find("_",found4+1);
				samplenum = atoi(line.substr(found4+1,found5).c_str());
				posenum = atoi(line.substr(found5+1,2).c_str());


			}
			std::cout<<"pose number: "<<posenum<<" "<<"sample number: "<<samplenum<<" "<<category_name<<" "<<instance_name<<std::endl;

			if (!boost::iequals(instance_name, prev_instance_name)){
				//std::cout<<instance<<std::endl;
				instance++;
				//std::cout<<instance<<std::endl;
				prev_instance_name = instance_name;
			}
			if (!boost::iequals(category_name, prev_category_name)){
				category++;
				prev_category_name = category_name;
			}



			if((samplenum % sampleRate) == 0)
			{

				if(posenum == testposnum)
					out_test<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
				else
					out_train<<filename<<" "<<category<<" "<<category_name<<" "<<instance<<" "<<instance_name<<std::endl;
			}

		}
	}

	out_test.close();
	out_train.close();




	return 1;
}

int Reader::BuildMasterCategoryInstanceMap(std::string master_category_list, std::map<std::string, int> &category_map, std::map<std::string, int> &instance_map)
{
	std::string filename,instancename,categoryname;
	int categorynum,instancenum;

	std::ifstream file;
	file.open(master_category_list.c_str());


//	std::ofstream ofile("master_category_list.txt");
	int prevcategorynum = 0;
	int previnstancenum = 0;
	while(!file.eof())
	{

		file>>categoryname>>categorynum>>instancename>>instancenum;
		std::cout<<instancename<<" "<<instancenum<<std::endl;
		instance_map.insert(std::pair<std::string,int>(instancename, instancenum));
		category_map.insert(std::pair<std::string,int>(categoryname, categorynum));

//		if(previnstancenum!=instancenum){
//			ofile<<categoryname<<" "<<categorynum<<" "<<instancename<<" "<<instancenum<<std::endl;
//			previnstancenum = instancenum;
//		}



	}
	file.close();

	std::cout<<instance_map.size()<<" "<<category_map.size()<<std::endl;
	return 1;
}

int Reader::ReadBOPList(std::string pcd_list,std::vector<cv::Mat> &BOPdescriptors)
{

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;
		cv::Mat bop_descriptors;
		ReadBOP(filename,bop_descriptors);
		BOPdescriptors.push_back(bop_descriptors);
		std::cout<<"Read descriptor size: "<<bop_descriptors.cols<<" "<<bop_descriptors.rows<<std::endl;
	}
	file.close();
	return 1;
}

int Reader::ReadBOPListuchar(std::string pcd_list,std::vector<cv::Mat> &BOPdescriptors)
{

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;
		cv::Mat bop_descriptors;
		ReadBOPuchar(filename,bop_descriptors);
		BOPdescriptors.push_back(bop_descriptors);
		std::cout<<"Read descriptor size: "<<bop_descriptors.cols<<" "<<bop_descriptors.rows<<std::endl;
	}
	file.close();
	return 1;
}

int Reader::ReadBOP(std::string pcd_file,cv::Mat &BOPdescriptors)
{
	std::ifstream file;
	file.open(pcd_file.c_str(), std::ifstream::in);
	int width, height;
	float val;
	file >> width;
	file >> height;
	//std::cout<<"Width: "<<width<<" ; Height: "<<height<<std::endl;
	//BOPdescriptors = cv::Mat(height,width,CV_8U);
	BOPdescriptors = cv::Mat(height,width,CV_32F);
	//std::cout<<"Width: "<<BOPdescriptors.cols<<" ; Height: "<<BOPdescriptors.rows<<std::endl;
	for(uint i=0;i<height;i++)
	{
		for(uint j=0;j<width;j++)
		{
			file >> val;
			//std::cout<<i<<" "<<j<<" "<<val<<std::endl;
			BOPdescriptors.at<float>(i,j) = val;
		}
	}

	file.close();
	file.clear();

	//std::cout<<"read end!"<<std::endl;
	//std::cout<<"Number of values: "<<values.size()<<std::endl;
	return 1;
}

int Reader::ReadBOPuchar(std::string pcd_file,cv::Mat &BOPdescriptors)
{
	std::ifstream file;
	file.open(pcd_file.c_str(), std::ifstream::in);
	int width, height;
	uchar val;
	file >> width;
	file >> height;
	//std::cout<<"Width: "<<width<<" ; Height: "<<height<<std::endl;
	BOPdescriptors = cv::Mat(height,width,CV_8U);
	//BOPdescriptors = cv::Mat(height,width,CV_32F);
	//std::cout<<"Width: "<<BOPdescriptors.cols<<" ; Height: "<<BOPdescriptors.rows<<std::endl;
	for(uint i=0;i<height;i++)
	{
		for(uint j=0;j<width;j++)
		{
			file >> val;
			//std::cout<<i<<" "<<j<<" "<<val<<std::endl;
			BOPdescriptors.at<uchar>(i,j) = val;
		}
	}

	file.close();
	file.clear();

	//std::cout<<"read end!"<<std::endl;
	//std::cout<<"Number of values: "<<values.size()<<std::endl;
	return 1;
}

int Reader::ReadBOPfast(std::string pcd_file,cv::Mat &BOPdescriptors)
{
	std::ifstream file;
	file.open(pcd_file.c_str(), std::ifstream::in);
	int width, height;
	uchar val;
	file >> width;
	file >> height;

	//from binary to uchar
	width = width/8;

	//std::cout<<"Width: "<<width<<" ; Height: "<<height<<std::endl;
	BOPdescriptors = cv::Mat(height,width,CV_8U);
	//std::cout<<"Width: "<<BOPdescriptors.cols<<" ; Height: "<<BOPdescriptors.rows<<std::endl;
	std::vector<uchar> values;
	int row = 0;
	int col = 0;
	/*
	while(file>>val){
			//std::cout<<row<<" "<<col<<std::endl;
			//std::cout<<val<<std::endl;
			//BOPdescriptors.at<int>(row,col) = val;
			values.push_back(val);
	}
	file.close();
	file.clear();

	for(uint i=0;i<height;i++)
	{
		for(uint j=0;j<width;j++)
		{
			std::cout<<i<<" "<<j<<std::endl;
			std::cout<<values[i*width + j]<<std::endl;
			BOPdescriptors.at<uchar>(i,j) = values[i*width + j];
		}
	}
	 */
	for(uint i=0;i<height;i++)
	{
		for(uint j=0;j<width;j++)
		{
			uchar val3;
			for(uint k=0;k<8;k++){
				file >> val;
				uchar val2;
				val2 =  val;
				val2 << 7-k;
				val3 = val3 + val2;
			}
			//std::cout<<i<<" "<<j<<std::endl;
			BOPdescriptors.at<uchar>(i,j) = val3;
		}
	}

	file.close();
	file.clear();

	//std::cout<<"read end!"<<std::endl;
	//std::cout<<"Number of values: "<<values.size()<<std::endl;
	return 1;
}

int Reader::ReadSPFH(std::string pcd_file,cv::Mat &SPFHdescriptors)
{
	std::ifstream file;
	file.open(pcd_file.c_str(), std::ifstream::in);
	int width, height;
	float val;
	file >> width;
	file >> height;
	std::cout<<"Width: "<<width<<" ; Height: "<<height<<std::endl;
	SPFHdescriptors = cv::Mat(height,width,CV_32F);
	//std::cout<<"Width: "<<BOPdescriptors.cols<<" ; Height: "<<BOPdescriptors.rows<<std::endl;
	std::vector<uchar> values;


	for(uint i=0;i<height;i++)
	{
		for(uint j=0;j<width;j++)
		{
			file >> val;
			//std::cout<<i<<" "<<j<<" "<<val<<std::endl;
			SPFHdescriptors.at<float>(i,j) = val;
			//std::cout<<i<<" "<<j<<" "<<SPFHdescriptors.at<float>(i,j)<<std::endl;
		}
	}

	file.close();
	file.clear();

	//std::cout<<"read end!"<<std::endl;
	//std::cout<<"Number of values: "<<values.size()<<std::endl;
	return 1;
}

int Reader::ReadSPFHList(std::string pcd_list,std::vector<cv::Mat> &SPFHdescriptors)
{

	std::ifstream file;
	file.open(pcd_list.c_str());
	std::string filename;
	while(!file.eof())
	{
		file>>filename;
		std::cout<<"Reading: "<<filename<<std::endl;
		cv::Mat spfh_descriptors;
		ReadSPFH(filename,spfh_descriptors);
		SPFHdescriptors.push_back(spfh_descriptors);
		std::cout<<"Read descriptor size: "<<spfh_descriptors.cols<<" "<<spfh_descriptors.rows<<std::endl;
	}
	file.close();
	return 1;
}

int Reader::ReadShrecInstanceNumbers(std::string list, std::vector<int> &instance_numbers)
{
	std::ifstream file;
		file.open(list.c_str());
		std::string filename, line;
		unsigned found1, found2, found3;
		int instance_num;
		while(!file.eof())
		{
			file>>filename;
			found1 = filename.find_last_of("/");
			line = filename.substr(found1+1);
			found2 = line.find("_");
			instance_num = atoi(line.substr(0,found2).c_str());
			std::cout<<instance_num<<std::endl;
			instance_numbers.push_back(instance_num);


		}
		file.close();


	return 1;
}
