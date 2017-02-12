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
#include <stdlib.h>


//PCL
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>



//OPENCV
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/ml/ml.hpp>



int main (int argc, char* argv[])
{


	std::time_t start = std::time(NULL);
	if(argc == 9)
	{
		std::string file;
		double R,sampling;
		int levels,bins,rgb_bins,keypointType;
		int rgb_type;
		file = argv[1];
		R = atof(argv[2]);
		levels = atoi(argv[3]);
		bins = atoi(argv[4]);
		rgb_type = atoi(argv[5]);
		rgb_bins = atoi(argv[6]);
		keypointType = atoi(argv[7]);
		sampling = atof(argv[8]);
		FeatureExtractor fe;
		fe.ExtractCOSPAIR(file,R,levels,bins,rgb_type,rgb_bins,keypointType, sampling);


	}
	else
	{
		std::cout<<"Missing or wrong number of argument(s)! "<<std::endl;
		std::cout<<"Arguments are: "<<"File name ; Support Radius ; # of Levels ; # of histogram bins for each feat. ; RGB type?; # of RGB histogram bins; Keypoint Type? (0=uniform,1=h3d,2=iss) ; sampling? "<<std::endl;
	}

	std::cout<<"Time Elapsed: "<<std::difftime(std::time(NULL), start)<<" seconds!"<<std::endl;

  return 1;
}
