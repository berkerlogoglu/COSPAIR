/*
 * myshapegenerator.cpp
 *
 *  Created on: Sep 6, 2014
 *      Author: berker
 */

#include "myshapegenerator.h"

my_shape_generator::my_shape_generator() {
	// TODO Auto-generated constructor stub

}

my_shape_generator::~my_shape_generator() {
	// TODO Auto-generated destructor stub
}
static const float PI = boost::math::constants::pi<float> ();

int  my_shape_generator::calculateNumberOfPoints (const float area, const float resolution)
{
  float num_points_float = area * resolution;
  int num_points;
  float frac;
  frac = boost::math::modf (num_points_float , &num_points);
  if (dist01_ (gen_) < frac)
    num_points++;
  return num_points;
}

void my_shape_generator::generate_sphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius, float resolution)
{
	  pcl::PointCloud <pcl::PointXYZRGB> temp_cloud;

	  float area = 4 * PI * pow (radius, 2);
	  int num_points = calculateNumberOfPoints (area, resolution);
	  while (temp_cloud.size () < num_points)
	  {
	    float x (-1 + 2 * dist01_ (gen_)), y (-1 + 2 * dist01_ (gen_)), z (-1 + 2 * dist01_ (gen_));
	    if (x * x + y * y + z * z <= 1)
	    {
	      float theta = atan2 (sqrt (x * x + y * y), z);
	      float phi = atan2 (y, x);
	      pcl::PointXYZRGB p = pcl::PointXYZRGB ();
	      p.x = radius * cos (phi) * sin (theta);
	      p.y = radius * sin (phi) * sin (theta);
	      p.z = radius * cos (theta);
	      temp_cloud.push_back (p);
	    }
	  }
	  pcl::copyPointCloud (temp_cloud, *cloud);

}
