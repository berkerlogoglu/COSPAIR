/*
 * myshapegenerator.h
 *
 *  Created on: Sep 6, 2014
 *      Author: berker
 */

#ifndef MYSHAPEGENERATOR_H_
#define MYSHAPEGENERATOR_H_


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>

#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/random.hpp>
#include <boost/array.hpp>
#include <boost/assign.hpp>
#include <boost/math/special_functions/modf.hpp>

class my_shape_generator {
public:
	my_shape_generator();
	virtual ~my_shape_generator();

	int  calculateNumberOfPoints (const float area, const float resolution);
	void generate_sphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius, float resolution);

private:
    /** \brief Random number generator*/
    boost::random::mt19937 gen_;

    /** \brief Uniform distribution needed for most samplings*/
    boost::random::uniform_real_distribution<float> dist01_;
};

#endif /* MYSHAPEGENERATOR_H_ */
