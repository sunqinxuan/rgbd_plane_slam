/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-09 09:44
#
# Filename:		PlaneMatching.h
#
# Description: 
#
===============================================*/
#pragma once
#include <stdio.h>
#include <fstream>
#include <vector>
#include <list>
#include <queue>
#include <stack>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include "Basics.h"

using namespace sun;

namespace sun
{
	class PlaneMatching
	{
	public:
		PlaneMatching();
		~PlaneMatching();
		void setDebug(bool d) {debug=d;}
		void loadPlanes(std::vector<Plane*> planes_ref,std::vector<Plane*> planes_cur);
		virtual void Match(std::vector<PlanePair> &matched_planes)=0;
	protected:
		bool debug;
		std::vector<Plane*> planes_ref;
		std::vector<Plane*> planes_cur;
	};
}
