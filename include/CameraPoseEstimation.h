/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-03-23 09:37
#
# Filename: CameraPoseEstimation.h
#
# Description: 
#
===============================================*/
#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>
#include "Basics.h"

using namespace sun;

namespace sun
{
	class CameraPoseEstimation
	{
	public:
		enum ConstraintCase { DoF_6, DoF_5, DoF_3 };
		CameraPoseEstimation();
		~CameraPoseEstimation();
		void setDebug(bool d) {debug=d;}
		CameraPose getCameraPose() {return pose_align_scans;}
		void loadMatchedPlanes(std::vector<PlanePair> &mp) {matched_planes=mp;}

	protected:
		std::vector<PlanePair> matched_planes;
		Eigen::Matrix3d H, H_svd_U, H_svd_V;
		CameraPose pose_align_scans;
		bool debug;
		void compute_H();
		ConstraintCase constraint_case();
	};
}
