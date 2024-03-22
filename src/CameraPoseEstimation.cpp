/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-03-23 09:42
#
# Filename: CameraPoseEstimation.cpp
#
# Description: 
#
===============================================*/
#include "CameraPoseEstimation.h"

namespace sun
{
	CameraPoseEstimation::CameraPoseEstimation()
	{
		pose_align_scans.Rotation=Eigen::Matrix3d::Identity();
		pose_align_scans.position=Eigen::Vector3d::Zero();
	}

	CameraPoseEstimation::~CameraPoseEstimation()
	{
	}
	
	CameraPoseEstimation::ConstraintCase CameraPoseEstimation::constraint_case()
	{
		Eigen::Vector3d H_singularValues;
		compute_H();
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
		H_svd_U=svd.matrixU();
		H_svd_V=svd.matrixV();
		H_singularValues=svd.singularValues();
		if(debug)
		{
			std::cout<<"svd of H: "<<H_singularValues.transpose()<<std::endl;
			std::cout<<H_svd_U<<std::endl;
			std::cout<<H_svd_V<<std::endl;
		}
		if(H_singularValues(0)>1000*H_singularValues(1))
			return DoF_3;
		else if(H_singularValues(1)>1000*H_singularValues(2))
			return DoF_5;
		else
			return DoF_6;
	}

	void CameraPoseEstimation::compute_H()
	{
		H.setZero();
		for(int i=0;i<matched_planes.size();i++)
		{
			H=H+matched_planes[i].cur->normal*matched_planes[i].ref->normal.transpose();
		}
	}

}
