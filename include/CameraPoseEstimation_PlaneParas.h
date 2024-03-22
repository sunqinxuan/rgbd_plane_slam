/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-23 09:33
#
# Filename:		CameraPoseEstimation_PlaneParas.h
#
# Description: 
#
===============================================*/
#pragma once
#include "CameraPoseEstimation.h"

namespace sun
{
	class CameraPoseEstimation_PlaneParas : public  CameraPoseEstimation
	{
	public:
		CameraPoseEstimation_PlaneParas();
		~CameraPoseEstimation_PlaneParas();
		CameraPose getPoseAlignPlanes() {return pose_align_planes;}
		void Pose2AlignPlanes();

	private:
		CameraPose pose_align_planes;
	};
}
