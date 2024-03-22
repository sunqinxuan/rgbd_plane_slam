/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-23 09:33
#
# Filename:		CameraPoseEstimation_PlaneParas.cpp
#
# Description: 
#
===============================================*/
#include "CameraPoseEstimation_PlaneParas.h"

namespace sun
{
	CameraPoseEstimation_PlaneParas::CameraPoseEstimation_PlaneParas()
	{
		pose_align_planes.Rotation=Eigen::Matrix3d::Identity();
		pose_align_planes.position=Eigen::Vector3d::Zero();
	}

	CameraPoseEstimation_PlaneParas::~CameraPoseEstimation_PlaneParas()
	{
	}

	void CameraPoseEstimation_PlaneParas::Pose2AlignPlanes()
	{
		static const int N=matched_planes.size();
		ConstraintCase tmp_case=constraint_case();
		Eigen::Matrix3d tmp_mat3d,tmp_inverse;
		bool invertible;
		pose_align_planes.Rotation=H_svd_U*H_svd_V.transpose();
		Eigen::MatrixXd A;
		Eigen::VectorXd d;
		if(tmp_case==DoF_6)
		{
			//pose_align_planes.Rotation=H_svd_U*H_svd_V.transpose();
			A=Eigen::MatrixXd::Zero(N,3);
			d=Eigen::VectorXd::Zero(N);
			for(int i=0;i<N;i++)
			{
				A.block<1,3>(i,0)=matched_planes[i].ref->normal.transpose();
				d(i)=matched_planes[i].cur->d-matched_planes[i].ref->d;
			}
			//tmp_mat3d=A.transpose()*A;
			//tmp_mat3d.computeInverseWithCheck(tmp_inverse,invertible);
			//if(invertible)
			//	pose_align_planes.position=tmp_inverse*A.transpose()*d;
			//else
			//{
			//	std::cerr<<"matrix A uninvertible!"<<std::endl;
			//	return;
			//}
			////Eigen::Matrix3d tmp_mat3d=A.transpose()*A;
			////tmp_mat3d=tmp_mat3d.inverse();
			////pose_align_planes.position=tmp_mat3d*A.transpose()*d;
			//if(debug)
			//{
			//	std::cout<<"DoF_6"<<std::endl;
			//	std::cout<<"A:"<<std::endl<<A<<std::endl;
			//	std::cout<<"d:"<<d.transpose()<<std::endl;
			//}
		}
		else if(tmp_case==DoF_5)
		{
			//pose_align_planes.Rotation=H_svd_U*H_svd_V.transpose();
			if(abs(pose_align_planes.Rotation.determinant()+1.0f)<1.0e-4)
			{
				H_svd_U.block<3,1>(0,2)=-H_svd_U.block<3,1>(0,2);
				pose_align_planes.Rotation=H_svd_U*H_svd_V.transpose();
				if(debug)
				{
					std::cout<<"U':"<<std::endl<<H_svd_U<<std::endl;
					std::cout<<"det(R'):"<<pose_align_planes.Rotation.determinant()<<std::endl;
				}
			}
			else
			{
				if(debug)
				{
					std::cout<<"U:"<<std::endl<<H_svd_U<<std::endl;
					std::cout<<"det(R):"<<pose_align_planes.Rotation.determinant()<<std::endl;
				}
			}
			A=Eigen::MatrixXd::Zero(N+1,3);
			d=Eigen::VectorXd::Zero(N+1);
			for(int i=0;i<N;i++)
			{
				//if(i==1) continue;
				A.block<1,3>(i,0)=matched_planes[i].ref->normal.transpose();
				d(i)=matched_planes[i].cur->d-matched_planes[i].ref->d;
			}
			A.block<1,3>(N,0)=H_svd_V.block<3,1>(0,2).transpose();
			//tmp_mat3d=A.transpose()*A;
			//tmp_mat3d.computeInverseWithCheck(tmp_inverse,invertible);
			//if(invertible)
			//	pose_align_planes.position=tmp_inverse*A.transpose()*d;
			//else
			//{
			//	std::cerr<<"matrix A uninvertible!"<<std::endl;
			//	return;
			//}
			//if(debug)
			//{
			//	std::cout<<"DoF_5"<<std::endl;
			//	std::cout<<"A"<<std::endl<<A<<std::endl;
			//	std::cout<<"d:"<<d.transpose()<<std::endl;
			//	//Eigen::Vector3d eular1, eular2;
			//	//Rotation2EularAngles(pose_align_planes.Rotation,eular1);
			//	//eular2=pose_align_planes.Rotation.eulerAngles(2,1,0);
			//	//std::cout<<"eular1:"<<eular1.transpose()<<std::endl;
			//	//std::cout<<"eular2:"<<eular2.transpose()<<std::endl;
			//	//Eigen::Quaterniond quaternion=Eigen::Quaterniond(pose_align_planes.Rotation);
			//	//std::cout<<"quaternion:"<<quaternion.w()<<","<<quaternion.vec().transpose()<<std::endl;
			//}
		}
		else if(tmp_case==DoF_3)
		{
			Eigen::Matrix3d H1, H_svd_U1, H_svd_V1;
			H1=H+H_svd_U.block<3,1>(0,2)*H_svd_V.block<3,1>(0,2).transpose();
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(H1, Eigen::ComputeFullU | Eigen::ComputeFullV);
			H_svd_U1=svd.matrixU();
			H_svd_V1=svd.matrixV();
			pose_align_planes.Rotation=H_svd_U1*H_svd_V1.transpose();
			if(abs(pose_align_planes.Rotation.determinant()+1.0f)<1.0e-4)
			{
				H_svd_U1.block<3,1>(0,2)=-H_svd_U1.block<3,1>(0,2);
				pose_align_planes.Rotation=H_svd_U1*H_svd_V1.transpose();
				if(debug)
				{
					std::cout<<"U1':"<<std::endl<<H_svd_U1<<std::endl;
					std::cout<<"det(R'):"<<pose_align_planes.Rotation.determinant()<<std::endl;
				}
			}
			else
			{
				if(debug)
				{
					std::cout<<"U1:"<<std::endl<<H_svd_U1<<std::endl;
					std::cout<<"det(R):"<<pose_align_planes.Rotation.determinant()<<std::endl;
				}
			}
			A=Eigen::MatrixXd::Zero(N+2,3);
			d=Eigen::VectorXd::Zero(N+2);
			for(int i=0;i<N;i++)
			{
				A.block<1,3>(i,0)=matched_planes[i].ref->normal.transpose();
				d(i)=matched_planes[i].ref->d-matched_planes[i].cur->d;
			}
			A.block<1,3>(N,0)=H_svd_V.block<3,1>(0,1).transpose();
			A.block<1,3>(N+1,0)=H_svd_V.block<3,1>(0,2).transpose();
			//tmp_mat3d=A.transpose()*A;
			//tmp_mat3d.computeInverseWithCheck(tmp_inverse,invertible);
			//if(invertible)
			//	pose_align_planes.position=tmp_inverse*A.transpose()*d;
			//else
			//{
			//	std::cerr<<"matrix A uninvertible!"<<std::endl;
			//	return;
			//}
			//if(debug)
			//{
			//	std::cout<<"DoF_5"<<std::endl;
			//	std::cout<<"A"<<std::endl<<A<<std::endl;
			//	std::cout<<"d:"<<d.transpose()<<std::endl;
			//}
		}
		tmp_mat3d=A.transpose()*A;
		tmp_mat3d.computeInverseWithCheck(tmp_inverse,invertible);
		if(invertible)
			pose_align_planes.position=tmp_inverse*A.transpose()*d;
		else
		{
			std::cerr<<"matrix A uninvertible!"<<std::endl;
			return;
		}
		if(debug)
		{
			std::cout<<"constrain case:"<<tmp_case<<std::endl;
			std::cout<<"A"<<std::endl<<A<<std::endl;
			std::cout<<"d:"<<d.transpose()<<std::endl;
			std::cout<<"camera pose:"<<std::endl;
			std::cout<<"Rotation:"<<std::endl<<pose_align_planes.Rotation<<std::endl;
			std::cout<<"translation:"<<pose_align_planes.position.transpose()<<std::endl;
			std::cout<<"eular angle:"<<pose_align_planes.Eulars().transpose()<<std::endl;
			std::cout<<"quatenion:"<<pose_align_planes.Quat().w()<<","<<pose_align_planes.Quat().vec().transpose()<<std::endl;
		}
	}


}
