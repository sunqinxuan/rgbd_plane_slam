/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-07-17 16:41
#
# Filename:		PlaneExtraction.cpp
#
# Description: base class for all the plane extraction methods.
#
===============================================*/

#include "PlaneExtraction.h"

namespace sun
{
	void Cells_bottom::Initialize()
	{
		cells.resize(bins_theta*bins_phy*bins_d);
		delta_theta=M_PI/bins_theta;
		delta_phy=M_PI*2/bins_phy;
		delta_d=6.0/bins_d;
		for(int i_d=0;i_d<bins_d;i_d++)
		{
			for(int i_theta=0;i_theta<bins_theta;i_theta++)
			{
				for(int i_phy=0;i_phy<bins_phy;i_phy++)
				{
					int i=index(i_d,i_theta,i_phy);
					cells[i].isEmpty=true;
					cells[i].isBottom=true;
					cells[i].avg_pps.setZero(3);
					cells[i].avg_rgb.setZero(3);
					cells[i].cov_pps.setZero(3,3);
					cells[i].cov_rgb.setZero(3,3);
					cells[i].num_points=0;
					cells[i].inliers = boost::make_shared<pcl::PointIndices>();
				}
			}
		}
	}

	void Cells_bottom::push_point(Point point_tmp, int inlier)
	{
		int theta=(point_tmp.pps[0])/delta_theta;
		int phy=(point_tmp.pps[1]+M_PI)/delta_phy;
		int d=(point_tmp.pps[2])/delta_d;
		if(theta>=bins_theta)
			theta=bins_theta-1;
		if(phy>=bins_phy)
			phy=bins_phy-1;
		if(d>=bins_d)
			d=bins_d-1;
		int i=index(d,theta,phy);
		cells[i].points_in.push_back(point_tmp);
		cells[i].inliers->indices.push_back(inlier);
		cells[i].isEmpty=false;
	}

	void Cells_bottom::ComputeCellAttribute(Eigen::Matrix3d Rotation2eigen)
	{
		Eigen::Vector3d tmp;
		for(int i=0;i<cells.size();i++)
		{
			if(cells[i].isEmpty)
				continue;
			cells[i].num_points=cells[i].points_in.size();
			// compute the expectation of position and color;
			for(std::vector<Point>::iterator it=cells[i].points_in.begin();it!=cells[i].points_in.end();++it)
			{
				cells[i].avg_pps=cells[i].avg_pps+it->pps;
				cells[i].avg_rgb=cells[i].avg_rgb+it->rgb;
			}
			cells[i].avg_pps=cells[i].avg_pps*(1.0/cells[i].num_points);
			cells[i].avg_rgb=cells[i].avg_rgb*(1.0/cells[i].num_points);
			// compute the covariance of position and color;
			for(std::vector<Point>::iterator it=cells[i].points_in.begin();it!=cells[i].points_in.end();++it)
			{
				tmp=it->pps;
				tmp-=cells[i].avg_pps;
				cells[i].cov_pps+=tmp*tmp.transpose();
				tmp=it->rgb;
				tmp-=cells[i].avg_rgb;
				cells[i].cov_rgb+=tmp*tmp.transpose();
			}
			// biased estimation of the covariance;
			cells[i].cov_pps=cells[i].cov_pps*(1.0/cells[i].num_points);
			cells[i].cov_rgb=cells[i].cov_rgb*(1.0/cells[i].num_points);
			// normal of the cells[i] in the original coordinate;
			tmp(0)=sin(cells[i].avg_pps(0))*cos(cells[i].avg_pps(1));
			tmp(1)=sin(cells[i].avg_pps(0))*sin(cells[i].avg_pps(1));
			tmp(2)=cos(cells[i].avg_pps(0));
			cells[i].normal=Rotation2eigen.transpose()*tmp;
			cells[i].d=cells[i].avg_pps(2);
		}
	}

	void Cells_bottom::SortCells()
	{
		Sorted_Cell tmp_sort;
		for(int i=0;i<cells.size();i++)
		{
			tmp_sort.index=i;
			tmp_sort.num_point=cells[i].num_points;
			sorted_cells.push_back(tmp_sort);
		}
		std::sort(sorted_cells.begin(), sorted_cells.end());
	}

	bool PlaneExtraction::LoadPoints(Scan *scan)
	{
		if(debug)
		{
			std::cout<<std::endl<<"LoadPoints:"<<std::endl;
			std::cout<<"\tthere are "<<scan->point_cloud->size()<<" points in input point point_cloud."<<std::endl;
		}
		if(scan->point_cloud->empty() || scan->normal_cloud->empty() || scan->pixel_cloud->empty())
		{
			std::cerr<<"load point_cloud, normal_cloud and pixel_cloud before this."<<std::endl;
			return false;
		}

		cells_bottom=new Cells_bottom(bins_theta,bins_phy,bins_d);

		Eigen::Vector3d tmp_vec3d;
		Eigen::Matrix3d tmp_mat3d=Eigen::Matrix3d::Zero();
		int tmp_count=0;
		double tmp_max=0,tmp_min=99999;
		Eigen::Vector3d x,y,z;

		// find the direction with least normal vectors;
		// set this direction as the z axis;
		for(int i=0;i<scan->normal_cloud->size();i++)
		{
			if(std::isnan(scan->normal_cloud->at(i).normal_x) && std::isnan(scan->normal_cloud->at(i).normal_y) && std::isnan(scan->normal_cloud->at(i).normal_z))
				continue;
			else
			{
				tmp_vec3d(0)=scan->normal_cloud->at(i).normal_x;
				tmp_vec3d(1)=scan->normal_cloud->at(i).normal_y;
				tmp_vec3d(2)=scan->normal_cloud->at(i).normal_z;
				tmp_mat3d+=tmp_vec3d*tmp_vec3d.transpose();
				tmp_count++;
			}
		}
		tmp_mat3d=tmp_mat3d*(1.0/tmp_count);
		Eigen::EigenSolver<Eigen::Matrix3d> es(tmp_mat3d);
		if(debug)
		{
			std::cout<<"computate Rotation2eigen:"<<std::endl;
			std::cout<<"the scatter matrix of normals:"<<std::endl;
			std::cout<<""<<tmp_mat3d<<std::endl;
			std::cout<<"the eigen values:"<<es.eigenvalues().transpose()<<std::endl;
		}
		for(int i=0;i<3;i++)
		{
			if(es.eigenvalues().real()[i]>tmp_max)
			{
				tmp_max=es.eigenvalues().real()[i];
				y=es.eigenvectors().real().block<3,1>(0,i);
			}
			if(es.eigenvalues().real()[i]<tmp_min)
			{
				tmp_min=es.eigenvalues().real()[i];
				z=es.eigenvectors().real().block<3,1>(0,i);
			}
		}
		x=y.cross(z);
		x.normalize();
		// Rotation2eigen
		// - rotation from the original coordinate to the coordinate where 
		// - the z axis pointing to the direction with least normals;
		Rotation2eigen.block<1,3>(0,0)=x.transpose();
		Rotation2eigen.block<1,3>(1,0)=y.transpose();
		Rotation2eigen.block<1,3>(2,0)=z.transpose();
		if(debug)
		{
			std::cout<<"Rotation2eigen:"<<std::endl;
			std::cout<<Rotation2eigen<<std::endl;
		}

//		points.clear();
		Point point_tmp;
		for(int i=0;i<scan->point_cloud->size();i++)
		{
			if(std::isnan(scan->normal_cloud->at(i).normal_x) && std::isnan(scan->normal_cloud->at(i).normal_y) && std::isnan(scan->normal_cloud->at(i).normal_z))
				continue;
			else
			{
				// coordinate in RGB [0,1];
				point_tmp.rgb[0]=(double)scan->point_cloud->at(i).r/255.0;
				point_tmp.rgb[1]=(double)scan->point_cloud->at(i).g/255.0;
				point_tmp.rgb[2]=(double)scan->point_cloud->at(i).b/255.0;
				// coordinate in the camera coordinate system;
				point_tmp.xyz[0]=scan->point_cloud->at(i).x;
				point_tmp.xyz[1]=scan->point_cloud->at(i).y;
				point_tmp.xyz[2]=scan->point_cloud->at(i).z;
				// local plane normal;
				point_tmp.normal[0]=scan->normal_cloud->at(i).normal_x;
				point_tmp.normal[1]=scan->normal_cloud->at(i).normal_y;
				point_tmp.normal[2]=scan->normal_cloud->at(i).normal_z;
				// coordinate in the PPS, after Rotation2eigen; 
				point_tmp.pps = Cartesian2PPS(point_tmp.normal,point_tmp.xyz,Rotation2eigen);
				// pixel coordinate;
				point_tmp.u=scan->pixel_cloud->at(i).x; // i/640;
				point_tmp.v=scan->pixel_cloud->at(i).y; // i%640;
				// push the point into the points;
//				points.push_back(point_tmp);
				// push the point into the corresponding cells;
				cells_bottom->push_point(point_tmp,i);
			}
		}
		cells_bottom->ComputeCellAttribute(Rotation2eigen);
		cells_bottom->SortCells();

		return true;
	}
	
	// Cartesian2PPS
	// - transform from the Cartesian space to the PPS;
	// - rotate the normal via Rotation2eigen if defined;
	Eigen::Vector3d PlaneExtraction::Cartesian2PPS(Eigen::Vector3d normal, Eigen::Vector3d point, Eigen::Matrix3d Rotation2eigen)
	{
		Eigen::Vector3d tmp_vec3d, pps;
		tmp_vec3d=Rotation2eigen*normal;
		pps(0)=acos(tmp_vec3d(2));
		if(pps(0)>M_PI)
		{
			pps(0)=M_PI*2-pps(0);
		}
		pps(1)=atan2(tmp_vec3d(1),tmp_vec3d(0));
		pps(2)=-normal.dot(point);
		return pps;
	}

	// computePlaneParas
	// - compute normal and d from points_in;
	// - using the least square method;
	void PlaneExtraction::computePlaneParas(Plane *plane)
	{
		// z=(-nx/nz)x+(-ny/nz)y+(-d/nz);
		// Z=[z1,z2,...,zn]^T;
		// phi_i=[x_i,y_i,1]^T;
		// Phi=[phi_1,phi_2,...,phi_n]^T;
		// theta=[(-nx/nz),(-ny/nz),(-d/nz)];
		// theta_hat=inv(Phi^T*Phi)*Phi^T*Z;
		Eigen::Matrix3d Phi=Eigen::Matrix3d::Zero();
		Eigen::Matrix3d M;
		Eigen::Vector3d Z=Eigen::Vector3d::Zero();
		Eigen::Vector3d tmp;
		bool invertible;
		std::vector<Point>::iterator it;
		for(it=plane->points_in.begin();it!=plane->points_in.end();++it)
		{
			tmp(0)=it->xyz(0);
			tmp(1)=it->xyz(1);
			tmp(2)=1;
			// Phi <- Phi^T*Phi;
			Phi=Phi+tmp*tmp.transpose();
			// Z <- Phi^T*Z;
			Z=Z+tmp*it->xyz(2);
		}
		Phi.computeInverseWithCheck(M,invertible);
		if(invertible)
		{
			tmp=M*Z;
		}
		else
		{
			std::cerr<<"compute plane paras error!"<<std::endl;
			return;
		}
		Eigen::Vector3d n;
		double d;
		// tmp = [(-nx/nz)x, (-ny/nz)y, (-d/nz)];
		n(0)=-tmp(0);
		n(1)=-tmp(1);
		n(2)=1;
		double scale=n.norm();
		n=n/scale;
		d=-tmp(2)/scale;
		it=plane->points_in.begin();
		if(it->xyz.transpose()*n>0)
		{
			n=-n;
			d=-d;
		}
		plane->normal=n;
		plane->d=d;
	}

	//Eigen::Vector3d PlaneExtraction::pps2normal(Eigen::Vector3d pps)
	//{
	//	Eigen::Vector3d normal;
	//	normal(0)=sin(pps(0))*cos(pps(1));
	//	normal(1)=sin(pps(0))*sin(pps(1));
	//	normal(2)=cos(pps(0));
	//	normal=Rotation2eigen.transpose()*normal;
	//	return normal;
	//}
}
