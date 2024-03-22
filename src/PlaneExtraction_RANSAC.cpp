/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-04-09 14:34
#
# Filename:		PlaneExtraction_RANSAC.cpp
#
# Description: 
#
===============================================*/

#include "PlaneExtraction_RANSAC.h"

namespace sun
{
	PlaneExtraction_RANSAC::PlaneExtraction_RANSAC()
	{
		maxdist_point2plane=0.1;
		max_plane=99;
		min_plane_size=7000;
		bins_theta=5;
		bins_phy=10;
		bins_d=1;
		thres_angle=0.2; // 0.2rad~=11.5deg
		thres_dist=0.1; // 10cm
		thres_color=0.2;
	}

	// ExtractPlanes
	// - extract planes from the scan;
	void PlaneExtraction_RANSAC::ExtractPlanes()
	{
		pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
		bool enough_plane = false;
		Eigen::Vector3d tmp_vec3d;
		pcl::PointXYZRGBA tmp_point_pcl;
		pcl::Normal tmp_normal_pcl;
		Plane *tmp_plane;
		Point tmp_point;
		bool have_same_plane=false;
		bool had_parallel_plane=false;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr allplane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_contain_plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_contain_plane_normal (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_contain_plane_image (new pcl::PointCloud<pcl::PointXY>);
		
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::Normal>::Ptr plane_normal (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXY>::Ptr plane_image (new pcl::PointCloud<pcl::PointXY>);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rest (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rest_final (new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normal;
		pcl::ExtractIndices<pcl::PointXY> extract_image;

		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(120);
		seg.setDistanceThreshold(0.01);

		// find the cell with the most points inside;
		std::vector<Sorted_Cell>::iterator iter_sorted_cells=cells_bottom->getHighestCell();
		int maxdir = 0;
		int maxnum = 0;
		maxdir=iter_sorted_cells->index;
		maxnum=iter_sorted_cells->num_point;

		cloud_rest_final->clear();
		scan->planes.clear();
		// extracting planes;
		while ( maxnum > min_plane_size && enough_plane == false )
		{
			// save points in cells[maxdir] to cloud_contain_plane;
			extract.setInputCloud (scan->point_cloud);
			extract.setIndices (cells_bottom->getCell(maxdir)->inliers);
			extract.setNegative (false);
			extract.filter (*cloud_contain_plane);
			// the corresponding normals to cloud_contain_plane_normal;
			extract_normal.setInputCloud(scan->normal_cloud);
			extract_normal.setIndices(cells_bottom->getCell(maxdir)->inliers);
			extract_normal.setNegative(false);
			extract_normal.filter(*cloud_contain_plane_normal);
			// the corresponding pixel_cloud pixel to cloud_contain_plane_image;
			extract_image.setInputCloud (scan->pixel_cloud);
			extract_image.setIndices (cells_bottom->getCell(maxdir)->inliers);
			extract_image.setNegative (false);
			extract_image.filter (*cloud_contain_plane_image);
			// enough points in the cells[maxdir];
			if(debug)
			{
				std::cout<<"ExtractPlanes:"<<std::endl;
				std::cout<<"\tmaxdir="<<maxdir<<", maxnum="<<maxnum<<std::endl;
				std::cout<<"\tcloud_contain_plane in the maxdir:"<<cloud_contain_plane->size()<<std::endl;
			}
			if ( cloud_contain_plane->size() > min_plane_size)
			{
				// estimate plane parameters using cells[maxdir];
				seg.setInputCloud(cloud_contain_plane);
				seg.segment(*inliers_plane, *coefficients_plane);
				// plane equation: ax+by+cz+d=0;
				// make the plane normal point to the origin;
				unifyPlaneDir(coefficients_plane);

				// expand cloud_contain_plane from cells near cell[maxdir];
					//for (int i_cell = 0;i_cell<8;i_cell++)  
					//{
					//	int tmp;
					//	switch(i_cell)
					//	{
					//	case 0:
					//		tmp=maxdir-1;
					//		break;
					//	case 1:
					//		tmp=maxdir+1;
					//		break;
					//	case 2:
					//		tmp=maxdir-bins_theta;
					//		break;
					//	case 3:
					//		tmp=maxdir-bins_theta-1;
					//		break;
					//	case 4:
					//		tmp=maxdir-bins_theta+1;
					//		break;
					//	case 5:
					//		tmp=maxdir+bins_theta;
					//		break;
					//	case 6:
					//		tmp=maxdir+bins_theta-1;
					//		break;
					//	case 7:
					//		tmp=maxdir+bins_theta+1;
					//		break;
					//	default:
					//		break;
					//	}
					//	if(tmp>=0 && tmp<bins_theta*bins_phy) //bins_d=1
					//	{
					//		for(int i=0;i<cells[tmp].num_points;i++)
					//		{
					//			if(dist_point2plane(cells[tmp].points_in[i].xyz,coefficients_plane)<maxdist_point2plane)
					//			{
					//				tmp_point_pcl.x=cells[tmp].points_in[i].xyz[0];
					//				tmp_point_pcl.y=cells[tmp].points_in[i].xyz[1];
					//				tmp_point_pcl.z=cells[tmp].points_in[i].xyz[2];
					//				tmp_point_pcl.r=cells[tmp].points_in[i].rgb[0];
					//				tmp_point_pcl.g=cells[tmp].points_in[i].rgb[1];
					//				tmp_point_pcl.b=cells[tmp].points_in[i].rgb[2];
					//				cloud_contain_plane->push_back(tmp_point_pcl);
					//				tmp_normal_pcl.normal_x=cells[tmp].points_in[i].normal[0];
					//				tmp_normal_pcl.normal_y=cells[tmp].points_in[i].normal[1];
					//				tmp_normal_pcl.normal_z=cells[tmp].points_in[i].normal[2];
					//				cloud_contain_plane_normal->push_back(tmp_normal_pcl);
					//				tmp_pixel_pcl.x=cells[tmp].points_in[i].u;
					//				tmp_pixel_pcl.y=cells[tmp].points_in[i].v;
					//				cloud_contain_plane_image->push_back(tmp_pixel_pcl);
					//			}
					//		}
					//	}
					//}
					//if(debug)
					//{
					//	std::cout<<"after expanding cloud_contain_plane:"<<cloud_contain_plane->size()<<std::endl;
					//}

				// extract plane in the expanded cloud_contain_plane;
				seg.setInputCloud(cloud_contain_plane);
				seg.segment(*inliers_plane, *coefficients_plane);

				// plane
				// - extracted plane;
				// - points on plane in the camera coordinate system;
				extract.setInputCloud (cloud_contain_plane);
				extract.setIndices (inliers_plane);
				extract.setNegative (false);
				extract.filter (*plane); // plane extracted in the maxdir direction
				extract.setNegative (true);
				extract.filter (*cloud_contain_plane);

				// plane_normal
				// - extracted plane;
				// - local normal of each point on plane;
				extract_normal.setInputCloud (cloud_contain_plane_normal);
				extract_normal.setIndices (inliers_plane);
				extract_normal.setNegative (false);
				extract_normal.filter (*plane_normal);
				extract_normal.setNegative (true);
				extract_normal.filter (*cloud_contain_plane_normal);

				// plane_image
				// - extracted plane;
				// - pixel coordinate corresponding to each point on plane;
				extract_image.setInputCloud (cloud_contain_plane_image);
				extract_image.setIndices (inliers_plane);
				extract_image.setNegative (false);
				extract_image.filter (*plane_image);
				extract_image.setNegative (true);
				extract_image.filter (*cloud_contain_plane_image);

				have_same_plane=false;

				if(plane->size()<=min_plane_size)
				{
					// if the extracted plane is not large enough;
					// then the following "while" will not be activated;
					*cloud_rest_final=*cloud_rest_final+*cloud_contain_plane;
					*cloud_rest_final=*cloud_rest_final+*plane;
				}

				while(plane->size() > min_plane_size && enough_plane == false)
				{
					if (scan->planes.size() < max_plane)
					{
						// tmp_plane
						// - allocate a new plane feaure;
						// - normal, d: from the extraction method;
						// - points_in, num_points;
						// - avg_pps, cov_pps, avg_rgb and cov_rgb: computed from points_in;
						tmp_plane=new Plane;
						unifyPlaneDir(coefficients_plane);
						tmp_plane->points_in.clear();
						tmp_plane->normal[0]=coefficients_plane->values[0];
						tmp_plane->normal[1]=coefficients_plane->values[1];
						tmp_plane->normal[2]=coefficients_plane->values[2];
						tmp_plane->d=coefficients_plane->values[3];
						for(int i_point=0;i_point<plane->size();i_point++)
						{
							tmp_point.xyz[0]=plane->at(i_point).x;
							tmp_point.xyz[1]=plane->at(i_point).y;
							tmp_point.xyz[2]=plane->at(i_point).z;
							tmp_point.rgb[0]=plane->at(i_point).r/255.0;
							tmp_point.rgb[1]=plane->at(i_point).g/255.0;
							tmp_point.rgb[2]=plane->at(i_point).b/255.0;
							tmp_point.normal[0]=plane_normal->at(i_point).normal_x;
							tmp_point.normal[1]=plane_normal->at(i_point).normal_y;
							tmp_point.normal[2]=plane_normal->at(i_point).normal_z;
							tmp_point.u=plane_image->at(i_point).x;
							tmp_point.v=plane_image->at(i_point).y;
							tmp_point.pps=Cartesian2PPS(tmp_point.normal,tmp_point.xyz,Rotation2eigen);
							tmp_plane->points_in.push_back(tmp_point);
							tmp_plane->num_points=tmp_plane->points_in.size();
						}
						// compute avg_pps, cov_pps, avg_rgb and cov_rgb of tmp_plane;
						computePlaneAvgCov(tmp_plane);
					
						// if "planes" are not empty, i.e., there are already extracted planes;
						// test if current plane is the same with any one in "planes";
						// if so, fuse the same planes;
						if(scan->planes.size()>0)
						{
							for(int i_plane=0;i_plane<scan->planes.size();i_plane++)
							{
								// angle(n1,n2)<thres_angle, |d1-d2|<thres_dist, delta_color<thres_color;
								double tmp_cos_angle=tmp_plane->normal.transpose()*scan->planes[i_plane]->normal;
								Eigen::Vector3d tmp_delta_rgb=tmp_plane->avg_rgb-scan->planes[i_plane]->avg_rgb;
								if(acos(tmp_cos_angle)<thres_angle && abs(tmp_plane->d-scan->planes[i_plane]->d)<thres_dist && tmp_delta_rgb.norm()<thres_color)
								{
									have_same_plane=true;
									fusePlanes(i_plane,tmp_plane);
									*allplane = *plane + *allplane;
									if(debug)
									{
										std::cout<<"same with the "<<i_plane<<"th plane, after fusion:"<<std::endl;
										std::cout<<"\tnormal="<<scan->planes[i_plane]->normal.transpose()<<std::endl;
										std::cout<<"\td="<<scan->planes[i_plane]->d<<std::endl;
										std::cout<<"\tsize="<<scan->planes[i_plane]->num_points<<std::endl;
										std::cout<<"\tavg_pps="<<scan->planes[i_plane]->avg_pps.transpose()<<std::endl;
										std::cout<<"\tavg_rgb="<<scan->planes[i_plane]->avg_rgb.transpose()<<std::endl;
										std::cout<<"cov_pps:"<<std::endl<<scan->planes[i_plane]->cov_pps<<std::endl;
										Eigen::EigenSolver<Eigen::Matrix3d> es(scan->planes[i_plane]->cov_pps);
										std::cout<<"eigenvalues_pps:"<<es.eigenvalues().real().transpose()<<std::endl;

										std::cout<<"cov_rgb:"<<std::endl<<scan->planes[i_plane]->cov_rgb<<std::endl;
										es=Eigen::EigenSolver<Eigen::Matrix3d>(scan->planes[i_plane]->cov_rgb);
										std::cout<<"eigenvalues_rgb:"<<es.eigenvalues().real().transpose()<<std::endl;
									}
								}
							}
						}

						// if no same plane, add the tmp_plane to "planes";
						if(have_same_plane == false)
						{
							// no similar planes;
							// save current plane tmp_plane to planes;
							tmp_plane->index=scan->planes.size();
							scan->planes.push_back(tmp_plane);
							*allplane = *plane + *allplane;
							if(debug)
							{
								std::cout<<"the "<<scan->planes.size()-1<<"th plane:"<<std::endl;
								std::cout<<"\tnormal="<<tmp_plane->normal.transpose()<<std::endl;
								std::cout<<"\td="<<tmp_plane->d<<std::endl;
								std::cout<<"\tsize="<<tmp_plane->num_points<<std::endl;
								std::cout<<"\tavg_pps="<<tmp_plane->avg_pps.transpose()<<std::endl;
								std::cout<<"\tavg_rgb="<<tmp_plane->avg_rgb.transpose()<<std::endl;
										std::cout<<"cov_pps:"<<std::endl<<tmp_plane->cov_pps<<std::endl;
										Eigen::EigenSolver<Eigen::Matrix3d> es(tmp_plane->cov_pps);
										std::cout<<"eigenvalues_pps:"<<es.eigenvalues().real().transpose()<<std::endl;

										std::cout<<"cov_rgb:"<<std::endl<<tmp_plane->cov_rgb<<std::endl;
										es=Eigen::EigenSolver<Eigen::Matrix3d>(tmp_plane->cov_rgb);
										std::cout<<"eigenvalues_rgb:"<<es.eigenvalues().real().transpose()<<std::endl;

								std::cout<<"\tcloud_contain_plane:"<<cloud_contain_plane->size()<<std::endl;
							}
						}

						// if there are a lot of points left in the cloud_contain_plane;
						// then more planes may be extracted;
						if (cloud_contain_plane->size() > min_plane_size)
						{
							seg.setInputCloud(cloud_contain_plane);
							seg.segment(*inliers_plane, *coefficients_plane);

							extract.setInputCloud (cloud_contain_plane);
							extract.setIndices (inliers_plane);
							extract.setNegative (false);
							extract.filter (*plane);
							extract.setNegative (true);
							extract.filter (*cloud_contain_plane);

							extract_normal.setInputCloud (cloud_contain_plane_normal);
							extract_normal.setIndices (inliers_plane);
							extract_normal.setNegative (false);
							extract_normal.filter (*plane_normal);
							extract_normal.setNegative (true);
							extract_normal.filter (*cloud_contain_plane_normal);

							extract_image.setInputCloud (cloud_contain_plane_image);
							extract_image.setIndices (inliers_plane);
							extract_image.setNegative (false);
							extract_image.filter (*plane_image);
							extract_image.setNegative (true);
							extract_image.filter (*cloud_contain_plane_image);

							have_same_plane=false;

							if (plane->size() > min_plane_size)
							{
								//had_parallel_plane = true;
							}
							else
							{
								*cloud_rest_final=*cloud_rest_final+*plane;
								*cloud_rest_final = *cloud_rest_final + *cloud_contain_plane;
							}
						}
						else
						{
							*cloud_rest_final = *cloud_rest_final + *cloud_contain_plane;
							plane->clear();
						}
					}
					else
					{
						enough_plane = true;
					}		
				}
			}
			else
			{
				*cloud_rest_final=*cloud_rest_final+*cloud_contain_plane;
			}
			if( enough_plane == false )
			{
				iter_sorted_cells--;
				maxdir=iter_sorted_cells->index;
				maxnum=iter_sorted_cells->num_point;
			}
		}
		if(debug)
		{
			std::cout<<"extracted "<<scan->planes.size()<<" planes"<<std::endl;
			for(int i=0;i<scan->planes.size();i++)
			{
				std::cout<<"\tnormal="<<scan->planes[i]->normal.transpose();
				std::cout<<", d="<<scan->planes[i]->d<<std::endl;
			}
		}
	}

	// unifyPlaneDir
	// - make the plane normal point to the origin;
	void PlaneExtraction_RANSAC::unifyPlaneDir(pcl::ModelCoefficients::Ptr plane)
	{
		double a=plane->values[0];
		double b=plane->values[1];
		double c=plane->values[2];
		double d=plane->values[3];
		bool flag_reverse;
		if ( abs(a)>=abs(b) && abs(a)>=abs(c) )
		{
			float x = (b+c-d)/a;
			if ( (a*(-x)+b+c)>0 )
				flag_reverse=false;
			else
				flag_reverse=true;
		}
		if ( abs(b)>=abs(a) && abs(b)>=abs(c) )
		{
			float y = (a+c-d)/b;
			if ( (b*(-y)+a+c)>0 )
				flag_reverse=false;
			else
				flag_reverse=true;
		}
		if ( abs(c)>=abs(b) && abs(c)>=abs(a) )
		{
			float z = (a+b-d)/c;
			if ( (c*(-z)+b+a)>0 )
				flag_reverse=false;
			else
				flag_reverse=true;
		}
		if(flag_reverse)
		{
			plane->values[0]=-plane->values[0];
			plane->values[1]=-plane->values[1];
			plane->values[2]=-plane->values[2];
			plane->values[3]=-plane->values[3];
		}
	}

	// dist_point2plane
	// - compute the vertical distance from a point to a plane;
	double PlaneExtraction_RANSAC::dist_point2plane(Eigen::Vector3d point, pcl::ModelCoefficients::Ptr plane)
	{
		//if (!pcl::isFinite(x) && !pcl::isFinite(x) && !pcl::isFinite(x))
		return abs(point[0]*plane->values[0]+point[1]*plane->values[1]+point[2]*plane->values[2]+plane->values[3])/sqrt(plane->values[0]*plane->values[0]+plane->values[1]*plane->values[1]+plane->values[2]*plane->values[2]);
	}

	// computePlaneAvgCov
	// - compute the avg_pps, cov_pps, avg_rgb and cov_rgb from the points_in;
	void PlaneExtraction_RANSAC::computePlaneAvgCov(Plane *plane)
	{
		Eigen::Vector3d tmp;
		if(plane->num_points>0)
		{
			plane->avg_pps.setZero();
			plane->avg_rgb.setZero();
			plane->cov_pps.setZero();
			plane->cov_rgb.setZero();
			for(std::vector<Point>::iterator it=plane->points_in.begin();it!=plane->points_in.end();++it)
			{
				plane->avg_pps=plane->avg_pps+it->pps;
				plane->avg_rgb=plane->avg_rgb+it->rgb;
			}
			plane->avg_pps=plane->avg_pps/plane->num_points;
			plane->avg_rgb=plane->avg_rgb/plane->num_points;
			for(std::vector<Point>::iterator it=plane->points_in.begin();it!=plane->points_in.end();++it)
			{
				tmp=it->pps-plane->avg_pps;
				plane->cov_pps=plane->cov_pps+tmp*tmp.transpose();
				tmp=it->rgb-plane->avg_rgb;
				plane->cov_rgb=plane->cov_rgb+tmp*tmp.transpose();
			}
			plane->cov_pps=plane->cov_pps/plane->num_points;
			plane->cov_rgb=plane->cov_rgb/plane->num_points;
		}
	}

	// fusePlanes
	// - fuse tmp_plane to planes[i_plane];
	void PlaneExtraction_RANSAC::fusePlanes(int i_plane, Plane *tmp_plane)
	{
		// compute the avg_pps and cov_pps of the fused plane;
		Eigen::Vector3d tmp_avg=scan->planes[i_plane]->avg_pps;
		scan->planes[i_plane]->avg_pps=(scan->planes[i_plane]->num_points*scan->planes[i_plane]->avg_pps+tmp_plane->num_points*tmp_plane->avg_pps)/(scan->planes[i_plane]->num_points+tmp_plane->num_points);
		scan->planes[i_plane]->cov_pps=(scan->planes[i_plane]->num_points*(scan->planes[i_plane]->cov_pps+tmp_avg*tmp_avg.transpose())+tmp_plane->num_points*(tmp_plane->cov_pps+tmp_plane->avg_pps*tmp_plane->avg_pps.transpose()))/(scan->planes[i_plane]->num_points+tmp_plane->num_points)-scan->planes[i_plane]->avg_pps*scan->planes[i_plane]->avg_pps.transpose();

		// compute the avg_rgb and cov_rgb of the fused plane;
		tmp_avg=scan->planes[i_plane]->avg_rgb;
		scan->planes[i_plane]->avg_rgb=(scan->planes[i_plane]->num_points*scan->planes[i_plane]->avg_rgb+tmp_plane->num_points*tmp_plane->avg_rgb)/(scan->planes[i_plane]->num_points+tmp_plane->num_points);
		scan->planes[i_plane]->cov_rgb=(scan->planes[i_plane]->num_points*(scan->planes[i_plane]->cov_rgb+tmp_avg*tmp_avg.transpose())+tmp_plane->num_points*(tmp_plane->cov_rgb+tmp_plane->avg_rgb*tmp_plane->avg_rgb.transpose()))/(scan->planes[i_plane]->num_points+tmp_plane->num_points)-scan->planes[i_plane]->avg_rgb*scan->planes[i_plane]->avg_rgb.transpose();
		
		// number of points;
		scan->planes[i_plane]->num_points+=tmp_plane->num_points;
		for(std::vector<Point>::iterator it=tmp_plane->points_in.begin();it!=tmp_plane->points_in.end();++it)
		{
			scan->planes[i_plane]->points_in.push_back(*it);
		}
	}

	// getCellPoints
	//void PlaneExtraction_RANSAC::getCellPoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p)
	//{
	//	cloud_p = new pcl::PointCloud<pcl::PointXYZRGBA>;
	//	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	//	extract.setInputCloud (scan->point_cloud);
	//	for(int i=0;i<cells_bottom->getCellSize();i++)
	//	{
	//		if(cells_bottom->getCell(maxdir)->num_points>5000)
	//		{
	//			extract.setIndices (cells_bottom->getCell(maxdir)->inliers);
	//			extract.setNegative (false);
	//			extract.filter (*cloud_p);
	//		}
	//	}
	//}
}
