/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-03-09 09:45
#
# Filename:		PlaneMatching.cpp
#
# Description: 
#
===============================================*/
#include "PlaneMatching.h"

namespace sun
{
	PlaneMatching::PlaneMatching()
	{}

	PlaneMatching::~PlaneMatching()
	{}

	void PlaneMatching::loadPlanes(std::vector<Plane*> _planes_ref, std::vector<Plane*> _planes_cur)
	{
		planes_ref.clear();
		planes_cur.clear();
		for(int i_ref=0;i_ref<_planes_ref.size();i_ref++)
		{
			planes_ref.push_back(_planes_ref[i_ref]);
		}
		for(int i_cur=0;i_cur<_planes_cur.size();i_cur++)
		{
			planes_cur.push_back(_planes_cur[i_cur]);
		}
	}
}
