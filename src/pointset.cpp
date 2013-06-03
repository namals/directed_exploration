#include <directed_exploration/mapprocessing/pointset.h>

#include <iostream>

using namespace mapprocessing;

PointSet::PointSet()
{
	pointMap = new std::map<int,PointX>(); // using default comparators
}

PointSet::~PointSet()
{
	std::map<int,PointX>::iterator it;
	for(it = pointMap->begin(); it != pointMap->end(); it++)
	{
		delete (it->second).points;
	}

	delete pointMap;

}

void
PointSet::addPoint(CvPoint p)
{
	std::map<int,PointX>::iterator it;
	it = pointMap->find(p.x);
	if( it == pointMap->end() ) 
	{
		// no mapping for x coordinate
		PointX px;
		px.points = new std::map<int,CvPoint>(); // using default comparators
		(px.points)->insert(std::pair<int,CvPoint>(p.y, p));
		pointMap->insert(std::pair<int,PointX>(p.x, px));
	}
	else
	{
		PointX px = it->second;
		std::map<int,CvPoint>::iterator it2;
		it2 = (px.points)->find(p.y);
		if(it2 == (px.points)->end() )
		{
			// no mapping for y coordinate
			(px.points)->insert(std::pair<int,CvPoint>(p.y, p));
		}
		// else the mapping already exists, so ignore
	}
}

void
PointSet::getPointsList(std::vector<CvPoint>* pointList)
{
	std::map<int,PointX>::iterator it1;
	std::map<int,CvPoint>::iterator it2;

	pointList->clear();

	PointX p;
	for(it1 = pointMap->begin(); it1 != pointMap->end(); it1++)
	{
		p = it1->second;
		for(it2 = (p.points)->begin(); it2 != (p.points)->end(); it2++)
		{
			pointList->push_back(it2->second);
		}
	}
}

