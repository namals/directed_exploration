#ifndef MAPPROCESSING_POINTSET_H
#define MAPPROCESSING_POINTSET_H

#include <map>
#include <vector>

#include <opencv/cv.h>

namespace mapprocessing
{

struct PointX
{ 
	std::map<int,CvPoint>* points;
}; 

class PointSet
{
public:
	PointSet();
	~PointSet();

	void addPoint(CvPoint p);
	void getPointsList(std::vector<CvPoint> * pl);

private:
	std::map<int,PointX>* pointMap;
};

} // ~namespace

#endif
