#ifndef MAPPROCESSING_OGMAP_H
#define MAPPROCESSING_OGMAP_H

#include <vector>
#include "mapimage.h"
#include "../util.h"

namespace mapprocessing
{

//bool compare(const ::orca::Pose2dHypothesis a, const ::orca::Pose2dHypothesis b);	

class OgMap
{
public:
    //OgMap(orca::OgMapData&);
    OgMap(double offsetX, double offsetY, int width, int height, double resolution, 
		  const unsigned char* data, double obs_inflation_dis, double frontier_clearance_dis);
	~OgMap();
	
	OgMap* getSubOgMap(double offsetX, double offsexY, double width, double height);
	OgMap* getSubOgMap(int offsetX, int offsetY, int width, int height);

	int getWidth();
	int getHeight();
	float getOffsetX();
	float getOffsetY();
	float getMetersPerCellX();
	float getMetersPerCellY();

	void displayMap();
	void displayShortestDistanceAsMap();
	void displayShortestPath();
	void saveMap(int sequenceNo);

	MapImage* getMapImage();
	Vector2d getPositionInMeters(int xpos, int ypos);
	CvPoint getPositionInPixelPoints(Vector2d);
	void updateMap(const unsigned char* data);
	void getFrontiers(std::vector<Vector2d> & frontiers);
	double getShortestDistanceToInMeters(const Vector2d&);
	double getShortestDistanceToInPixels(const CvPoint& );
	void addInflatedObsLayer(std::vector<mapprocessing::Vector2d> & inflated_layer);

	/*
	 * Gets the shortest path to the point <pos> from the recently updated distance map
	 * path contains the shortest path with index 0 containing the source and
	 * index (legth-1) containing the destination point (<pos>)
	 */
	void getShortestPath(std::vector<Vector2d> & path, Vector2d pos);
	unsigned char getMapVal(int row, int col);
	void calculateShortestDistances(const Vector2d & pos);
	double getExploredFraction(const std::vector< Vector2d > & pathHistory);

	void displayDistanceMap();

	double getInformationGain(int x, int y, int range);

private:
	/* no of cells in x and y directions */
	int width;
	int height;

	/* starting points of lower left corner in global coordinates in meters */
	double offsetX;
	double offsetY;

	double metersPerCellX;
	double metersPerCellY;
	
	double obs_inflation_dis;
	double frontier_clearance_dis;
	
	unsigned char** data;
	double** distances;
	bool** visit;
	CvPoint** father;

	MapImage* mapImage;

	char* getMapAsSingleArray();

	OgMap(unsigned char**, int, int, double, double, double, double);
	void initMap(int, int);
	Vector2d converFromImgToGlobalCoordinates(CvPoint&);
	double euclideanDistance(double x1, double y1, double x2, double y2);
	bool isInsideNeighborhood(double targetX, double targetY,
							  double refX, double refY, double radius = 0.5);

	void getShortestPathInPixels(std::vector<CvPoint> & path, CvPoint p);	
};

}  // ~namespace

#endif
