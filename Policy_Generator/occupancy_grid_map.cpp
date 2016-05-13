#include "occupancy_grid_map.h"
#include <cmath>

OccupancyGridMap::OccupancyGridMap()
{

}

OGM_TYPE OccupancyGridMap::CalculateAverageProbability(const OGM_MAP &MapData)
{
	double curAverage = 0;

	POS_2D curPos;
	for(curPos.X=0; curPos.X<MapData.GetWidth(); curPos.X++)
	{
		for(curPos.Y=0; curPos.Y<MapData.GetHeight(); curPos.Y++)
		{
			curAverage += MapData.GetPixel(curPos);
		}
	}

	return static_cast<OGM_TYPE>(std::round(curAverage/(MapData.GetHeight()*MapData.GetWidth())));
}
