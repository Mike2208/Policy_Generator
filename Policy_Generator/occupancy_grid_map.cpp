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

void OccupancyGridMap::CalculateLogMapFromOGM(const OGM_MAP &MapData, OGM_LOG_MAP &NewLogMap)
{
	// Resize new map
	NewLogMap.ResizeMap(MapData.GetHeight(), MapData.GetWidth());

	// Calculate Log value for all grid cells
	for(POS_2D_TYPE X = 0; X < MapData.GetWidth(); X++)
	{
		for(POS_2D_TYPE Y = 0; Y < MapData.GetHeight(); Y++)
		{
			NewLogMap.SetPixel(X,Y, OccupancyGridMap::CalculateLogValue(MapData.GetPixel(X,Y)));
		}
	}
}

inline OGM_LOG_TYPE OccupancyGridMap::CalculateLogValue(const OGM_TYPE &Value)
{
	return -logf(OGM_CELL_OCCUPIED-Value)+logf(OGM_CELL_OCCUPIED);			// Add logf(OGM_CELL_OCCUPIED) to normalize
}
