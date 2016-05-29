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

OGM_LOG_TYPE OccupancyGridMap::CalculateLogValue(const OGM_TYPE &Value)
{
	return -logf(OGM_CELL_OCCUPIED-Value)+logf(OGM_CELL_OCCUPIED);			// Add logf(OGM_CELL_OCCUPIED) to normalize
}

OGM_LOG_TYPE OccupancyGridMap::CalculateCellEntropy(const OGM_TYPE &Value)
{
	const OGM_LOG_TYPE prob = static_cast<OGM_LOG_TYPE>(OccupancyGridMap::CalculateProbability(Value));
	if(prob == 0 || prob == 1)
		return 0;	// avoid error
	const OGM_LOG_TYPE inverseProb = OGM_PROB_MAX-prob;
	return -(prob*std::log2(prob)+inverseProb*std::log2(inverseProb));
}

OGM_LOG_TYPE OccupancyGridMap::CalculateMapEntropy(const OGM_MAP &Map)
{
	OGM_LOG_TYPE mapUncertainty = 0;

	// Go through all cells and calculate uncertainty
	for(POS_2D_TYPE X=0; X<Map.GetWidth(); X++)
	{
		for(POS_2D_TYPE Y=0; Y<Map.GetHeight(); Y++)
		{
			mapUncertainty += OccupancyGridMap::CalculateCellEntropy(Map.GetPixel(X,Y));
		}
	}

	return mapUncertainty;
}

OGM_PROB_TYPE OccupancyGridMap::CalculateProbability(const OGM_TYPE &Value)
{
	return static_cast<OGM_PROB_TYPE>(Value)/static_cast<OGM_PROB_TYPE>(OGM_CELL_OCCUPIED);
}
