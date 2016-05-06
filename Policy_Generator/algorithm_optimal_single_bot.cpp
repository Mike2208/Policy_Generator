#include "algorithm_optimal_single_bot.h"
#include <array>
#include <climits>

AlgorithmOptimalSingleBot::AlgorithmOptimalSingleBot()
{

}

AlgorithmOptimalSingleBot::~AlgorithmOptimalSingleBot()
{

}



//int AlgorithmOptimalSingleBot::CalculatePolicy(const std::vector<Map_Statistical> &Maps, const POSE &StartPose, const POSE &EndPose, const ALG_PROBABILITY_TYPE &MinMapCertainty)
//{
//	Map_BoolType visitedPositions;																			// 0 for not visited, 1 for visited
//	visitedPositions.SetMapToValue(false);
//	std::vector<ALG_PROBABILITY_TYPE, Maps.size()> mapPercentage(1.0/Maps.size());							// Probabiliies of different maps
//	unsigned int curCost = 0;

//	const unsigned int areaSize = Maps[0].GetHeight()*Maps[0].GetWidth();

//	// Set current position to start position
//	POS_2D curPos = StartPose.Position;

//	// Set current tyle to visited
//	visitedPositions.SetPixel(curPos, true);

//	// Compare value of current field to map values and update certainties
//	for(unsigned int i=0; i<mapPercentage.size(); i++)
//	{
//		const OCCUPANCYGRID_STAT_TYPE curPixelVal = Maps[i].GetPixel(curPos);

//		// if Map states that tile should be empty, add to certainty, else remove from certainty
//		if(curPixelVal <= 0.5)
//		{
//			mapPercentage[i] += (1.0 - ((ALG_PROBABILITY_TYPE)(OCCUPANCYGRID_STAT_HALF-curPixelVal))/(OCCUPANCYGRID_STAT_HALF))/areaSize;
//		}
//		else
//		{
//			mapPercentage[i] -= (1.0 - ((ALG_PROBABILITY_TYPE)(curPixelVal)-OCCUPANCYGRID_STAT_HALF)/(OCCUPANCYGRID_STAT_HALF))/areaSize;
//		}
//	}

//	// Check surroundings for full tyles and compare that to map data
//	for(int i=-1; i<=1; i++)
//	{
//		for(int j=-1; j<= 1; j++)
//		{
//			OCCUPANCYGRID_STAT_TYPE curPixelVal;

//			// Check that pixel is valid
//			if(Maps[i].GetPixel(curPos.X + i, curPos.Y + j, curPixelVal))
//				continue;

//			// Check that current pixel hasn't been visited yet
//			if(visitedPositions.GetPixel(curPos.X + i, curPos.Y + j) == true)
//				continue;
//		}
//	}
//}
