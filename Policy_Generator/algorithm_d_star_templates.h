#ifndef ALGORITHM_D_STAR_TEMPLATES_H
#define ALGORITHM_D_STAR_TEMPLATES_H

#include "algorithm_d_star.h"
#include "robot_navigation.h"
#include <queue>

template<class T>
int AlgorithmDStar::CalculateDStarMap(const Map<T> &CostMap, const POS_2D &ZeroPos, const T &MaxVal, const T &MinVal, Map<T> &DStarMap)
{
	// Reset DStarMap
	DStarMap.ResetMap(CostMap.GetHeight(), CostMap.GetWidth(), MaxVal);

	std::queue<POS_2D> posToCheck;

	POS_2D curPos = ZeroPos;
	T adjacentDist;

	// Set zero position
	DStarMap.SetPixel(ZeroPos, MinVal);

	// Start at zero position
	posToCheck.push(curPos);

	// Continue checking adjacent positions until all pos have been reached
	do
	{
		const T curVal = DStarMap.GetPixel(posToCheck.front());
		const T curCostVal = CostMap.GetPixel(posToCheck.front());

		// Check all adjacent positions
		for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
		{
			const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(posToCheck.front(), i);

			// Check whether adjacent pixel exists
			if(DStarMap.GetPixel(adjacentPos, adjacentDist) < 0)
				continue;	// Ignore if it doesn't

			const T compVal = curVal + curCostVal;

			// Check whether adjacent distance can be shortened
			if(adjacentDist > compVal)
			{
				// Update DStarMap
				DStarMap.SetPixel(adjacentPos, compVal);

				// Check adjacent position if update was necessary
				posToCheck.push(adjacentPos);
			}
		}

		// Remove first position after it was checked
		posToCheck.pop();
	}
	while(posToCheck.size() > 0);

	return 1;
}

#endif // ALGORITHM_D_STAR_TEMPLATES_H
