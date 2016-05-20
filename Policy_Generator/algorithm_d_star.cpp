#include "algorithm_d_star.h"

AlgorithmDStar::AlgorithmDStar()
{

}

#include "robot_navigation.h"
#include <queue>
int AlgorithmDStar::CalculateDStarMap(const Map<TMP> &CostMap, const POS_2D &ZeroPos, const TMP &MaxVal, const TMP &MinVal, Map<TMP> &DStarMap)
{
	// Reset DStarMap
	DStarMap.ResetMap(CostMap.GetHeight(), CostMap.GetWidth(), MaxVal);

	std::queue<POS_2D> posToCheck;

	POS_2D curPos = ZeroPos;
	TMP adjacentDist;

	// Set zero position
	DStarMap.SetPixel(ZeroPos, MinVal);

	// Start at zero position
	posToCheck.push(curPos);

	// Continue checking adjacent positions until all pos have been reached
	do
	{
		const TMP curVal = DStarMap.GetPixel(posToCheck.front());
		const TMP curCostVal = CostMap.GetPixel(posToCheck.front());

		// Check all adjacent positions
		for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
		{
			const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(posToCheck.front(), i);

			// Check whether adjacent pixel exists
			if(DStarMap.GetPixel(adjacentPos, adjacentDist) < 0)
				continue;	// Ignore if it doesn't

			const TMP compVal = curVal + curCostVal;

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
