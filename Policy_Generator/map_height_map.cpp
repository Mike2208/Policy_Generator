#include "map_height_map.h"
#include "obstacle_connection.h"
#include "robot_navigation.h"
#include <climits>
#include <cmath>
#include <queue>

HeightMap::HeightMap()
{

}

int HeightMap::CalculateHeightMap(const std::vector<HEIGHT_POS> &Maximas, const unsigned int &MapHeight, const unsigned int &MapWidth, Map_IntType &HeightMap)
{
	// Reset map to correct height and width, and set all values to 0
	HeightMap.ResetMap(MapHeight, MapWidth, 0);

	// Create vector for values that will need to be checked
	std::vector<POS_2D> posToCheck;
	posToCheck.resize(Maximas.size());		// Enough space for Maximas
	unsigned int curPosNum;

	// Set all maximas in the map add adjacent positions to posToCheck if those values are smaller than current value-1
	for(unsigned int i=0; i<Maximas.size(); i++)
	{
		HeightMap.SetPixel(Maximas[i].Position, Maximas[i].Height);

		// Add this position to the ones that need to be checked later
		posToCheck[i] = Maximas[i].Position;
	}

	curPosNum = 0;
	while(curPosNum < posToCheck.size())
	{
		const POS_2D &curPos = posToCheck[curPosNum];
		POS_2D adjecentPos = curPos;
		HEIGHT_TYPE curPixelValue, adjacentPixelVal;

		// Get current pixel value
		curPixelValue = HeightMap.GetPixel(curPos);

		// Check top (code copied top/right/bottom/left)
		adjecentPos.X = curPos.X;
		adjecentPos.Y = curPos.Y + 1;
		if(HeightMap.GetPixel(adjecentPos, adjacentPixelVal) >= 0)
		{
			// Get next pixel value and skip this part if they are not available

			// Check that value is smaller than curPixelVal-1
			if(adjacentPixelVal < curPixelValue - 1)
			{
				// Update ajacent position
				HeightMap.SetPixel(adjecentPos, curPixelValue - 1);

				// Add this to positions that need to be checked
				posToCheck.push_back(adjecentPos);
			}
		}

		// Check right (code copied top/right/bottom/left)
		adjecentPos.X = curPos.X + 1;
		adjecentPos.Y = curPos.Y;
		if(HeightMap.GetPixel(adjecentPos, adjacentPixelVal) >= 0)
		{
			// Get next pixel value and skip this part if they are not available

			// Check that value is smaller than curPixelVal-1
			if(adjacentPixelVal < curPixelValue - 1)
			{
				// Update ajacent position
				HeightMap.SetPixel(adjecentPos, curPixelValue - 1);

				// Add this to positions that need to be checked
				posToCheck.push_back(adjecentPos);
			}
		}

		// Check bottom (code copied top/right/bottom/left)
		adjecentPos.X = curPos.X;
		adjecentPos.Y = curPos.Y - 1;
		if(HeightMap.GetPixel(adjecentPos, adjacentPixelVal) >= 0)
		{
			// Get next pixel value and skip this part if they are not available

			// Check that value is smaller than curPixelVal-1
			if(adjacentPixelVal < curPixelValue - 1)
			{
				// Update ajacent position
				HeightMap.SetPixel(adjecentPos, curPixelValue - 1);

				// Add this to positions that need to be checked
				posToCheck.push_back(adjecentPos);
			}
		}

		// Check left (code copied top/right/bottom/left)
		adjecentPos.X = curPos.X - 1;
		adjecentPos.Y = curPos.Y;
		if(HeightMap.GetPixel(adjecentPos, adjacentPixelVal) >= 0)
		{
			// Get next pixel value and skip this part if they are not available

			// Check that value is smaller than curPixelVal-1
			if(adjacentPixelVal < curPixelValue - 1)
			{
				// Update ajacent position
				HeightMap.SetPixel(adjecentPos, curPixelValue - 1);

				// Add this to positions that need to be checked
				posToCheck.push_back(adjecentPos);
			}
		}

		curPosNum++;
	}

	return 1;
}

int HeightMap::FindMinHeightPos(const Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST> &HeightMap, const POS_2D &CurPos, POS_2D &MinHeightPos)
{
	OBSTACLE_PATH_FINDER::MAP_ID_DIST tmpData = HeightMap.GetPixel(CurPos);

	// Get Current ID and distance
	const OBSTACLE_ID curID = tmpData.ID;
	unsigned int curDist = tmpData.Distance;
	bool errorEncountered;

	MinHeightPos = CurPos;
	while(curDist > 0)
	{
		errorEncountered = true;

		// Check all adjacent positions
		for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
		{
			const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(MinHeightPos, i);

			if(HeightMap.GetPixel(adjacentPos, tmpData) >= 0)
			{
				// Check if ID is the same
				if(tmpData.ID != curID)
					continue;

				// Check if distance is less
				if(tmpData.Distance < curDist)
				{
					MinHeightPos = adjacentPos;
					curDist = tmpData.Distance;

					errorEncountered = false;		// Error checking

					break;
				}
			}
		}

		// no lower distance found, return error
		if(errorEncountered)
			return -1;
	}

	return 1;
}
