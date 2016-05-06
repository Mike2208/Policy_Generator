#include "map_height_map.h"
#include "obstacle_connections.h"
#include <climits>
#include <cmath>

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

