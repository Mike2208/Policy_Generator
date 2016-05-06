#include "map_comparator.h"

#include <iostream>

MapComparator::MapComparator()
{
}

MapComparator::~MapComparator()
{
}

int MapComparator::CompareMaps(const Map_BoolType &Map1, const Map_BoolType &Map2, Map_BoolType &Result) const
{
	const unsigned int height = Map1.GetHeight();
	const unsigned int width = Map1.GetWidth();

	// Check that both maps are same size
	if(height != Map2.GetHeight() || width != Map2.GetWidth())
	{
		std::cerr << "MapComparator::CompareMaps() ERROR: Maps have different area\n";
		return -1;
	}

	// Resize result map and empty it
	Result.ResetMap(height, width, OCCUPANCYGRID_DISCRETE_EMPTY);

	// Go through maps and determine where they differ
	for(unsigned int posX = 0; posX < width; posX++)
	{
		for(unsigned int posY = 0; posY < height; posY++)
		{
			// Record where the difference between values is large enough
			if(std::abs(Map1.GetPixel(posX, posY) - Map2.GetPixel(posX, posY)) >= MAPCOMPARATOR_MINDIFF)
			{
				Result.SetPixel(posX, posY, OCCUPANCYGRID_DISCRETE_FULL);
			}
		}
	}

	return 1;
}
