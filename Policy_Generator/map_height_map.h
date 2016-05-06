#ifndef HEIGHTMAP_H
#define HEIGHTMAP_H

#include "map.h"
#include "map_standards.h"
#include "graph_class.h"
#include "obstacle_connections.h"

#include <vector>

struct HEIGHT_POS
{
	POS_2D Position;
	HEIGHT_TYPE Height;
};

class HeightMap
{
	public:
		HeightMap();

		static int CalculateHeightMap(const std::vector<HEIGHT_POS> &Maximas, const unsigned int &MapHeight, const unsigned int &MapWidth, Map<HEIGHT_TYPE> &HeightMap);
};

#endif // HEIGHTMAP_H
