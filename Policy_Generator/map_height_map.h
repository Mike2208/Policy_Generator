#ifndef HEIGHTMAP_H
#define HEIGHTMAP_H

#include "map.h"
#include "robot_navigation.h"
#include "map_standards.h"
#include "graph_class.h"
#include "obstacle_connection.h"
#include "obstacle_path_finder.h"

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

		//template<class T>
		//static int FindMinHeightPos(const Map<T> &HeightMap, const POS_2D &CurPos, POS_2D &MinHeightPos);
		static int FindMinHeightPos(const Map<unsigned int> &DistMap, const Map_ID &IDMap,const POS_2D &CurPos, POS_2D &MinHeightPos);

		template<class T>
		static int FindMinCostPathLength(const Map<T> &HeightMap, const POS_2D &StartPos, const POS_2D &ZeroPos, unsigned int &PathLength);		// Calculates minimum path length that follows gradient towards position with zero value

		template<class T>
		static int CalculateHomologyDistMap(const Map<T> &OriginalMap, const POS_2D &DestPos, const T &CutoffValue, Map_IntType &HomologyMap);			// Calculates a distance map to destination setting all values above CutoffValue as occupied and all below as free

	private:
};

#include "map_height_map_templates.h"

#endif // HEIGHTMAP_H
