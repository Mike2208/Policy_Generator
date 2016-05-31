#ifndef MAPPATHFINDER_H
#define MAPPATHFINDER_H

/*	class MapPathFinder
 *		finds the best path from point A to B in a given map
 */

#include "map.h"
#include "map_standards.h"
#include "map_path.h"

class MapPathFinder
{
	public:
		MapPathFinder();
		MapPathFinder(const MapPathFinder &S);
		~MapPathFinder();

		static int CalculateDistMap(const Map_BoolType &MainMap, const POS_2D &StartPos, Map_IntType &DistMap);			// Finds the distance from a given point to the rest of the map

		static int CalculatePath(const Map_BoolType &MainMap, const POS_2D &Start, const POS_2D &Destination, MapPath &Path);		// Calculates the path from point A to point B (ignores rotation)
		static int CalculatePath(const Map_BoolType &MainMap, const POSE &Start, const POSE &Destination, MapPath &Path);			// Calculates the path from point A to point B

	private:

};


// Include functions here because templates require direct access to both the declaration and implementation
//#include "map_path_finder.cpp"

#endif // MAPPATHFINDER_H
