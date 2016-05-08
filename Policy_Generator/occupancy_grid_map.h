#ifndef OCCUPANCYGRIDMAP_H
#define OCCUPANCYGRIDMAP_H

/*	class OccupancyGridMap
 *		data regarding occupancy grids
 */

#include "map.h"

typedef unsigned char OGM_TYPE;
typedef Map<OGM_TYPE> OGM_MAP;

class OccupancyGridMap
{
	public:
		OccupancyGridMap();

		static OGM_TYPE CalculateAverageProbability(const OGM_MAP &MapData);		// returns average probability of entire map
};

#endif // OCCUPANCYGRIDMAP_H
