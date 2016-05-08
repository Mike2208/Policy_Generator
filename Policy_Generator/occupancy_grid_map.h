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
};

#endif // OCCUPANCYGRIDMAP_H
