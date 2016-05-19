#ifndef OCCUPANCYGRIDMAP_H
#define OCCUPANCYGRIDMAP_H

/*	class OccupancyGridMap
 *		data regarding occupancy grids
 */

#include "map.h"

typedef unsigned char OGM_TYPE;
typedef Map<OGM_TYPE> OGM_MAP;

const OGM_TYPE OGM_CELL_OCCUPIED = 100;		// Value for occupancy
const OGM_TYPE OGM_CELL_FREE	 = 0;		// Value for free

class OccupancyGridMap
{
	public:
		OccupancyGridMap();

		unsigned int GetMapHeight() const { return this->_MapData.GetHeight(); }
		unsigned int GetMapWidth() const { return this->_MapData.GetWidth(); }

		OGM_TYPE GetGridValue(const POS_2D &Pos) const { return this->_MapData.GetPixel(Pos); }
		int GetGridValue(const POS_2D &Pos, OGM_TYPE &Value) const { return this->_MapData.GetPixel(Pos, Value); }

		void SetData(const OGM_MAP &NewData) { this->_MapData.CopyMapData(NewData); }

		static OGM_TYPE CalculateAverageProbability(const OGM_MAP &MapData);		// returns average probability of entire map

	private:

		OGM_MAP _MapData;
};

#endif // OCCUPANCYGRIDMAP_H
