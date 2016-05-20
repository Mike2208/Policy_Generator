#ifndef OCCUPANCYGRIDMAP_H
#define OCCUPANCYGRIDMAP_H

/*	class OccupancyGridMap
 *		data regarding occupancy grids
 */

#include "map.h"
#include <limits>

typedef unsigned char OGM_TYPE;
typedef Map<OGM_TYPE> OGM_MAP;

const OGM_TYPE OGM_CELL_OCCUPIED = 100;		// Value for occupancy
const OGM_TYPE OGM_CELL_FREE	 = 0;		// Value for free

typedef float OGM_LOG_TYPE;
typedef Map<OGM_LOG_TYPE> OGM_LOG_MAP;

const OGM_LOG_TYPE	OGM_LOG_CELL_OCCUPIED = std::numeric_limits<OGM_LOG_TYPE>::infinity();		// Log value of occupied cell
const OGM_LOG_TYPE	OGM_LOG_CELL_FREE = 0;														// Log value of free cell

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
		static void CalculateLogMapFromOGM(const OGM_MAP &MapData, OGM_LOG_MAP &NewLogMap);		// Calculates the log map from the OGMap

	private:

		OGM_MAP _MapData;
};

#endif // OCCUPANCYGRIDMAP_H
