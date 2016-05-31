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
const OGM_TYPE OGM_CELL_UNKNOWN  = (OGM_CELL_OCCUPIED-OGM_CELL_FREE)/2;		// Value for complete uncertainty

typedef float OGM_LOG_TYPE;
typedef Map<OGM_LOG_TYPE> OGM_LOG_MAP;

typedef float OGM_PROB_TYPE;
const OGM_PROB_TYPE	OGM_PROB_MAX = 1;
const OGM_PROB_TYPE	OGM_PROB_MIN = 0;

class OccupancyGridMap
{
	public:
		OccupancyGridMap();
		OccupancyGridMap(const OccupancyGridMap &S);

		unsigned int GetMapHeight() const { return this->_MapData.GetHeight(); }
		unsigned int GetMapWidth() const { return this->_MapData.GetWidth(); }

		const OGM_MAP &GetMapData() const { return this->_MapData; }

		OGM_TYPE GetGridValue(const POS_2D &Pos) const { return this->_MapData.GetPixel(Pos); }
		int GetGridValue(const POS_2D &Pos, OGM_TYPE &Value) const { return this->_MapData.GetPixel(Pos, Value); }

		void SetData(const OGM_MAP &NewData) { this->_MapData.CopyMapData(NewData); }

		static OGM_TYPE CalculateAverageProbability(const OGM_MAP &MapData);		// returns average probability of entire map
		static void CalculateLogMapFromOGM(const OGM_MAP &MapData, OGM_LOG_MAP &NewLogMap);		// Calculates the log map from the OGMap

		static OGM_LOG_TYPE CalculateLogValue(const OGM_TYPE &Value);

		static OGM_LOG_TYPE	CalculateCellEntropy(const OGM_TYPE &Value);		// Calculates entropy of one cell
		static OGM_LOG_TYPE CalculateMapEntropy(const OGM_MAP &Map);			// Calculates uncertainty of given map

		static OGM_PROB_TYPE CalculateProbability(const OGM_TYPE &Value);

	private:

		OGM_MAP _MapData;
};

const OGM_LOG_TYPE OGM_LOG_CELL_OCCUPIED = GetInfinteVal<OGM_LOG_TYPE>();						// Log value of occupied cell
const OGM_LOG_TYPE OGM_LOG_CELL_FREE = 0;														// Log value of free cell
const OGM_LOG_TYPE OGM_LOG_CELL_UNKNOWN = OccupancyGridMap::CalculateLogValue(OGM_CELL_UNKNOWN);

#endif // OCCUPANCYGRIDMAP_H
