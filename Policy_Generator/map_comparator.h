#ifndef MAPANALYSER_H
#define MAPANALYSER_H

/*	class MapAnalyser
 *		used to compare two maps and highlight the differences
 */


#include "map.h"


class MapComparator
{
	public:
		MapComparator();
		~MapComparator();

		int CompareMaps(const Map_BoolType &Map1, const Map_BoolType &Map2, Map_BoolType &Result) const;
};

#endif // MAPANALYSER_H
