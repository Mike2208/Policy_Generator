#ifndef ALGORITHMDSTAR_H
#define ALGORITHMDSTAR_H

/*	class AlgorithmDStar
 *		calculates D Star map
 */

#include "map.h"

class AlgorithmDStar
{
	public:
		AlgorithmDStar();

		template<class T>
		static int CalculateDStarMap(const Map<T> &CostMap, const POS_2D &ZeroPos, const T &MinVal, const T &MaxVal, Map<T> &DStarMap);		// Calculates DStar map, which is a map that shows the cost to reach ZeroPos from any other point on the map (MaxVal is needed to initiate DStarMap, and shows which positions can't be reached later on)

		typedef int TMP;

		//template<class T>
		static int UpdateDStarMap(Map<TMP> &DStarMap, )

	private:


};


// Contains all methods of AlgorithmDStar that require templates
#include "algorithm_d_star_templates.h"

#endif // ALGORITHMDSTAR_H
