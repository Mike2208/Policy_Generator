#ifndef ALGORITHM_OPTIMAL_SINGLE_BOT_H
#define ALGORITHM_OPTIMAL_SINGLE_BOT_H

/*	class AlgorithmOptimalSingleBot
 *		used to calculate the optimal policy for statistic OGM
 */

#include "map.h"
#include "map_path_finder.h"
#include "map_standards.h"

class AlgorithmOptimalSingleBot
{
	public:

		AlgorithmOptimalSingleBot();
		~AlgorithmOptimalSingleBot();

		//int CalculatePolicy(const std::vector<Map_Statistical> &Maps, const POSE &StartPose, const POSE &EndPose, const ALG_PROBABILITY_TYPE &MinMapCertainty);			// Calculate optimal policy

	private:

};

#endif // ALGORITHM_OPTIMAL_SINGLE_BOT_H
