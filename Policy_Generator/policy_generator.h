#ifndef POLICY_GENERATOR_H
#define POLICY_GENERATOR_H

/*	class PolicyGenerator
 *		creates a movement policy given a probabilistic occupancy grid map
 */



/*	THEORY:
 *		- Get OGM map, robot scan range and start and destination
 *		- Separate OGM map into free areas and occupied areas using a percentage threshold t as separator
 *		- Separate free areas into districts through using the minimum distance between adjacent obstacles as dividers
 *		- If a district has dividers that are further than scan range from the center, set the threshold to the average of the all values in district and divide it again
 *		- Do the same for the occupied regoins
 *		- Create a graph G
 *			- Vertices are centers of districts, centers of dividers, and center of edge between free space and obstacles
 *			- Edges connect centers of districts with dividers. They contain an estimate of path length between vertices and reliability of path
 *		- Use this graph G to create policy
 *			- Movement options are either move to new vertice or observe whether edge exists
 *			- One can only move along an edge after it has been observed or already traversed
 *			- Start at start position
 */

#include "map.h"
#include "map_standards.h"
#include "occupancy_grid_map.h"
#include "policy_data.h"

namespace POLICY_GENERATOR
{

}

class PolicyGenerator
{
	public:

		PolicyGenerator();

		int CalculatePolicy(const OGM_MAP &OGMmap, PolicyData &DecisionData);		// use a map to create a decision tree

	private:

		OGM_TYPE CalculateAverageProbability(const OGM_MAP &OGMmap);		// returns average probability of entire map
};

#endif // POLICY_GENERATOR_H
