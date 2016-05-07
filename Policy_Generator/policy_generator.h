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

namespace POLICY_GENERATOR
{

}

typedef unsigned char OGM_TYPE;
typedef Map<OGM_TYPE> OGM_MAP;

class PolicyGenerator
{
	public:

		PolicyGenerator();
};

#endif // POLICY_GENERATOR_H
