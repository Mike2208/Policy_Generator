#ifndef MONTE_CARLO_SIMULATION_H
#define MONTE_CARLO_SIMULATION_H

/*	class MonteCarloSearchSimulation
 *		used to simulate game using selected node data as starting point
 *
 */

#include "tree_class.h"

namespace MONTE_CARLO_SIMULATION
{
}

template<class T>
class MonteCarloSimulation
{
	public:
		typedef int (*SimulationFcn)(const TreeNode<T> *ExploreNode);		// Function used to explore/simulate

		MonteCarloSimulation();
		MonteCarloSimulation(const MonteCarloSimulation &S);

		void SetSimulationFcn(SimulationFcn NewFcn) { this->_RunSimulationFcn = NewFcn; }
		void SetTree(TreeClass<T> * const TreeData) { this->_Tree = TreeData; }

		int RunSimulation(TreeNode<T> *ExploreNode) { return this->_RunSimulationFcn(ExploreNode); }

	private:

		SimulationFcn _RunSimulationFcn;		// Function to run for simulation
		TreeClass<T>  *_Tree;					// Tree data
};

#endif // MONTE_CARLO_SIMULATION_H
