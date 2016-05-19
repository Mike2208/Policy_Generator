#ifndef MONTE_CARLO_STANDARDS_H
#define MONTE_CARLO_STANDARDS_H

#include "tree_class.h"

namespace MONTE_CARLO_STANDARDS
{
	// Data stored in tree nodes
	template<class T>
	class TREE_DATA
	{
		public:
			float NodeValue;			// Wins and Losses of this node
			int NumVisits;				// Number of visits to this node

			T	ExtraData;				// Extra user data for node

			TREE_DATA() : NodeValue(0), NumVisits(0) {}
	};

	template<class T>
	using MONTE_CARLO_TREE = TreeClass<TREE_DATA<T>>;

	template<class T>
	using MONTE_CARLO_TREE_NODE = TreeNode<TREE_DATA<T>>;
}


#endif // MONTECARLOSEARCHSTANDARDS_H
