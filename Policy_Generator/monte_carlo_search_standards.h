#ifndef MONTE_CARLO_SEARCH_STANDARDS_H
#define MONTE_CARLO_SEARCH_STANDARDS_H

namespace MONTE_CARLO_SEARCH_STANDARDS
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
}


#endif // MONTECARLOSEARCHSTANDARDS_H
