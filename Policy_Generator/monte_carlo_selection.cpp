#include "monte_carlo_selection.h"
#include <cmath>

namespace MONTE_CARLO_SELECTION
{
	int UCL_Function(UCL_TREE_CLASS &Tree, const UCL_TREE_NODE **SelectedNode)
	{
		 UCL_TREE_DATA curData;

		// Perform selection of next node
		UCL_TREE_NODE *pCurNode = Tree.GetRoot();

		unsigned int timesParentVisited;
		float bestValue, nextValue;
		UCL_TREE_NODE *pBestNode = pCurNode;

		// Go through children until a leaf is discovered
		while(!pCurNode->IsLeaf())
		{
			// Get relevant parent data
			UCL_TREE_NODE * const pParentNode = pCurNode;
			timesParentVisited = pParentNode->GetData().NumVisits;

			// Reset best value
			bestValue = 0;
			pBestNode = 0;

			// Find child with most promising value
			for(TREE_NODE::ID i=0; i < pParentNode->GetNumChildren(); i++)
			{
				// Get child data
				pCurNode = pParentNode->GetChildNode(i);
				curData = pCurNode->GetData();

				// Calculate value of child
				nextValue = curData.NodeValue + UCL_COEFFICIENT*sqrt(log(timesParentVisited)/curData.NumVisits);

				// Check if this value is better
				if(bestValue < nextValue)
				{
					// Set this node as best node if it's better
					bestValue = nextValue;
					pBestNode = pCurNode;
				}
			}

			// Repeat while loop until best node is also a leaf
			pCurNode = pBestNode;
		}

		// Set best node as the one used for further exploration
		*SelectedNode = pBestNode;

		return 1;
	}
}
