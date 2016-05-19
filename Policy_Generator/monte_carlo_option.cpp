#include "monte_carlo_option.h"
#include "robot_navigation.h"

#include <array>

MonteCarloOption::MonteCarloOption()
{

}

int MonteCarloOption::PerformMonteCarlo(const OccupancyGridMap &OGMap)
{

}

int MonteCarloOption::Selection(const TREE_CLASS &Tree, const TREE_NODE *SelectedNode, void *ExtraData)
{
	// Set class
	MonteCarloOption *pClass = static_cast<MonteCarloOption *>(ExtraData);

	// Get root node
	const TREE_NODE *pBestNode = Tree.GetRoot();		// Start at root
	NODE_VALUE_TYPE bestValue = pBestNode->GetData().NodeValue;

	// Continue through tree until leaf is reached
	while(!pBestNode->IsLeaf())
	{
		const TREE_NODE * const pParentNode = pBestNode;
		// Get node value of all children and select best one
		for(TREE_NODE::ID i=0; i<pParentNode->GetNumChildren(); i++)
		{
			// Get child node
			const TREE_NODE *const pCurNode = pParentNode->GetChildNode(i);

			// Compare current value to best value
			NODE_VALUE_TYPE curValue = pCurNode->GetData().NodeValue;
			if(bestValue < curValue)
			{
				// Set this node as best node
				bestValue = curValue;
				pBestNode = pCurNode;
			}
		}

		// Continue until pBestNode is a leaf (has no more children)
	}

	// return best node
	SelectedNode = pBestNode;

	return 1;
}

int MonteCarloOption::Expansion(const TREE_CLASS &Tree, TREE_NODE &NodeToExpand, void *ExtraData)
{
	// Set class
	MonteCarloOption *const pClass = static_cast<MonteCarloOption *const>(ExtraData);

	// Go through this node and all ancestors
	TREE_NODE *pCurNode = &NodeToExpand;
	do
	{
		// Add data of this ancestor to temp maps of pClass ( they will be used later in the simulation )
		if(pCurNode->GetData().Observe)
		{
			// If this node corresponds to an observation option, update probability with observation
			if(pCurNode->GetData().OccupiedCell)
				pClass->_TmpProbMap.SetPixel(pCurNode->GetData().NewCell, OGM_CELL_OCCUPIED);
			else
				pClass->_TmpProbMap.SetPixel(pCurNode->GetData().NewCell, OGM_CELL_FREE);
		}
		else
		{
			// Increment number of visits by one
			pClass->_TmpVisitMap.SetPixel(pCurNode->GetData().NewCell, pClass->_TmpVisitMap.GetPixel(pCurNode->GetData().NewCell)+1);
		}

		// Get next ancestor
		pCurNode = pCurNode->GetParent();
	}
	while(pCurNode != NULL);

	// Add new nodes
	for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
	{
		const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(NodeToExpand.GetData().NewCell, i);

		// Get probability of position being free
		OGM_TYPE curProb = pClass->_TmpProbMap.GetPixel(adjacentPos);

		if(curProb == OGM_CELL_FREE)
		{
			// if cell is 100% known to be free, add movement option to this position
			NodeToExpand.AddChild(NODE_DATA(0, 0, 0, 0, adjacentPos, 0, 0));
		}
		else if(curProb != OGM_CELL_OCCUPIED)
		{
			// if cell status is unknown, add observation action
			NodeToExpand.AddChild(NODE_DATA(0, 0, 0, 0, adjacentPos, 1, 0));
			NodeToExpand.AddChild(NODE_DATA(0, 0, 0, 0, adjacentPos, 1, 1));
		}
	}

	return 1;

//	// Check whether adjacent nodes have previously been observed by ancestors of this node
//	for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
//	{
//		const POS_2D curAdjacentPos = RobotNavigation::GetNextMovementPosition(NodeToExpand.GetData().NewCell, i);
//		cellPreviouslyObserved = false;

//		// Go through all ancestors of this node and check whether adjacent node was observed
//		do
//		{
//			// Compare observed position of ancestor to adajcent one
//			if(pCurNode->GetData().NewCell == curAdjacentPos)
//			{
//				if(pCurNode->GetData().Observe == false)	// This means position was moved to, not observed
//				{
//					// Count how often this position was traversed in the past
//					numPreviousPasses++;
//				}
//				else
//				{
//					// If this position was observed in this node, stop ( there wont be anymore visits to this cell because it needs to be observed before visiting)
//					cellPreviouslyObserved = true;
//					break;
//				}
//			}

//			// Next ancestor
//			pCurNode = pCurNode->GetParent();
//		}
//		while(pCurNode != NULL);

//		// If adjacent cell has already been checked and cell is free, add a branch that moves here
//		if(cellPreviouslyObserved)
//		{
//			if(!pCurNode->GetData().OccupiedCell)
//				NodeToExpand.AddChild(NODE_DATA(0, numPreviousPasses, 0, 0, curAdjacentPos, 0, 0));
//		}
//		else
//		{
//			// Else if cell wasn't observe, add two options, observe and free, and observe and occupied
//			NodeToExpand.AddChild(NODE_DATA(0, 0, 0, 0, curAdjacentPos, 1, 0));
//			NodeToExpand.AddChild(NODE_DATA(0, 0, 0, 0, curAdjacentPos, 1, 1));
//		}
//	}
}

int MonteCarloOption::Simulation(const TREE_CLASS &Tree, TREE_NODE *ParentOfNodesToSimulate, void *ExtraData)
{
	// Set class
	MonteCarloOption *pClass = static_cast<MonteCarloOption *>(ExtraData);
}

int MonteCarloOption::Backtrack(const TREE_CLASS &Tree, TREE_NODE *LeafToBacktrack, void *ExtraData)
{

}
