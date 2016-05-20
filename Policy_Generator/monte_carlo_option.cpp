#include "monte_carlo_option.h"
#include "robot_navigation.h"
#include "algorithm_d_star.h"
#include "map_height_map.h"

#include <array>

MonteCarloOption::MonteCarloOption()
{

}

int MonteCarloOption::PerformMonteCarlo(const OccupancyGridMap &OGMap)
{

}

int MonteCarloOption::Selection(const MCO_TREE_CLASS &Tree, const MCO_TREE_NODE *SelectedNode, void *ExtraData)
{
	// Set class
	MonteCarloOption *pClass = static_cast<MonteCarloOption *>(ExtraData);

	// Get root node
	const MCO_TREE_NODE *pBestNode = Tree.GetRoot();		// Start at root
	MCO_NODE_VALUE_TYPE bestValue = pBestNode->GetData().NodeValue;

	// Continue through tree until leaf is reached
	while(!pBestNode->IsLeaf())
	{
		const MCO_TREE_NODE * const pParentNode = pBestNode;
		// Get node value of all children and select best one
		for(TREE_NODE::ID i=0; i<pParentNode->GetNumChildren(); i++)
		{
			// Get child node
			const MCO_TREE_NODE *const pCurNode = pParentNode->GetChildNode(i);

			// Compare current value to best value
			MCO_NODE_VALUE_TYPE curValue = pCurNode->GetData().NodeValue;
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

int MonteCarloOption::Expansion(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToExpand, void *ExtraData)
{
	// Set class
	MonteCarloOption *const pClass = static_cast<MonteCarloOption *const>(ExtraData);

	// Go through this node and all ancestors
	bool curPosFound = false;		// Search for last position we moved to
	pClass->_pLastPos = &(Tree.GetRoot()->GetData().NewCell);		// Set first cell visited as current bot position

	MCO_TREE_NODE *pCurNode = &NodeToExpand;
	do
	{
		// Add data of this ancestor to temp maps of pClass ( they will be used later in the simulation )
		if(pCurNode->GetData().Observe)
		{
			// If this node corresponds to an observation option, update probability with observation
			if(pCurNode->GetData().OccupiedCell)
				pClass->_TmpLogMap.SetPixel(pCurNode->GetData().NewCell, OGM_LOG_CELL_OCCUPIED);
			else
				pClass->_TmpLogMap.SetPixel(pCurNode->GetData().NewCell, OGM_LOG_CELL_FREE);
		}
		else
		{
			// Increment number of visits by one
			pClass->_TmpVisitMap.SetPixel(pCurNode->GetData().NewCell, pClass->_TmpVisitMap.GetPixel(pCurNode->GetData().NewCell)+1);

			// Check if this was last position bot moved to
			if(!curPosFound)
			{
				pClass->_pLastPos = &(pCurNode->GetData().NewCell);
				curPosFound = true;
			}
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
		OGM_LOG_TYPE curProb = pClass->_TmpLogMap.GetPixel(adjacentPos);

		if(curProb <= OGM_LOG_CELL_FREE)
		{
			// if cell is 100% known to be free, add movement option to this position
			NodeToExpand.AddChild(MCO_NODE_DATA(0, 0, 0, 0, adjacentPos, 0, 0));
		}
		else if(curProb >= OGM_LOG_CELL_OCCUPIED)
		{
			// if cell status is unknown, add observation action
			NodeToExpand.AddChild(MCO_NODE_DATA(0, 0, 0, 0, adjacentPos, 1, 0));
			NodeToExpand.AddChild(MCO_NODE_DATA(0, 0, 0, 0, adjacentPos, 1, 1));
		}
	}

	return 1;
}

int MonteCarloOption::Simulation(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE *ParentOfNodesToSimulate, void *ExtraData)
{
	// Set class
	MonteCarloOption *pClass = static_cast<MonteCarloOption *>(ExtraData);

	// Go through all children
	for(TREE_NODE::ID i=0; i<ParentOfNodesToSimulate->GetNumChildren(); i++)
	{
		// Run simulation function
		pClass->SimulateNode_MaxReliability(Tree, *ParentOfNodesToSimulate->GetChildNode(i));
	}
}

int MonteCarloOption::Backtrack(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE *LeafToBacktrack, void *ExtraData)
{

}


int MonteCarloOption::SimulateNode_MaxReliability(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToSimulate)
{
	// Get node to simulate
	MCO_NODE_DATA curData = NodeToSimulate.GetData();
	OGM_LOG_TYPE oldProb;
	const POS_2D *pOldPos;
	unsigned int oldNumVisits;

	// Add action of node to maps
	if(curData.Observe)			// If last action was observation, add observed probability to map
	{
		// Save old data
		oldProb = this->_TmpLogMap.GetPixel(curData.NewCell);

		// Set new data with result of node
		if(curData.OccupiedCell)
			this->_TmpLogMap.SetPixel(curData.NewCell, OGM_LOG_CELL_OCCUPIED);
		else
			this->_TmpLogMap.SetPixel(curData.NewCell, OGM_LOG_CELL_FREE);
	}
	else		// If last action was movement, set new position
	{
		// Save old data
		pOldPos = this->_pLastPos;
		oldNumVisits = this->_TmpVisitMap.GetPixel(curData.NewCell);

		// Set new data
		this->_TmpVisitMap.SetPixel(curData.NewCell, oldNumVisits + 1);
		this->_pLastPos = &(curData.NewCell);
	}

	// Run simulation
	AlgorithmDStar::CalculateDStarMap(this->_TmpLogMap, this->_DestPosition, OGM_LOG_CELL_FREE, OGM_LOG_CELL_OCCUPIED, this->_TmpDStarMap);

	// Store expected reliability
	curData.Certainty = this->_TmpDStarMap.GetPixel(*this->_pLastPos);

	// Calculate expected length by following path with maximum reliability
	HeightMap::FindMinCostPathLength(this->_TmpDStarMap, *this->_pLastPos, this->_DestPosition, (float)(curData.ExpectedLength));

	// Calculate value of node
	this->CalculateNodeValueFromSimulation(curData, curData.NodeValue);

	// Set rest of node data
	curData.NumVisits = 1;

	// Save new node data
	NodeToSimulate.SetData(curData);

	// Reset map data
	if(curData.Observe)
	{
		// Reverse probability map
		this->_TmpLogMap.SetPixel(curData.NewCell, oldProb);
	}
	else
	{
		// Reverse position map and lastpos
		this->_TmpVisitMap.SetPixel(curData.NewCell, oldNumVisits);
		this->_pLastPos = pOldPos;
	}
}

void MonteCarloOption::CalculateNodeValueFromSimulation(const MCO_NODE_DATA &NodeData, MONTE_CARLO_OPTION::NODE_VALUE_TYPE &Value)
{
	// Calculate cost reliability ration
	Value = NodeData.ExpectedLength/expf(-NodeData.Certainty);
}
