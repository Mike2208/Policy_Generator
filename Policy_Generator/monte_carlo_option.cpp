#include "monte_carlo_option.h"
#include "robot_navigation.h"
#include "algorithm_d_star.h"
#include "map_height_map.h"

#include <array>
#include <algorithm>    // std::sort

MonteCarloOption::MonteCarloOption() : _CompareClass()
{

}

int MonteCarloOption::PerformMonteCarlo(const OccupancyGridMap &OGMap, const POS_2D &StartPos, const POS_2D &Destination)
{
	// Initialize parameters
	this->_pTmpProbMap = &(OGMap.GetMapData());
	this->_DestPosition = StartPos;

	this->_MoveCost = 1;
	this->_ObservationCost = 1;

	this->_TmpDStarMap.ResizeMap(OGMap.GetMapHeight(), OGMap.GetMapWidth());
	this->_TmpLogMap.ResizeMap(OGMap.GetMapHeight(), OGMap.GetMapWidth());
	this->_TmpVisitMap.ResizeMap(OGMap.GetMapHeight(), OGMap.GetMapWidth());

	// Set functions for Monte Carlo
	MonteCarlo<MCO_NODE_DATA> algorithm;

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

int MonteCarloOption::Backtrack(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE *ParentOfLeafsToBacktrack, void *ExtraData)
{
	// Set class
	MonteCarloOption *pClass = static_cast<MonteCarloOption *>(ExtraData);

	// Set pointer node to backtrack to
	MCO_TREE_NODE *pCurNode = ParentOfLeafsToBacktrack;

	// Go through all ancestors and backtrack every time
	while(pCurNode != NULL)
	{
		// Calculate new value
		pClass->CalculateNodeValueFromBacktrack(Tree, *pCurNode);

		// Move to parent
		pCurNode = pCurNode->GetParent();
	}

	return 1;
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
	curData.Certainty = expf(-this->_TmpDStarMap.GetPixel(*this->_pLastPos));

	// Calculate expected length by following path with maximum reliability
	MONTE_CARLO_OPTION::NODE_EXPECTEDLENGTH_TYPE expectedLength = curData.ExpectedLength;
	HeightMap::FindMinCostPathLength(this->_TmpDStarMap, *this->_pLastPos, this->_DestPosition, expectedLength);
	curData.ExpectedLength = expectedLength;			// Hack to convert from / to float

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
	Value = NodeData.ExpectedLength/NodeData.Certainty;
}

void MonteCarloOption::CalculateNodeValueFromBacktrack(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &CurBacktrackNode)
{
	const MONTE_CARLO_OPTION::NODE_CERTAINTY_TYPE maxCertainty = 1;

	MCO_NODE_DATA data = CurBacktrackNode.GetData();
	MONTE_CARLO_OPTION::NODE_CERTAINTY_TYPE &curCertainty = data.Certainty;
	MONTE_CARLO_OPTION::NODE_EXPECTEDLENGTH_TYPE &curLength = data.ExpectedLength;
	MONTE_CARLO_OPTION::ACTION_COST_TYPE &curCost = data.CostToDest;

	curCertainty = 0;
	curLength = 0;
	curCost = 0;

	// get all children node pointers
	std::vector<MCO_TREE_NODE *> children(CurBacktrackNode.GetNumChildren());
	for(TREE_NODE::ID i = 0; i<CurBacktrackNode.GetNumChildren(); i++)
	{
		children[i] = CurBacktrackNode.GetChildNode(i);
	}

	// Sort all children according to expected length
	std::sort(children.begin(), children.end(), this->_CompareClass);

	// Go through all nodes and update certainty and expected length
	for(unsigned int i=0; i<children.size(); i++)
	{
		const MCO_TREE_NODE &curChild = *(children[i]);
		const MCO_NODE_DATA &curData = curChild.GetData();

		// Update expected length
		curLength += curData.ExpectedLength*(maxCertainty-curCertainty);

		// Update certainty ( if observation action, divide by two to ensure action isn't counted twice, as observation action creates two nodes )
		if(curData.Observe)
		{
			curCost += (maxCertainty-curCertainty)*(curData.CostToDest+this->_ObservationCost)/2;
			curCertainty += ((maxCertainty-curCertainty)*curData.Certainty*(this->_pTmpProbMap->GetPixel(curData.NewCell)))/2;
		}
		else
		{
			curCost += (maxCertainty-curCertainty)*(curData.CostToDest+this->_MoveCost);
			curCertainty += curData.Certainty;
		}

		// Check that certainty less than 1
		if(curCertainty >= maxCertainty)
		{
			curCertainty = maxCertainty;
			break;
		}
	}

	// Update node data
	//data.Certainty = curCertainty;
	//data.ExpectedLength = curLength;
	//data.CostToDest = curCost;

	// Add one number of visit
	data.NumVisits++;

	// Calculate new value of node ( use same calcualtion as before)
	this->CalculateNodeValueFromSimulation(data, data.NodeValue);

	// Save new data
	CurBacktrackNode.SetData(data);

	return;
}
