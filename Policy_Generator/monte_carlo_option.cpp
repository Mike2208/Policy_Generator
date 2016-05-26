#include "monte_carlo_option.h"
#include "robot_navigation.h"
#include "algorithm_d_star.h"
#include "map_height_map.h"

#include <array>
#include <algorithm>    // std::sort

MonteCarloOption::MonteCarloOption() : _pLastPos(NULL), _pBestDestNode(NULL), _CompareClass()
{

}

int MonteCarloOption::PerformMonteCarlo(const OccupancyGridMap &OGMap, const POS_2D &StartPos, const POS_2D &Destination)
{
	// Check trivial situation
	if(StartPos == Destination)
	{
		return 0;
	}

	// Initialize parameters
	this->_pTmpProbMap = &(OGMap.GetMapData());
	this->_pBestDestNode = NULL;
	this->_DestPosition = StartPos;

	this->_MoveCost = 1;
	this->_ObservationCost = 1;

	this->_TmpDStarMap.ResizeMap(OGMap.GetMapHeight(), OGMap.GetMapWidth());
	this->_TmpLogMap.ResizeMap(OGMap.GetMapHeight(), OGMap.GetMapWidth());
	this->_TmpVisitMap.ResetMap(OGMap.GetMapHeight(), OGMap.GetMapWidth(), 0);

	// Calculate log map
	OccupancyGridMap::CalculateLogMapFromOGM(OGMap.GetMapData(), this->_TmpLogMap);

	// Free start and dest position
	this->_TmpLogMap.SetPixel(StartPos, OGM_LOG_CELL_FREE);
	this->_TmpLogMap.SetPixel(Destination, OGM_LOG_CELL_FREE);

	// Setup Monte Carlo Tree
	TreeClass<MCO_NODE_DATA> mcTree;

	{
		// Add observation action to clear start pos
		MCO_NODE_DATA rootData;
		rootData.Certainty = 0;
		rootData.CostToDest = std::numeric_limits<MONTE_CARLO_OPTION::ACTION_COST_TYPE>::infinity();
		rootData.ExpectedLength = std::numeric_limits<MONTE_CARLO_OPTION::NODE_EXPECTEDLENGTH_TYPE>::infinity();
		rootData.NewCell = StartPos;
		rootData.NodeValue = 0;
		rootData.NumVisits = 1;
		rootData.Action.SetActionObserve();
		mcTree.ResetTree(rootData);

		// Add observation result to set robot in start pos
		rootData.Action.SetActionResult_FreeCell();
		mcTree.GetRoot()->AddChild(rootData);

		// Add Move action to set bot to start position
		rootData.Action.SetActionMove();
		mcTree.GetRoot()->GetChildNode(0)->AddChild(rootData);
	}

	// Possibly update maps here and then set move node as new parent

	// Continue until certainty reaches limit and node was reached with one iteration
	while(mcTree.GetRoot()->GetData().Certainty < 0.5f || this->_pBestDestNode == NULL)
	{
		MCO_TREE_NODE *pCurNode;

		// Select a node to expand
		this->Selection(mcTree, &pCurNode, this);

		// Expand node
		this->Expansion(mcTree, *pCurNode, this);

		// Simulate expanded node
		this->Simulation(mcTree, pCurNode, this);

		// Backtrack until parent is reached
		this->Backtrack(mcTree, pCurNode, this);
	}

	return 1;
}

int MonteCarloOption::Selection(MCO_TREE_CLASS &Tree, MCO_TREE_NODE **SelectedNode, void *ExtraData)
{
	// Set class
	MonteCarloOption *pClass = static_cast<MonteCarloOption *>(ExtraData);

	// Get root node
	MCO_TREE_NODE *pBestNode = Tree.GetRoot();		// Start at root
	MCO_NODE_VALUE_TYPE bestValue = pBestNode->GetData().NodeValue;

	// Reset position to start
	pClass->_pLastPos = &(pBestNode->GetData().NewCell);

	// Add node data
	pClass->UpdateTmpDataWithNodeData(pBestNode->GetData());

	// Continue through tree until leaf is reached
	while(!pBestNode->IsLeaf())
	{
		MCO_TREE_NODE * const pParentNode = pBestNode;

		// Start first node
		pBestNode = pParentNode->GetChildNode(0);
		bestValue = pBestNode->GetData().NodeValue;

		// Get node value of all children and select best one
		for(TREE_NODE::ID i=1; i<pParentNode->GetNumChildren(); i++)
		{
			// Get child node
			MCO_TREE_NODE *const pCurNode = pParentNode->GetChildNode(i);

			// Compare current value to best value
			MCO_NODE_VALUE_TYPE curValue = pCurNode->GetData().NodeValue;
			if(bestValue < curValue)
			{
				// Set this node as best node
				bestValue = curValue;
				pBestNode = pCurNode;
			}
		}

		// Change tmp map data and current position
		pClass->UpdateTmpDataWithNodeData(pBestNode->GetData());

		// Continue until pBestNode is a leaf (has no more children)
	}

	// return best node
	*SelectedNode = pBestNode;

	return 1;
}

int MonteCarloOption::Expansion(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToExpand, void *ExtraData)
{
	// Set class
	MonteCarloOption *const pClass = static_cast<MonteCarloOption *const>(ExtraData);
	bool newNodeAdded = false;

	// Add new nodes
	for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
	{
		const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(NodeToExpand.GetData().NewCell, i);

		// Get probability of position being free
		OGM_LOG_TYPE curProb;
		if(pClass->_TmpLogMap.GetPixel(adjacentPos, curProb) < 0)
			continue;			// Position doesn't exist, continue with next pos

		// Save that a new node was added
		newNodeAdded = true;

		if(curProb <= OGM_LOG_CELL_FREE || adjacentPos == pClass->_DestPosition)
		{
			// if cell is 100% known to be free, add movement option to this position
			NodeToExpand.AddChild(MCO_NODE_DATA(0, 0, 0, 0, 0, adjacentPos, MONTE_CARLO_OPTION::NODE_ACTION_MOVE));
		}
		else if(curProb < OGM_LOG_CELL_OCCUPIED)
		{
			// if cell status is unknown, add observation action
			NodeToExpand.AddChild(MCO_NODE_DATA(0, 0, 0, 0, 0, adjacentPos, MONTE_CARLO_OPTION::NODE_ACTION_OBSERVATION));
		}
	}

	// Return whether new node was added
	if(!newNodeAdded)
		return 0;

	return 1;
}

int MonteCarloOption::Simulation(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE *ParentOfNodesToSimulate, void *ExtraData)
{
	// Set class
	MonteCarloOption *pClass = static_cast<MonteCarloOption *>(ExtraData);
	bool pathToDestFound = false;

	// Go through all children
	for(TREE_NODE::ID i=0; i<ParentOfNodesToSimulate->GetNumChildren(); i++)
	{
		MCO_TREE_NODE *pNodeToSimulate = ParentOfNodesToSimulate->GetChildNode(i);

		// Check if new node is an observation action
		if(pNodeToSimulate->GetData().Action.IsObserveAction())
		{
			// Add both possible observation results
			MCO_NODE_DATA curData = pNodeToSimulate->GetData();

			// Add occupied observation and simulate
			curData.Action.SetActionResult_OccupiedCell();
			pNodeToSimulate->AddChild(curData);
			if(pClass->SimulateNode(Tree, *pNodeToSimulate->GetChildNode(0)) == 1)		// Run simulation function
				pathToDestFound = true;

			// Add free observation and simulate
			curData.Action.SetActionResult_FreeCell();
			pNodeToSimulate->AddChild(curData);
			if(pClass->SimulateNode(Tree, *pNodeToSimulate->GetChildNode(1)) == 1)		// Run simulation function
				pathToDestFound = true;

			// Perform Backtrack to this node
			pClass->CalculateNodeValueFromBacktrack(Tree, *pNodeToSimulate);
		}
		else		// for Move or observation result, just simulate
		{
			// Run simulation function
			if(pClass->SimulateNode(Tree, *pNodeToSimulate) == 1)
				pathToDestFound = true;
		}
	}

	// Return whether one of the nodes reaches destination
	if(!pathToDestFound)
		return 0;

	return 1;
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

		// Reset changes made to tmpMaps during selection phase
		pClass->ResetTmpDataWithNodeData(pCurNode->GetData());

		// Move to parent
		pCurNode = pCurNode->GetParent();
	}

	return 1;
}

int MonteCarloOption::SimulateNode(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToSimulate)
{
	return this->SimulateNode_MaxReliability(Tree, NodeToSimulate);
}

int MonteCarloOption::SimulateNode_MaxReliability(const MCO_TREE_CLASS &Tree, MCO_TREE_NODE &NodeToSimulate)
{
	// Get node to simulate
	MCO_NODE_DATA curData = NodeToSimulate.GetData();
	OGM_LOG_TYPE oldProb = 0;
	const POS_2D *pOldPos = NULL;
	unsigned int oldNumVisits = 0;
	bool destIsReachable;			// Stores whether simulation can reach destination from this node

	// Add action of node to maps
	if(curData.Action.IsObserveResult())			// If this node contains observation, add observed probability to map
	{
		// Save old data
		oldProb = this->_TmpLogMap.GetPixel(curData.NewCell);

		// Set new data with result of node
		if(curData.Action.IsCellOccupied())
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

	// Update D* Map
	AlgorithmDStar::CalculateDStarMap(this->_TmpLogMap, this->_DestPosition, OGM_LOG_CELL_FREE, OGM_LOG_CELL_OCCUPIED, this->_TmpDStarMap);

	// Run simulation

	// Store expected reliability
	curData.Certainty = expf(-this->_TmpDStarMap.GetPixel(*this->_pLastPos));

	// Calculate expected length by following path with maximum reliability
	unsigned int expectedLength; // = curData.ExpectedLength;
	destIsReachable = (HeightMap::FindMinCostPathLength(this->_TmpDStarMap, *this->_pLastPos, this->_DestPosition, expectedLength, &OGM_LOG_CELL_OCCUPIED) > 0 ? 1:0);
	curData.ExpectedLength = expectedLength;			// Hack to convert from / to unsigned int

	// Calculate value of node while taking reachability of goal into account
	if(destIsReachable)
		this->CalculateNodeValueFromSimulation(curData, curData.NodeValue);
	else
	{
		// Set node as unreachable
		curData.NodeValue = MONTE_CARLO_OPTION::NODE_VALUE_DEAD_END;
		curData.ExpectedLength = MONTE_CARLO_OPTION::NODE_EXPECTEDLENGTH_MAX;
		curData.Certainty = 0;
	}

	// Set rest of node data
	curData.NumVisits = 1;

	// Save new node data
	NodeToSimulate.SetData(curData);

	// Reset map data
	if(curData.Action.IsObserveResult())
	{
		// Reverse probability map
		this->_TmpLogMap.SetPixel(curData.NewCell, oldProb);
	}
	else
	{
		// Reverse position map and lastpos
		this->_TmpVisitMap.SetPixel(curData.NewCell, oldNumVisits);
		this->_pLastPos = pOldPos;

		// Save if this node has reached the destination
		if(curData.NewCell == this->_DestPosition)
		{
			this->_pBestDestNode = &NodeToSimulate;
		}
	}

	// Return whether dest is not reachable from here
	if(!destIsReachable)
		return 0;

	return 1;
}

void MonteCarloOption::CalculateNodeValueFromSimulation(const MCO_NODE_DATA &NodeData, MONTE_CARLO_OPTION::NODE_VALUE_TYPE &Value)
{
	// Calculate cost reliability ration
	Value = NodeData.Certainty/NodeData.ExpectedLength;
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

	// If no children where found, run a simulation
	if(CurBacktrackNode.GetNumChildren() == 0)
	{
		this->SimulateNode(Tree, CurBacktrackNode);
	}

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
		if(curData.Action.IsObserveAction())
		{
			curCost += (maxCertainty-curCertainty)*(curData.CostToDest+this->_ObservationCost)/2;

			if(curData.OccupiedCell)
				curCertainty += ((maxCertainty-curCertainty)*curData.Certainty*(1-this->_pTmpProbMap->GetPixel(curData.NewCell))/2);
			else
				curCertainty += ((maxCertainty-curCertainty)*curData.Certainty*(this->_pTmpProbMap->GetPixel(curData.NewCell))/2);
		}
		else
		{
			curLength += 1*(maxCertainty-curCertainty);

			curCost += (maxCertainty-curCertainty)*(curData.CostToDest+this->_MoveCost);
			curCertainty += (maxCertainty-curCertainty)*curData.Certainty;
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

void MonteCarloOption::UpdateTmpDataWithNodeData(const MCO_NODE_DATA &NodeData)
{
	if(NodeData.Observe)
	{
		// At observation action, update log map with new cell data and recalculate D* map
		if(NodeData.OccupiedCell)
			this->_TmpLogMap.SetPixel(NodeData.NewCell, OGM_LOG_CELL_OCCUPIED);
		else
			this->_TmpLogMap.SetPixel(NodeData.NewCell, OGM_LOG_CELL_FREE);

		AlgorithmDStar::CalculateDStarMap(this->_TmpLogMap, this->_DestPosition, OGM_LOG_CELL_FREE, OGM_LOG_CELL_OCCUPIED, this->_TmpDStarMap);
	}
	else
	{
		// At move action, update current position and visit map
		this->_pLastPos = &NodeData.NewCell;
		this->_TmpVisitMap.SetPixel(NodeData.NewCell, this->_TmpVisitMap.GetPixel(NodeData.NewCell)+1);
	}
}

void MonteCarloOption::ResetTmpDataWithNodeData(const MCO_NODE_DATA &NodeData)
{
	if(NodeData.Observe)
	{
		// At observation action, reset log map and recalculate D* map
		this->_TmpLogMap.SetPixel(NodeData.NewCell, OccupancyGridMap::CalculateLogValue(this->_pTmpProbMap->GetPixel(NodeData.NewCell)));
		AlgorithmDStar::CalculateDStarMap(this->_TmpLogMap, this->_DestPosition, OGM_LOG_CELL_FREE, OGM_LOG_CELL_OCCUPIED, this->_TmpDStarMap);
	}
	else
	{
		// At Move action, decrease number of visits
		this->_TmpVisitMap.SetPixel(NodeData.NewCell, this->_TmpVisitMap.GetPixel(NodeData.NewCell)-1);
	}
}
