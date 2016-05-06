#include "obstacle_path_finder.h"
#include "obstacle_connections.h"
#include <climits>
#include <cmath>

ObstaclePathFinder::ObstaclePathFinder()
{

}

int ObstaclePathFinder::FindAllPaths(const Map_BoolType &ObstacleMap, const std::vector<POS_2D> &Obstacles, const POS_2D &Start, const POS_2D &Destination)
{
	ObstacleConnections connections;

	// Get obstacles that are next to each other
	FindAdjoiningObstacles(ObstacleMap, Obstacles, Start, Destination, connections);

	// Plan all possible paths around obstacles

	// Variables to keep track of robot position
	unsigned int	curObstacleID;			// Id of obstacle that we are currently circling
	ROT_ANGLE_TYPE	curAngle;				// Current angle to current obstacle
	bool			ccwRotation = false;	// Are we currently moving clockwise(0) or counter-clockwise(1) around obstacle. This is used to prevent robot from turning around

	const unsigned int startID = Obstacles.size() - (MAP_CONST_EDGE_IDS::NumExtraIDs-1-MAP_CONST_EDGE_IDS::StartID_Add);
	const unsigned int destID = Obstacles.size() - (MAP_CONST_EDGE_IDS::NumExtraIDs-1-MAP_CONST_EDGE_IDS::DestID_Add);

	// Begin at start, and calculate angle to destination
	curObstacleID = startID;

	curAngle = atan2(connections[destID].Position.Y - connections[startID].Position.Y, connections[destID].Position.X-connections[startID].Position.X);

	// From here, get ID of obstacles that is connected clockwise and ccw of this angle
	std::vector<std::vector<OBSTACLE_PATH_FINDER::OBSTACLE_CONNECTION>> paths;
	OBSTACLE_PATH_FINDER::OBSTACLE_CONNECTION curConnection;
	paths.resize(2);

	// Get next possible path in clockwise and ccw direction
	curConnection.ObstacleID = curObstacleID;
	curConnection.ConnectedObstacleID = connections.GetNextObstacleInRotDir(curObstacleID, curAngle, ccwRotation);
	paths[0].push_back(curConnection);

	curConnection.ConnectedObstacleID = connections.GetNextObstacleInRotDir(curObstacleID, curAngle, !ccwRotation);
	paths[1].push_back(curConnection);

	// Continue finding all possible paths recursively for both of these paths
	ObstaclePathFinder::FindNextPathStep(connections, ccwRotation, 0, paths);
	ObstaclePathFinder::FindNextPathStep(connections, !ccwRotation, 1, paths);

	return 1;
}

int ObstaclePathFinder::FindAdjoiningObstacles(const Map_BoolType &ObstacleMap, const std::vector<POS_2D> &Obstacles, const POS_2D &Start, const POS_2D &Destination, ObstacleConnections &AdjoiningObstacles)
{
	const unsigned int numObstacles = Obstacles.size();

	OBSTACLE_PATH_FINDER::CONNECTED_IDS connectedIDs;													// array conatining all obstacles that border each other
	connectedIDs.resize(numObstacles + MAP_CONST_EDGE_IDS::NumExtraIDs);		// create enough space for all IDs

	// positions that need to be cheked
	std::vector<POS_2D> posToCheck;

	// Create map with empty positions
	OBSTACLE_PATH_FINDER::MAP_ID_DIST curIdDist = { MAP_CONST_EDGE_IDS::EmptyID, MAP_CONST_EDGE_IDS::EmptyDist };		// Empty distance
	Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST> idDistMap(ObstacleMap.GetHeight(), ObstacleMap.GetWidth(), curIdDist);	// Fill distance map with empty variables

	// Array to keep track of obstacle positions
	//		This will search for the center of the obstacle
	std::vector<OBSTACLE_PATH_FINDER::AVERAGE_OBSTACLE_POS> obstaclePositions;
	obstaclePositions.resize(Obstacles.size());

	for(unsigned int i=0; i<Obstacles.size(); i++)
	{
		obstaclePositions[i].AveragePos = Obstacles[i];
		obstaclePositions[i].NumObstaclesInAverage = 1;
	}

	// Set Obstacle distances to 0 and add them to the array. They will be checked later
	posToCheck.resize(Obstacles.size()+2);
	for(unsigned int i=0; i<numObstacles; i++)
	{
		curIdDist.Distance = 0;
		curIdDist.ID = i;

		idDistMap.SetPixel(Obstacles[i], curIdDist);

		posToCheck[i] = Obstacles[i];
	}

	// Also add start and end to positions to same array
	curIdDist.Distance = 0;
	curIdDist.ID = Obstacles.size() + MAP_CONST_EDGE_IDS::StartID_Add;
	idDistMap.SetPixel(Start, curIdDist);
	posToCheck[curIdDist.ID] = Start;

	curIdDist.Distance = 0;
	curIdDist.ID = Obstacles.size() + MAP_CONST_EDGE_IDS::DestID_Add;
	idDistMap.SetPixel(Destination, curIdDist);
	posToCheck[curIdDist.ID] = Destination;


	// Now go through all of them and register adjoining Obstacles
	unsigned int curPosNum = 0;
	while(curPosNum < posToCheck.size())
	{
		const POS_2D curPos = posToCheck[curPosNum];		// This needs to be copied as the vector posToCheck will be changed later on, making any reference point to the wrong destination

		// Check this and adjacent pixels for borders
		//		This will also update posToCheck with the next positions to check
		ObstaclePathFinder::CompareAllAdjacentPositions(ObstacleMap, idDistMap, curPos, connectedIDs, posToCheck, obstaclePositions);

		curPosNum++;
		//posToCheck.erase(posToCheck.begin()+curPosNum);
	}

	// For debugging, create an image
	//idDistMap.PrintMapToFile("/tmp/IdDistMap.pbm", 500);

	// Order the obstacles according to the angle between each other
	AdjoiningObstacles.CreateNewEmptyObstacles(Obstacles.size()+6);		// Add 6 obstacles for edges and start/destination
	for(unsigned int i=0; i<Obstacles.size(); i++)			// Set the positions of all obstacles
		AdjoiningObstacles.SetObstaclePos(i, obstaclePositions[i].AveragePos);
	AdjoiningObstacles.SetObstaclePos(Obstacles.size()+MAP_CONST_EDGE_IDS::BottomID_Add, POS_2D(ObstacleMap.GetWidth()/2,0));		// Add bottom position
	AdjoiningObstacles.SetObstaclePos(Obstacles.size()+MAP_CONST_EDGE_IDS::BottomID_Add, POS_2D(0,ObstacleMap.GetHeight()/2));		// Add left position
	AdjoiningObstacles.SetObstaclePos(Obstacles.size()+MAP_CONST_EDGE_IDS::TopID_Add, POS_2D(ObstacleMap.GetWidth()/2,ObstacleMap.GetHeight()-1));		// Add top position
	AdjoiningObstacles.SetObstaclePos(Obstacles.size()+MAP_CONST_EDGE_IDS::TopID_Add, POS_2D(ObstacleMap.GetWidth()-1,ObstacleMap.GetHeight()/2));		// Add right position
	AdjoiningObstacles.SetObstaclePos(Obstacles.size()+MAP_CONST_EDGE_IDS::StartID_Add, Start);				// Add start position
	AdjoiningObstacles.SetObstaclePos(Obstacles.size()+MAP_CONST_EDGE_IDS::DestID_Add, Destination);		// Add destination position


	// Add connections between IDs
	for(unsigned int i=0; i<connectedIDs.size(); i++)
	{
		std::vector<OBSTACLE_PATH_FINDER::MAP_ID_DIST_POS> &curConnectedID = connectedIDs[i];

		for(unsigned int j=0; j<curConnectedID.size(); j++)
		{
			// This connects the two obstacles, calculates the angles between them, and sorts the connections according to the angle
			AdjoiningObstacles.AddConnectedObstacle(i, curConnectedID[j].ID, curConnectedID[j].Position);
		}
	}

	return 1;
}

void ObstaclePathFinder::CompareAllAdjacentPositions(const Map_BoolType &ObstacleMap, Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST> &IdDistMap, const POS_2D &CurPos, OBSTACLE_PATH_FINDER::CONNECTED_IDS &ConnectedIds, OBSTACLE_PATH_FINDER::POSTOCHECK &PosToCheck, std::vector<OBSTACLE_PATH_FINDER::AVERAGE_OBSTACLE_POS> &AverageObstaclePos)
{
	POS_2D adjacentPos;

	OBSTACLE_PATH_FINDER::MAP_ID_DIST curIdDist, adjacentIdDist;

	// Go through adjacent positions and check for border

	// Get current pixel
	IdDistMap.GetPixel(CurPos, curIdDist);

	// Check top
	adjacentPos.X = CurPos.X;
	adjacentPos.Y = CurPos.Y+1;
	if(IdDistMap.GetPixel(adjacentPos, adjacentIdDist) >= 0)		// Try ot get adjacent pixel and compare it
		ObstaclePathFinder::CompareOneAdjacentPosition(ObstacleMap, IdDistMap, curIdDist, adjacentIdDist, adjacentPos, ConnectedIds, PosToCheck, AverageObstaclePos);
	else	// If an error occured, the top was reached, add the top to  the adjacent borders
		ObstaclePathFinder::AddAdjacentEdge(curIdDist, CurPos, ConnectedIds.size() - 1 - (MAP_CONST_EDGE_IDS::NumExtraIDs-MAP_CONST_EDGE_IDS::TopID_Add), ConnectedIds);

	// Check right
	adjacentPos.X = CurPos.X+1;
	adjacentPos.Y = CurPos.Y;
	if(IdDistMap.GetPixel(adjacentPos, adjacentIdDist) >= 0)		// Try ot get adjacent pixel and compare it
		ObstaclePathFinder::CompareOneAdjacentPosition(ObstacleMap, IdDistMap, curIdDist, adjacentIdDist, adjacentPos, ConnectedIds, PosToCheck, AverageObstaclePos);
	else	// If an error occured, the right was reached, add the right to  the adjacent borders
		ObstaclePathFinder::AddAdjacentEdge(curIdDist, CurPos, ConnectedIds.size() - 1 - (MAP_CONST_EDGE_IDS::NumExtraIDs-MAP_CONST_EDGE_IDS::RightID_Add), ConnectedIds);

	// Check bottom
	adjacentPos.X = CurPos.X;
	adjacentPos.Y = CurPos.Y-1;
	if(IdDistMap.GetPixel(adjacentPos, adjacentIdDist) >= 0)		// Try ot get adjacent pixel and compare it
		ObstaclePathFinder::CompareOneAdjacentPosition(ObstacleMap, IdDistMap, curIdDist, adjacentIdDist, adjacentPos, ConnectedIds, PosToCheck, AverageObstaclePos);
	else	// If an error occured, the bottom was reached, add the bottom to  the adjacent borders
		ObstaclePathFinder::AddAdjacentEdge(curIdDist, CurPos, ConnectedIds.size() - 1 - (MAP_CONST_EDGE_IDS::NumExtraIDs-MAP_CONST_EDGE_IDS::BottomID_Add), ConnectedIds);

	// Check left
	adjacentPos.X = CurPos.X-1;
	adjacentPos.Y = CurPos.Y;
	if(IdDistMap.GetPixel(adjacentPos, adjacentIdDist) >= 0)		// Try ot get adjacent pixel and compare it
		ObstaclePathFinder::CompareOneAdjacentPosition(ObstacleMap, IdDistMap, curIdDist, adjacentIdDist, adjacentPos, ConnectedIds, PosToCheck, AverageObstaclePos);
	else	// If an error occured, the left was reached, add the left to  the adjacent borders
		ObstaclePathFinder::AddAdjacentEdge(curIdDist, CurPos, ConnectedIds.size() - 1 - (MAP_CONST_EDGE_IDS::NumExtraIDs-MAP_CONST_EDGE_IDS::RightID_Add), ConnectedIds);
}

void ObstaclePathFinder::CompareOneAdjacentPosition(const Map_BoolType &ObstacleMap, Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST> &IdDistMap, const OBSTACLE_PATH_FINDER::MAP_ID_DIST &CurIdDist, const OBSTACLE_PATH_FINDER::MAP_ID_DIST &AdjacentIdDist, const POS_2D &AdjacentPos, OBSTACLE_PATH_FINDER::CONNECTED_IDS &ConnectedIds, OBSTACLE_PATH_FINDER::POSTOCHECK &PosToCheck, std::vector<OBSTACLE_PATH_FINDER::AVERAGE_OBSTACLE_POS> &AverageObstaclePos)
{
	// Check if this new position is also an obstacle
	if(ObstacleMap.GetPixel(AdjacentPos) == OCCUPANCYGRID_DISCRETE_FULL )
	{
		// if this obstacle has already been checked, skip it
		if(AdjacentIdDist.ID != MAP_CONST_EDGE_IDS::EmptyID)
			return;			// has already been checked, nothing to do

		// If it is, set this pixel to same ID and distance 0, then check the adjacent positions
		const OBSTACLE_PATH_FINDER::MAP_ID_DIST tmpIdDist( CurIdDist.ID, 0 );
		IdDistMap.SetPixel(AdjacentPos, tmpIdDist);

		// Update average obstacle position
		OBSTACLE_PATH_FINDER::AVERAGE_OBSTACLE_POS &average = AverageObstaclePos[CurIdDist.ID];
		average.AveragePos.X = average.AveragePos.X*average.NumObstaclesInAverage + AdjacentPos.X;
		average.AveragePos.Y = average.AveragePos.Y*average.NumObstaclesInAverage + AdjacentPos.Y;
		average.NumObstaclesInAverage++;

		// Also check adjacent positions NOW to prevent errors later on
		ObstaclePathFinder::CompareAllAdjacentPositions(ObstacleMap, IdDistMap, AdjacentPos, ConnectedIds, PosToCheck, AverageObstaclePos);

		return;
	}

	// Check whether value is larger than curPixelVal or whether pixel has already been recorded
	if(AdjacentIdDist.ID == MAP_CONST_EDGE_IDS::EmptyID ||
		AdjacentIdDist.Distance > CurIdDist.Distance+1)// ||
		//(AdjacentIdDist.Distance == CurIdDist.Distance && AdjacentIdDist.ID == CurIdDist.ID))
	{
		// Update adjacent position
		const OBSTACLE_PATH_FINDER::MAP_ID_DIST tmpIdDist(CurIdDist.ID, CurIdDist.Distance+1);
		IdDistMap.SetPixel(AdjacentPos, tmpIdDist);

		// Add this to positions that need to be checked
		PosToCheck.push_back(AdjacentPos);
	}
	// could be replaced with else if(AdjacentIdDist.Distance >= CurIdDist.Distance-1) because everything above AdjacentIdDist.Distance > CurIdDist.Distance is already covered above
	else if(CurIdDist.ID != AdjacentIdDist.ID && (AdjacentIdDist.Distance == CurIdDist.Distance || AdjacentIdDist.Distance == CurIdDist.Distance-1))
	{
		// Add this border to both IDs if not yet there
		ObstaclePathFinder::AddAdjacentBorder(CurIdDist.ID, AdjacentIdDist, AdjacentPos, ConnectedIds);
	}
}

void ObstaclePathFinder::AddAdjacentBorder(const unsigned int &CurrentID, const OBSTACLE_PATH_FINDER::MAP_ID_DIST &AdjacentIdDist, const POS_2D &AdjacentPos, OBSTACLE_PATH_FINDER::CONNECTED_IDS &ConnectedIds)
{
	// Check if ID is already recorded
	for(unsigned int i=0; i<ConnectedIds[CurrentID].size(); i++)
	{
		// If adjacent ID is already stored, check if new position is closer
		if(ConnectedIds[CurrentID][i].ID == AdjacentIdDist.ID)
		{
			if(AdjacentIdDist.Distance < ConnectedIds[CurrentID][i].Distance)
			{
				// Update both this record and the one of the adjacent obstacle
				ConnectedIds[CurrentID][i].Distance = AdjacentIdDist.Distance;
				ConnectedIds[CurrentID][i].Position = AdjacentPos;

				// Find record in other obstacles array and update
				for(unsigned int j=0; j<ConnectedIds[AdjacentIdDist.ID].size(); j++)
				{
					if(ConnectedIds[AdjacentIdDist.ID][j].ID == CurrentID)
					{
						ConnectedIds[AdjacentIdDist.ID][j].Distance = AdjacentIdDist.Distance;
						ConnectedIds[AdjacentIdDist.ID][j].Position = AdjacentPos;
					}
				}
			}

			// Stop after record was found
			return;
		}
	}

	// If no record was found, add it to both arrays
	OBSTACLE_PATH_FINDER::MAP_ID_DIST_POS tmpData(AdjacentIdDist.ID, AdjacentIdDist.Distance, AdjacentPos);

	ConnectedIds[CurrentID].push_back(tmpData);				// Add to current record

	tmpData.ID = CurrentID;
	ConnectedIds[AdjacentIdDist.ID].push_back(tmpData);		// Add to adjacent obstacle record
}

void ObstaclePathFinder::AddAdjacentEdge(const OBSTACLE_PATH_FINDER::MAP_ID_DIST &CurrentIdDist, const POS_2D &CurPos, const unsigned int &EdgeID, OBSTACLE_PATH_FINDER::CONNECTED_IDS &ConnectedIDs)
{
	ObstaclePathFinder::AddAdjacentBorder(EdgeID, CurrentIdDist, CurPos, ConnectedIDs);
}

void ObstaclePathFinder::FindNextPathStep(const ObstacleConnections &Connections, const bool CCW, const unsigned int &CurPathNum, std::vector<std::vector<OBSTACLE_PATH_FINDER::OBSTACLE_CONNECTION>> &CurPaths)
{
	const unsigned int startID = Connections.GetNumObstacles() - (MAP_CONST_EDGE_IDS::NumExtraIDs-1-MAP_CONST_EDGE_IDS::StartID_Add);
	const unsigned int destID = Connections.GetNumObstacles() - (MAP_CONST_EDGE_IDS::NumExtraIDs-1-MAP_CONST_EDGE_IDS::DestID_Add);

	// Two options from here on:
	//		1: Continue in same rotation direction on this obstacle
	//		2: Continue in opposite rotation direction on other connected obstacle
	std::vector<OBSTACLE_PATH_FINDER::OBSTACLE_CONNECTION> &curPath = CurPaths[CurPathNum];
	OBSTACLE_PATH_FINDER::OBSTACLE_CONNECTION &curConnection = curPath[curPath.size()-1];

	// Option 1:
	// Get next connected Obstacle in same direction on same obstacle
	unsigned int nextSameObstacleID = Connections.GetNextObstacleInRotDir(curConnection.ObstacleID, curConnection.ConnectedObstacleID, CCW);

	// Option 2:
	// Get next connected Obstacle in other direction on other obstacle
	unsigned int nextOtherObstacleID = Connections.GetNextObstacleInRotDir(curConnection.ConnectedObstacleID, curConnection.ObstacleID, !CCW);		// switch obstacle IDs around and switch Rotation Dir

	// Check whether these connections have already been tried out connection hasn't been visited before
	bool sameVisited = false;
	bool otherVisited = false;
	for(unsigned int i=0; i<curPath.size()-1; i++)
	{
		const OBSTACLE_PATH_FINDER::OBSTACLE_CONNECTION &tmpCon = curPath[i];

		// Check whether same was visted before
		if((nextSameObstacleID == tmpCon.ConnectedObstacleID && curConnection.ObstacleID == tmpCon.ObstacleID) ||
				(nextSameObstacleID == tmpCon.ObstacleID && curConnection.ObstacleID == tmpCon.ConnectedObstacleID))
		{
			sameVisited = true;

			// both have been visited, stop now
			if(otherVisited)
				break;
		}

		// Check whether other was visited before
		if((nextOtherObstacleID == tmpCon.ConnectedObstacleID && curConnection.ConnectedObstacleID == tmpCon.ObstacleID) ||
				(nextOtherObstacleID == tmpCon.ObstacleID && curConnection.ConnectedObstacleID == tmpCon.ConnectedObstacleID))
		{
			otherVisited = true;

			// both have been visited, stop now
			if(sameVisited)
				break;
		}
	}

	// If same wasn't visited yet, visit it now
	if(sameVisited)
	{
		OBSTACLE_PATH_FINDER::OBSTACLE_CONNECTION tmpCon;
		tmpCon.ObstacleID = curConnection.ObstacleID;
		tmpCon.ConnectedObstacleID = nextSameObstacleID;

		curPath.push_back(tmpCon);

		// Continue looking for complete path until destination is reached
		if(tmpCon.ConnectedObstacleID != destID && tmpCon.ObstacleID != destID)
			ObstaclePathFinder::FindNextPathStep(Connections, CCW, CurPathNum, CurPaths);

		if(otherVisited)
		{
			// Create new path for this case and append it to end of CurPaths
			CurPaths.push_back(curPath);

			tmpCon.ObstacleID = curConnection.ConnectedObstacleID;
			tmpCon.ConnectedObstacleID = nextOtherObstacleID;

			CurPaths[CurPaths.size()-1].push_back(tmpCon);

			// Continue looking for complete path until destination is reached
			if(tmpCon.ConnectedObstacleID != destID && tmpCon.ObstacleID != destID)
				ObstaclePathFinder::FindNextPathStep(Connections, !CCW, CurPaths.size()-1, CurPaths);
		}
	}
	else if (otherVisited)
	{
		// if only other path visited is valid, continue with this one
		OBSTACLE_PATH_FINDER::OBSTACLE_CONNECTION tmpCon;
		tmpCon.ObstacleID = curConnection.ConnectedObstacleID;
		tmpCon.ConnectedObstacleID = nextOtherObstacleID;

		curPath.push_back(tmpCon);

		// Continue looking for complete path until destination is reached
		if(tmpCon.ConnectedObstacleID != destID && tmpCon.ObstacleID != destID)
			ObstaclePathFinder::FindNextPathStep(Connections, !CCW, CurPathNum, CurPaths);
	}
	else
	{
		// if none of them are valid paths, erase the current attempt
		CurPaths.erase(CurPaths.begin()+CurPathNum);
	}
}
