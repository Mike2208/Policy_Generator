#include "obstacle_connection.h"
#include "obstacle_path_finder.h"
#include "map_height_map.h"
#include "robot_navigation.h"

#include <cmath>

ObstacleConnection::ObstacleConnection()
{
}

ObstacleConnection::~ObstacleConnection()
{

}

void ObstacleConnection::Reset()
{
	this->_ID = OBSTACLE_ID_EMPTY;
	this->_Connections.clear();
}

int ObstacleConnection::CalculateAllConnections(const ObstacleMap &Obstacles)
{
	// Reset everyhting
	this->Reset();

	//
}
