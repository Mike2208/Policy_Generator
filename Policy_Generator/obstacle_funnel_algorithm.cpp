#include "obstacle_funnel_algorithm.h"

ObstacleFunnelAlgorithm::ObstacleFunnelAlgorithm()
{

}


int ObstacleFunnelAlgorithm::CalculateOptimalRoute(const ObstacleConnections &ObstacleData, const Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST> &IdDistMap, const VERTICE_PATH_DATA &PathData, const POS_2D &StartPos, const POS_2D &Destination)
{
	// Start by getting relevant vertice points ( Get points on obstacle that are nearest to the Connection point )
	std::vector<OBSTACLE_FUNNEL_ALGORITHM::SINGLE_VERTICE_POSITIONS> connectionPositions;

	for(unsigned int i=0; i<PathData.size(); i++)
	{
		// Get connection position
		const OBSTACLE_FUNNEL_ALGORITHM::SINGLE_VERTICE_PATH_DATA &curData = PathData[i];
		POS_2D connectionPos;

		if(ObstacleData.GetConnectionPos(curData.ObstacleID, curData.ConnectedObstacleID, connectionPos) < 0)
			return -1;

		// Find closest point on obstacles
		if(IdDistMap.GetPixel(connectionPos) == curData.ObstacleID)
		{
			// Start with obstacleID

		}
	}
}
