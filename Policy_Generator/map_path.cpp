#include "map_path.h"
#include <cmath>

using std::vector;

MapPath::MapPath()
{

}

MapPath::~MapPath()
{

}

void MapPath::Reset()
{
	// Remove data
	this->_MovementOrders.resize(0);
	this->_MovementOrders.clear();

	// Reset positions
	this->_StartPosition.Position.X = 0;
	this->_StartPosition.Position.Y = 0;
	this->_StartPosition.RotationAngle = 0;

	this->_EndPosition = this->_StartPosition;
}

void MapPath::AddMoveOrder(const PATH_DATA_LOCAL &NewOrder)
{
	// Add movement to path
	this->_MovementOrders.push_back(NewOrder);

	// Calculate new end position
	this->UpdateEndPose(NewOrder);
}

void MapPath::SetPath(const POSE &StartPose, const std::vector<PATH_DATA_LOCAL> &Movements)
{
	// Reset data
	this->Reset();

	// Set Start position
	this->_StartPosition = StartPose;

	// Copy movement data
	this->_MovementOrders = Movements;

	// Calculate end position
	this->RecalculateEndPose();
}

void MapPath::SetDirectPath(const POSE &StartPose, const POSE &EndPose)
{
	// Reset data
	this->Reset();

	// Set poses
	this->_StartPosition = StartPose;
	this->_EndPosition = EndPose;

	// Calculate move order to get from point A to B
	PATH_DATA_LOCAL moveOrder;
	const ROT_ANGLE_TYPE  angleBetweenPoses = atan2(this->_EndPosition.Position.Y-this->_StartPosition.Position.Y, this->_EndPosition.Position.X-this->_StartPosition.Position.X);

	moveOrder.RotAngle = angleBetweenPoses - StartPose.RotationAngle;		// Rotate to face end pose
	moveOrder.PathLength = sqrt((this->_EndPosition.Position.Y-this->_StartPosition.Position.Y)*(this->_EndPosition.Position.Y-this->_StartPosition.Position.Y)+(this->_EndPosition.Position.X-this->_StartPosition.Position.X)*(this->_EndPosition.Position.X-this->_StartPosition.Position.X));

	// Add move order
	this->_MovementOrders.push_back(moveOrder);

	// Rotate to correct end pose
	moveOrder.PathLength = 0;
	moveOrder.RotAngle = this->_EndPosition.RotationAngle - angleBetweenPoses;

	// Add rotation
	this->_MovementOrders.push_back(moveOrder);
}

unsigned int MapPath::GetNumMovementOrders() const
{
	return this->_MovementOrders.size();
}

int	MapPath::GetMovementOrder(const unsigned int &OrderNum, PATH_DATA_LOCAL &Order) const
{
	// Check order number
	if(OrderNum >= this->_MovementOrders.size())
		return -1;

	Order = this->_MovementOrders[OrderNum];

	return 1;
}

POSE MapPath::GetStartPose() const
{
	return this->_StartPosition;
}

POSE MapPath::GetEndPose()	const
{
	return this->_EndPosition;
}

PATH_DATA_LOCAL MapPath::GetMovementOrder(const unsigned int &OrderNum) const
{
	PATH_DATA_LOCAL tmpOrder;
	tmpOrder.PathLength = 0;
	tmpOrder.RotAngle = 0;

	this->GetMovementOrder(OrderNum, tmpOrder);

	return tmpOrder;
}

void MapPath::RecalculateEndPose()
{
	// Reset end position
	this->_EndPosition = this->_StartPosition;

	// Go through move orders and calculate new position
	const unsigned int numMoveOrders = this->_MovementOrders.size();
	for(unsigned int i=0; i<numMoveOrders; i++)
	{
		this->UpdateEndPose(this->_MovementOrders[i]);
	}
}

void MapPath::UpdateEndPose(const PATH_DATA_LOCAL &NextMoveOrder)
{
	// Calculate new end position
	this->_EndPosition.RotationAngle += NextMoveOrder.RotAngle;		// New end rotation
	this->_EndPosition.Position.X += cos(this->_EndPosition.RotationAngle)*NextMoveOrder.PathLength;
	this->_EndPosition.Position.Y += sin(this->_EndPosition.RotationAngle)*NextMoveOrder.PathLength;
}
