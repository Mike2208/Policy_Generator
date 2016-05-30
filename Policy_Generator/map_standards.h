/* MapGenerator_standards.h
 *  	contains standard values that are used throughout the program
 */
#ifndef MAP_STANDARDS_H
#define MAP_STANDARDS_H

#include <climits>
#include <limits>

template<class T>
T GetInfinteVal()
{
	if(std::numeric_limits<T>::has_infinity)
		return std::numeric_limits<T>::infinity();
	else
		return std::numeric_limits<T>::max();
}

// Values for the Occupancy Grid map
// 	These values are for a deterministic map, a position is either empty (0) or full (1), nothing in between
typedef unsigned char				OCCUPANCYGRID_STAT_TYPE;
const	OCCUPANCYGRID_STAT_TYPE		OCCUPANCYGRID_STAT_FULL  = UCHAR_MAX;
const	OCCUPANCYGRID_STAT_TYPE		OCCUPANCYGRID_STAT_HALF	 = (OCCUPANCYGRID_STAT_FULL/2);
const	OCCUPANCYGRID_STAT_TYPE		OCCUPANCYGRID_STAT_EMPTY = 0;

typedef	unsigned char		OCCUPANCYGRID_TYPE;
const OCCUPANCYGRID_TYPE	OCCUPANCYGRID_EMPTY = 0;
const OCCUPANCYGRID_TYPE	OCCUPANCYGRID_FULL  = 255;

// Values for a discrete Occupancy Grid
typedef bool		OCCUPANCYGRID_DISCRETE_TYPE;
const OCCUPANCYGRID_DISCRETE_TYPE OCCUPANCYGRID_DISCRETE_EMPTY = 0;
const OCCUPANCYGRID_DISCRETE_TYPE OCCUPANCYGRID_DISCRETE_FULL  = 1;

// Position definition
typedef unsigned int POS_2D_TYPE;
struct POS_2D
{
	POS_2D_TYPE 	X;
	POS_2D_TYPE 	Y;

	POS_2D() {}
	POS_2D(const POS_2D &S) : X(S.X), Y(S.Y) {}
	POS_2D(POS_2D_TYPE _X, POS_2D_TYPE _Y) : X(_X), Y(_Y) {}

	POS_2D operator+(const POS_2D &i) const { return POS_2D(this->X+i.X,this->Y+i.Y); }
	POS_2D operator-(const POS_2D &i) const { return POS_2D(this->X-i.X,this->Y-i.Y); }
	bool operator==(const POS_2D &i) const { return (this->X==i.X && this->Y == i.Y); }
	bool operator!=(const POS_2D &i) const { return !(i==*this); }
};

// Global Pose of robot (includes global position and global rotation angle)
//		In the map, the robot is at position POSE::Position, and has rotated POSE::RotAngle counter-clockwise from facing right
typedef float ROT_ANGLE_TYPE;
struct POSE
{
		POS_2D				Position;
		ROT_ANGLE_TYPE		RotationAngle;
};

// Path data that only stores movement, but doesn't include Start Position
//		The robot should follow this by first rotating by the amount in RotAngle, then driving forward for a length of PathLength
typedef float PATH_DATA_LENGTH_TYPE;
struct PATH_DATA_LOCAL
{
		ROT_ANGLE_TYPE				RotAngle;			// Angle the robot should turn to face next movement direction
		PATH_DATA_LENGTH_TYPE		PathLength;			// How far the robot should move
};

// Values for MapComparator
const unsigned int MAPCOMPARATOR_MINDIFF = 1;		// Minimum difference between two maps for it to be registered


// Values for Algorithms
typedef float ALG_PROBABILITY_TYPE;
const ALG_PROBABILITY_TYPE ALG_PROBABILITY_HALF	= ((ALG_PROBABILITY_TYPE) 0.5);

// Values for height map
typedef unsigned int HEIGHT_TYPE;


// Obstacle data
typedef unsigned int				OBSTACLE_ID;
const OBSTACLE_ID	OBSTACLE_ID_EMPTY	= UINT_MAX;
const OBSTACLE_ID	OBSTACLE_ID_INVALID	= UINT_MAX-1;
const OBSTACLE_ID	OBSTACLE_ID_RESERVED= UINT_MAX-2;

#endif
