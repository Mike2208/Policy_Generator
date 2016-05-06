/* MapGenerator_standards.h
 *  	contains standard values that are used throughout the program
 */
#ifndef MAP_STANDARDS_H
#define MAP_STANDARDS_H

#include <climits>

// Values for the Occupancy Grid map
// 	These values are for a deterministic map, a position is either empty (0) or full (1), nothing in between
#define OCCUPANCYGRID_STAT_TYPE		unsigned char
#define OCCUPANCYGRID_STAT_FULL		UCHAR_MAX
#define OCCUPANCYGRID_STAT_HALF		(OCCUPANCYGRID_STAT_FULL/2)
#define OCCUPANCYGRID_STAT_EMPTY	0

#define OCCUPANCYGRID_TYPE 		unsigned char
#define OCCUPANCYGRID_EMPTY 	0
#define OCCUPANCYGRID_FULL 		255

// Values for a discrete Occupancy Grid
#define OCCUPANCYGRID_DISCRETE_TYPE		bool
#define OCCUPANCYGRID_DISCRETE_EMPTY	0
#define OCCUPANCYGRID_DISCRETE_FULL		1

// Position definition
#define POS_2D_TYPE 		unsigned int
struct POS_2D
{
	POS_2D_TYPE 	X;
	POS_2D_TYPE 	Y;

	POS_2D() {}
	POS_2D(POS_2D_TYPE _X, POS_2D_TYPE _Y) : X(_X), Y(_Y) {}

	POS_2D operator+(const POS_2D &i) const { return POS_2D(this->X+i.X,this->Y+i.Y); }
};

// Global Pose of robot (includes global position and global rotation angle)
//		In the map, the robot is at position POSE::Position, and has rotated POSE::RotAngle counter-clockwise from facing right
#define ROT_ANGLE_TYPE		float
struct POSE
{
		POS_2D				Position;
		ROT_ANGLE_TYPE		RotationAngle;
};

// Path data that only stores movement, but doesn't include Start Position
//		The robot should follow this by first rotating by the amount in RotAngle, then driving forward for a length of PathLength
#define PATH_DATA_LENGTH_TYPE		float
struct PATH_DATA_LOCAL
{
		ROT_ANGLE_TYPE				RotAngle;			// Angle the robot should turn to face next movement direction
		PATH_DATA_LENGTH_TYPE		PathLength;			// How far the robot should move
};

// Values for MapComparator
#define MAPCOMPARATOR_MINDIFF		1		// Minimum difference between two maps for it to be registered


// Values for Algorithms
#define ALG_PROBABILITY_TYPE		unsigned float
#define ALG_PROBABILITY_HALF		((ALG_PROBABILITY_TYPE) 0.5)

// Values for height map
#define HEIGHT_TYPE					unsigned int


// Obstacle data
typedef unsigned int				OBSTACLE_ID;

#endif
