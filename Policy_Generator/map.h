/* Map.h
 * 	includes class Map.h
 */
#ifndef MAP_H
#define MAP_H

/* class Map
 * 	contains one map of a certain width and height
 * 	the type of the pixels of this map are determined by T
 */

#include "map_standards.h"

#include <vector>
#include <fstream>

template<class T>
class Map
{
	public:

		Map();
		Map(const unsigned int &Height, const unsigned int &Width, const T &PixelInitValue );
		~Map();
		
		int ResetMap(const T &ResetValue);		// Resets entire Map to the given value
		int ResetMap(const POS_2D_TYPE &NewHeight, const POS_2D_TYPE &NewWidth, const T &ResetValue);	// Clears Map and resizes it to new Height and Width, also sets all entries to given value

		int SetMapToValue(T Value); 			// Sets entire map to given value

		int SetPixel(POS_2D Position, T Value);						// Sets one pixel to value
		int SetPixel(POS_2D_TYPE PosX, POS_2D_TYPE PosY, T Value);		// Sets one pixel to value

		int SetArea(POS_2D TopLeft, POS_2D BottomRight, T Value);		// Sets area to given value
		int SetArea(POS_2D_TYPE TopLeftX, POS_2D_TYPE TopLeftY, POS_2D_TYPE BottomRightX, POS_2D_TYPE BottomRightY, T Value);

		int SetPath(const POS_2D &StartPos, const POS_2D &EndPos, const T &Value);		// Set a path to given Value

		int GetPixel(POS_2D Position, T &Value) const; 	// Gets pixel from position and places value in Value, returns error if out of bounds
		int GetPixel(POS_2D_TYPE PosX, POS_2D_TYPE PosY, T &Value) const;		// Gets pixel from position and places value in Value, returns error if out of bounds

		T GetPixel(POS_2D Position) const;				// Like other GetPixel, except no error checking
		T GetPixel(POS_2D_TYPE PosX, POS_2D_TYPE PosY) const;				// Like other GetPixel, except no error checking

		unsigned int GetHeight() const;
		unsigned int GetWidth() const;

		int PrintMapToFile(const char *FileName, const unsigned int maxMapValue);

	private:

		unsigned int _Height, _Width; 	// Height and Width of map
		std::vector<T> _OccupancyGrid;
};

typedef Map<bool> Map_BoolType;
typedef Map<unsigned char> Map_CharType;
typedef Map<unsigned int> Map_IntType;
typedef Map<OCCUPANCYGRID_STAT_TYPE> Map_Statistical;
typedef Map<OBSTACLE_ID>	Map_ID;

// Include map.cpp because of template
#include "map.cpp"


#endif
