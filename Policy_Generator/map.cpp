#ifndef MAP_CPP
#define MAP_CPP

#include "map.h"

template<class T>
Map<T>::Map() : _Height(0), _Width(0), _OccupancyGrid(0)
{
}

template<class T>
Map<T>::Map(const unsigned int &Height, const unsigned int &Width, const T &PixelInitValue ) : _Height(Height), _Width(Width)
{
	this->_OccupancyGrid.resize(Height*Width);

	this->SetMapToValue(PixelInitValue);
}

template<class T>
Map<T>::~Map()
{
}

template<class T>
int Map<T>::ResetMap(const T &ResetValue)
{
	return this->SetMapToValue(ResetValue);
}

template<class T>
int Map<T>::ResetMap(const POS_2D_TYPE &NewHeight, const POS_2D_TYPE &NewWidth, const T &ResetValue)
{
	this->_Height = NewHeight;
	this->_Width = NewWidth;
	this->_OccupancyGrid.resize(NewHeight*NewWidth);

	return this->SetMapToValue(ResetValue);
}

template<class T>
int Map<T>::SetMapToValue(T Value)
{
	for(unsigned int i=0; i<this->_Height*this->_Width; i++)
	{
		this->_OccupancyGrid[i] = Value;
	}

	return 1;
}

template<class T>
int Map<T>::SetPixel(POS_2D Position, T Value)
{
	return this->SetPixel(Position.X, Position.Y, Value);
}

template<class T>
int Map<T>::SetPixel(POS_2D_TYPE PosX, POS_2D_TYPE PosY, T Value)
{
	this->_OccupancyGrid[PosX+this->_Width*PosY] = Value;

	return 1;
}

template<class T>
int Map<T>::SetArea(POS_2D TopLeft, POS_2D BottomRight, T Value)
{
	return this->SetArea(TopLeft.X, TopLeft.Y, BottomRight.X, BottomRight.Y, Value);
}

template<class T>
int Map<T>::SetArea(POS_2D_TYPE TopLeftX, POS_2D_TYPE TopLeftY, POS_2D_TYPE BottomRightX, POS_2D_TYPE BottomRightY, T Value)
{
	for(unsigned int i=TopLeftX; i<=BottomRightX; i++)
	{
		for(unsigned int j=BottomRightY; j<=TopLeftY; j++)
		{
			this->SetPixel(i,j, Value);
		}
	}

	return 1;
}

template<class T>
int Map<T>::GetPixel(POS_2D Position, T &Value) const
{
	return this->GetPixel(Position.X, Position.Y, Value);
}

template<class T>
int Map<T>::GetPixel(POS_2D_TYPE PosX, POS_2D_TYPE PosY, T &Value) const
{
	if(PosX < 0 || PosX >= this->_Width
			|| PosY < 0 || PosY >= this->_Height)
		return -1;

	Value = this->_OccupancyGrid[PosX + PosY * this->_Width];

	return 1;
}

template<class T>
T Map<T>::GetPixel(POS_2D Position) const
{
	T curVal;
	this->GetPixel(Position.X, Position.Y, curVal);

	return curVal;
}

template<class T>
T Map<T>::GetPixel(POS_2D_TYPE PosX, POS_2D_TYPE PosY) const
{
	T curVal;
	this->GetPixel(PosX, PosY, curVal);

	return curVal;
}

template<class T>
unsigned int Map<T>::GetHeight() const
{
	return this->_Height;
}

template<class T>
unsigned int Map<T>::GetWidth() const
{
	return this->_Width;
}

template<class T>
int Map<T>::PrintMapToFile(const char *FileName, const unsigned int maxMapValue)
{
	std::fstream file;
	file.open(FileName, std::fstream::out | std::fstream::trunc );

	if(!file.is_open())
		return -1;

	// PBM for maps with only two values
	if(std::is_same<T, OCCUPANCYGRID_DISCRETE_TYPE>())
	{
		file << "P1\n" << this->_Height << " " << this->_Width << "\n";

		for(int i=this->_Height-1; i>=0; i--)
		{
			for(unsigned int j=0; j<this->_Width; j++)
			{
				if(this->GetPixel(j,i) == OCCUPANCYGRID_DISCRETE_EMPTY)
				{
					file << "0 ";
				}
				else
				{
					file << "1 ";
				}
			}
			file << "\n";
		}
	}
	else	// more than two values
	{
		file << "P2\n" << this->_Height << " " << this->_Width << "\n";
		file << "255\n";

		for(int i=this->_Height-1; i>=0; i--)
		{
			for(unsigned int j=0; j<this->_Width; j++)
			{
				unsigned int tmpDist = this->GetPixel(j,i);
				if(tmpDist > maxMapValue)
					file << "255 ";
				else
				{
					unsigned char tmp = (((double)tmpDist)/maxMapValue)*255;
					file << std::to_string(tmp) << " ";
				}

			}
			file << "\n";
		}
	}

	file.close();

	return 1;
}


#endif
