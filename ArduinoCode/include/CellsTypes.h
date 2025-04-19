#ifndef _CELLS_TYPES_H_
#define _CELLS_TYPES_H_

#include "Arduino.h"

/*  /N\
  <W + E>
    \S/
*/

struct Vec2
{
    Vec2(){};
    Vec2(uint8_t x, uint8_t y) : x(x), y(y){}
    uint8_t x, y;
};

template<uint8_t N>
struct Vec2Array{
public:
    Vec2Array(){
        for (uint8_t i = 0; i < N; i++) {
            _data[i] = {0, 0};
        }
    }

    Vec2Array(const Vec2 (&arr)[N]){
        for(uint8_t i = 0; i < N; i++){
            _data[i] = arr[i];
        }
    }
    
    bool operator==(const Vec2Array& other) const{
        if(other._size != this->_size) return false;

        for(uint8_t i = 0; i < _size; i++){
            if(this->_data[i].x != other._data[i].x) return false;
            if(this->_data[i].y != other._data[i].y) return false;
        }

        return true;
    }

    Vec2 operator[](uint8_t index){ return _data[index]; }
    const Vec2 operator[](uint8_t index) const{ return _data[index]; }

private:
    Vec2 _data[N];
    const uint8_t _size = N;
};

constexpr uint8_t DIRECTION_SIZE = 4;
enum class Direction : uint8_t{
    N = 0, E, S, W
};

enum class DirectionState : uint8_t{
    UNDEF = 0,
    DEF
};

enum class WallState : uint8_t{
    UNDEF = 0, LO, HI
};

struct RawCellStore{
    enum class PathDirStore : uint8_t{
        first = 0, second
    };

    WallState s_wall : 2;
    WallState e_wall : 2;
    Direction cell_dir : 2;
    DirectionState is_def_cell_dir : 1;
    PathDirStore path_dir : 1; 

    /** it may be both LO_PATH_DIR and HI_PATH_DIR. it depend by current element
      * so, take maze 2x2 
      * in string view (here it is cell_store): 0 1 2 3 4 5 6 7 8 
      * in my view:       0 1  
      *                 2 3 4
      *                 5 6 7,
      *     
      * but we "see":     3 4
      *                   6 7
      * 
      * in this way, 0th-element is HI_PATH_DIR, 1th-element is LO_PATH_DIR.
      * but, 0th and 1th elements together are the full direction of the path
      * for example, HI_PATH_DIR are: 0, 2, 4, 6;
      *              LO_PATH_DIR are: 1, 3, 5, 7;
      *       but full direction are: {0, 1}, {2, 3}, {4, 5}, {6, 7};
      * 
      * that are, (full struct) Direction dir = static_cast<Direction>((cell_store[0] << 1) | cell_store[1])
    */
};

struct DirectionStore
{
    DirectionState is_def_cell_dir : 1;
    Direction cell_dir : 2;
};

struct Cell{
    WallState north_wall;
    WallState east_wall;
    WallState south_wall;
    WallState west_wall;
};

#endif