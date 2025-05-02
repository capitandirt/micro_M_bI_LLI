#ifndef _CELLS_TYPES_H_
#define _CELLS_TYPES_H_

#include "Arduino.h"

/*  /N\
  <W + E>
    \S/
*/

constexpr uint8_t DIRECTION_SIZE = 4;
enum class Direction : uint8_t{
    N = 0, E, S, W
};

struct Vec2
{
    Vec2(){};
    Vec2(uint8_t x, uint8_t y) : x(x), y(y){}

    Vec2 getOrtVector(Direction dir){
             if(dir == Direction::N) y--;
        else if(dir == Direction::S) y++;
        else if(dir == Direction::W) x--;
        else if(dir == Direction::E) x++;
        
        return *this;
    }

    uint8_t x, y;
};

inline uint8_t toInt(Direction dir){
    return static_cast<uint8_t>(dir);
}

inline Direction incDir(Direction dir){
    return static_cast<Direction>((toInt(dir) + 1) % DIRECTION_SIZE);
}

inline Direction decDir(Direction dir){
    return static_cast<Direction>((toInt(dir) - 1 + DIRECTION_SIZE) % DIRECTION_SIZE);
}

enum class DirectionState : uint8_t{
    UNDEF = 0,
    DEF
};

enum class WallState : uint8_t{
    LO = 0, HI
};

inline WallState toWallState(bool exp){
    return exp? WallState::HI : WallState::LO;
}

inline bool toBool(WallState exp){
    if(exp == WallState::HI) return true;
    return false;
}

struct RawCellStore{
    enum class PathDirStore : uint8_t{
        first = 0, second
    };

    WallState s_wall : 1;
    WallState e_wall : 1;
    Direction cell_dir : 2;
    DirectionState is_def_cell_dir : 1;
    PathDirStore path_dir : 1;
    bool is_cell_passed : 1; 

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