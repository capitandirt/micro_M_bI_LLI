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

inline Direction toOpposite(Direction dir){
    return static_cast<Direction>((static_cast<uint8_t>(dir) + DIRECTION_SIZE / 2) % DIRECTION_SIZE);
}

struct Vec2
{
    Vec2(){};
    Vec2(uint8_t x, uint8_t y) : x(x), y(y){}

    Vec2 plusOrtVector(Direction dir){
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
    LO = 0, HI = 1, UNDEF = 2
};

inline uint8_t toInt(WallState ws){
    return static_cast<uint8_t>(ws);
}

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

    WallState s_wall : 2;
    WallState e_wall : 2;
    Direction cell_dir : 2;
    DirectionState is_def_cell_dir : 1;
    PathDirStore path_dir : 1;

    /** it may be both LO_PATH_DIR and HI_PATH_DIR. it depend by current element
      * so, take maze 2x2 
      * in string view (here it is cell_store): 0 1 2 3 4 5 6 7
      * in my view:       0 1  
      *                 2 3 4
      *                 5 6 7,
      *     
      * but we "see":     3 4
      *                   6 7
      * 
      * in this way, 0th-element is HI_PATH_DIR, 1th-element is LO_PATH_DIR.
      * but, 0th and 1th elements together are the full direction of the path
      * for example, HI_PATH_DIR is: 0, 2, 4, 6;
      *              LO_PATH_DIR is: 1, 3, 5, 7;
      *       but full direction is: {0, 1}, {2, 3}, {4, 5}, {6, 7};
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

union Cell_u{
    Cell c;
    WallState walls[DIRECTION_SIZE];
};

inline Cell inDir(const Cell c, const Direction d){
    Cell out_cell;

    switch (d)
    {
    case Direction::N:
        out_cell.north_wall = c.north_wall;
        out_cell.east_wall  = c.east_wall;
        out_cell.south_wall = c.south_wall;
        out_cell.west_wall  = c.west_wall;
        break;
    case Direction::E:
        out_cell.north_wall = c.west_wall;
        out_cell.east_wall  = c.north_wall;
        out_cell.south_wall = c.east_wall;
        out_cell.west_wall  = c.south_wall;
        break;
    case Direction::S:
        out_cell.north_wall = c.south_wall;
        out_cell.east_wall  = c.west_wall;
        out_cell.south_wall = c.north_wall;
        out_cell.west_wall  = c.east_wall;
        break;
    case Direction::W:
        out_cell.north_wall = c.east_wall;
        out_cell.east_wall  = c.south_wall;
        out_cell.south_wall = c.west_wall;
        out_cell.west_wall  = c.north_wall;
        break;
    }

    return out_cell;
}


// N | E | S | W
inline uint8_t toInt(Cell c){
    return toInt(c.north_wall) << 6 | toInt(c.east_wall) << 4 | 
           toInt(c.south_wall) << 2 | toInt(c.west_wall);
}

#endif