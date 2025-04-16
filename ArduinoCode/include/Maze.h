#ifndef _MAZE_H_
#define _MAZE_H_

#include "Arduino.h"
#include "Config.h"

#define HORIZ_WALL "==="
#define VERTIC_WALL " | "

#define BLANK_HORIZ_WALL "   "
#define BLANK_VERTIC_WALL "   "

#define ANGLE " + "
#define LEFT_ANGLE "* "
#define RIGHT_ANGLE " *"

#define VOID_CELL " X "

#define UNDEF_DIR "   "
#define N_DIR " N "
#define E_DIR " E "
#define S_DIR " S "
#define W_DIR " W "

/*  /N\
  <W + E>
    \S/
*/

enum class WallState : uint8_t{
    UND = 0, LO, HI
};

constexpr uint8_t DIRECTION_SIZE = 4;
enum class Direction : uint8_t{
    N = 0, E, S, W
};

enum class DirectionState : uint8_t{
    UNDEF = 0,
    DEF
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

class Maze{
public:
    void PrimaryFill() noexcept;
    
    void SetCell(const Cell set_cell, const uint8_t x, const uint8_t y) noexcept;
    void UndefCell(const uint8_t x, const uint8_t y) noexcept;

    void GetCell(Cell& get_cell, const uint8_t x, const uint8_t y) const noexcept;
    
    void SetCellDir(const Direction direction, const uint8_t x, const uint8_t y) noexcept;
    void GetCellDir(DirectionStore& direction_store, const uint8_t x, const uint8_t y) const noexcept;

    void PushBackPathDir(const Direction dir) noexcept;
    
    void SetPathDir(const Direction dir, uint8_t ind) noexcept;
    void GetPathDir(Direction& dir, uint8_t ind) const noexcept;
    Direction GetPathDir(uint8_t ind) const noexcept;

    void ClearPath() noexcept;
    uint8_t GetPathSize() const noexcept;

    void PrintDirPath() const noexcept;

    void PrintCell(const uint8_t x, const uint8_t y) const noexcept;
    void Print() const noexcept;

private:

    bool cell_request_is_out_of_range_cell_blocks(const uint8_t& x, const uint8_t& y) const noexcept;

    void print_cell_north_wall(   const uint8_t x, const uint8_t y ) const noexcept;
    void print_cell_middle_walls( const uint8_t x, const uint8_t y ) const noexcept;
    void print_cell_south_wall(   const uint8_t x, const uint8_t y ) const noexcept;
    void print_cell_path(         const uint8_t x, const uint8_t y ) const noexcept;

private:
    RawCellStore            _cell_blocks[MAZE_MEM_SIZE];      
    
    mutable RawCellStore*   _buf_cell_ptr = nullptr;
    mutable DirectionStore  _buf_direction_store;
    mutable Direction       _buf_path_direction_store;

    uint8_t                 _path_ind = 0;
};

#endif // !_MAZE_H_