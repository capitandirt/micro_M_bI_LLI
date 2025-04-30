#ifndef _MAZE_H_
#define _MAZE_H_

#include "Arduino.h"
#include "Config.h"
#include "CellsTypes.h"

#define HORIZ_WALL "==="
#define VERTIC_WALL " | "

#define BLANK_HORIZ_WALL "   "
#define BLANK_VERTIC_WALL "   "

#define VOID "   "
#define ANGLE " + "
#define LEFT_ANGLE "* "
#define RIGHT_ANGLE " *"

#define VOID_CELL " X "

#define UNDEF_DIR "   "
#define N_DIR " N "
#define E_DIR " E "
#define S_DIR " S "
#define W_DIR " W "

class Maze{
public:
    void PrimaryFill() noexcept;
    void Clear() noexcept;

    void SetCell(const Cell set_cell, const Vec2 v) noexcept;
    void UndefCell(const Vec2 vec2) noexcept;
    
    void GetCell(Cell& get_cell, const Vec2 v) const noexcept;
    
    void SetCellDir(const Direction direction, const Vec2 v) noexcept;
    void GetCellDir(DirectionStore& direction_store, const Vec2 v) const noexcept;    

    void PushBackPathDir(const Direction dir) noexcept;
    
    void SetPathDir(const Direction dir, uint8_t ind) noexcept;
    void GetPathDir(Direction& dir, uint8_t ind) const noexcept;
    Direction GetPathDir(uint8_t ind) const noexcept;

    bool CellIsPassed(const Vec2 v) const noexcept;
    void PassCell(const Vec2 v) noexcept;

    void ClearPath() noexcept;
    uint8_t GetPathSize() const noexcept;

    void PrintDirPath() const noexcept;

    void PrintCell(const Vec2 v) const noexcept;
    void Print() const noexcept;

    uint8_t static Vec2ToInd(Vec2 v);
    Vec2 static IndToVec2(uint8_t ind);
private:
    bool cell_request_is_out_of_range_cell_blocks(const Vec2& v) const noexcept;

    void print_cell_north_wall(   const Vec2 v ) const noexcept;
    void print_cell_middle_walls( const Vec2 v ) const noexcept;
    void print_cell_south_wall(   const Vec2 v ) const noexcept;
    void print_cell_path(         const Vec2 v ) const noexcept;

private:
    RawCellStore            _cell_blocks[MAZE_MEM_SIZE];      
    
    mutable RawCellStore*   _buf_cell_ptr = nullptr;
    mutable DirectionStore  _buf_direction_store;
    mutable Direction       _buf_path_direction_store;

    uint8_t                 _path_ind = 0;
};

#endif // !_MAZE_H_