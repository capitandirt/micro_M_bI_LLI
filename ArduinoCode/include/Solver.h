#ifndef _SOLVER_H_
#define _SOLVER_H_

#include "CycloWorker.h"
#include "Maze.h"
#include "MyQueue.h"

class Solver{
public:
    Solver(Maze* maze) : _maze(maze){}
    
    void MazeTestConfig() noexcept;
    void ExplorerSolveBfsMaze(const Vec2 start, const Vec2 finish);
    void FastSolveBfsMaze(const Vec2 start, const Vec2 finish);
    
    Vec2 firstUndefCellCoords();
    
private:
    void calc_path(const uint8_t ind_s, const uint8_t ind_f);

private:
    Maze* _maze;
    Queue<uint8_t> _queue;   

    uint16_t _path_len;
    Vec2 _first_undef_cell_coords;
    bool WAS_UNDEF_CELL = 0;

    DirectionStore _buf_cell_dir;   
};

#endif // !_SOLVER_H_