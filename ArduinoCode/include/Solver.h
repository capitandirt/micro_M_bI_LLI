#ifndef _SOLVER_H_
#define _SOLVER_H_

#include "CycloWorker.h"
#include "Maze.h"
#include "MyQueue.h"

class Solver{
public:
    Solver(Maze* maze) : _Maze(maze){}
    
    void MazeTestConfig() noexcept;
    void SolveBfsMaze(const Vec2 start, const Vec2 finish);
private:
    void calc_path(const uint8_t ind_s, const uint8_t ind_f);

private:
    Maze* _Maze;
    Queue<uint8_t> _queue;   

    DirectionStore _buf_cell_dir;   
};

#endif // !_SOLVER_H_