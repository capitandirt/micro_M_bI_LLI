#include "Solver.h"

void Solver::calc_path(const uint8_t ind_s, const uint8_t ind_f){
    uint8_t cur_cell_ind = ind_s;
    uint8_t cur_x, cur_y;

    while(cur_cell_ind != ind_f){
        cur_x = cur_cell_ind % MAZE_SIDE_LENGTH;
        cur_y = cur_cell_ind / MAZE_SIDE_LENGTH;

        _Maze->GetCellDir(_buf_cell_dir, cur_x, cur_y);  
        _Maze->PushBackPathDir(_buf_cell_dir.cell_dir);

        switch (_buf_cell_dir.cell_dir)
        {
        case Direction::N:
            cur_cell_ind -= MAZE_SIDE_LENGTH;
            break;

        case Direction::E:
            cur_cell_ind++;
            break;

        case Direction::S:
            cur_cell_ind += MAZE_SIDE_LENGTH;
            break;

        case Direction::W:
            cur_cell_ind--;
            break;
        }
    }
}

void Solver::MazeTestConfig(){
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::HI}, 0, 0);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::HI, WallState::LO}, 2, 0);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 4, 0);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::LO, WallState::HI}, 6, 0);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 8, 0);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::HI, WallState::LO}, 1, 1);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::HI, WallState::HI}, 3, 1);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::HI}, 5, 1);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::HI}, 7, 1);
    _Maze->SetCell({WallState::HI, WallState::HI, WallState::LO, WallState::HI}, 9, 1);
    _Maze->SetCell({WallState::HI, WallState::HI, WallState::LO, WallState::HI}, 0, 2);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::LO}, 2, 2);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 4, 2);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::LO, WallState::HI}, 6, 2);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::HI, WallState::LO}, 8, 2);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 1, 3);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::HI, WallState::LO}, 3, 3);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 5, 3);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 7, 3);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::LO}, 9, 3);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::LO, WallState::HI}, 0, 4);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::HI, WallState::LO}, 2, 4);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::LO, WallState::HI}, 4, 4);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 6, 4);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::HI, WallState::LO}, 8, 4);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 1, 5);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::HI}, 3, 5);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::HI}, 5, 5);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::HI, WallState::LO}, 7, 5);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::HI}, 9, 5);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::HI}, 0, 6);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::HI, WallState::HI}, 2, 6);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::LO}, 4, 6);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 6, 6);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::LO, WallState::HI}, 8, 6);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::HI}, 1, 7);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 3, 7);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 5, 7);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::LO}, 7, 7);
    _Maze->SetCell({WallState::HI, WallState::HI, WallState::HI, WallState::LO}, 9, 7);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::HI}, 0, 8);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::LO, WallState::HI}, 2, 8);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::LO, WallState::LO}, 4, 8);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 6, 8);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::LO, WallState::HI}, 8, 8);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::LO, WallState::LO}, 1, 9);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::HI}, 3, 9);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::LO, WallState::LO}, 5, 9);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::LO, WallState::LO}, 7, 9);
    _Maze->SetCell({WallState::HI, WallState::HI, WallState::HI, WallState::LO}, 9, 9);
    _Maze->SetCell({WallState::HI, WallState::HI, WallState::LO, WallState::HI}, 0, 0);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState ::HI, WallState::LO}, 5, 10);
    _Maze->SetCell({WallState::HI, WallState::HI, WallState::LO, WallState::LO}, 5, 9);
    _Maze->SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, 4, 9);
    _Maze->SetCell({WallState::LO, WallState::HI, WallState::HI, WallState::LO}, 1, 9);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::HI, WallState::LO}, 8, 9);
    _Maze->SetCell({WallState::LO, WallState::LO, WallState::HI, WallState::LO}, 7, 9);

    _Maze->PrimaryFill();
}

void Solver::SolveBfsMaze(const uint8_t x_s, const uint8_t y_s, const uint8_t x_f, const uint8_t y_f){
    uint8_t ind_s = x_s + y_s * MAZE_SIDE_LENGTH;   
    uint8_t ind_f = x_f + y_f * MAZE_SIDE_LENGTH;   
    
    _queue.pushBack(ind_f);

    while(!_queue.isEmpty()){
        uint8_t cur_cell_ind = _queue.popFront();
        
        if(cur_cell_ind == ind_s){
            break;
        }

        uint8_t cur_x = cur_cell_ind % MAZE_SIDE_LENGTH;
        uint8_t cur_y = cur_cell_ind / MAZE_SIDE_LENGTH;

        Cell cur_cell;
        _Maze->GetCell(cur_cell, cur_x, cur_y);

        if(cur_cell.north_wall != WallState::HI){
            _Maze->GetCellDir(_buf_cell_dir, cur_x, cur_y - 1);
            
            if(_buf_cell_dir.is_def_cell_dir != DirectionState::DEF){
                _Maze->SetCellDir(Direction::S, cur_x, cur_y - 1);
                _queue.pushBack(cur_cell_ind - MAZE_SIDE_LENGTH); // cur_y--
            }
        }

        if(cur_cell.east_wall != WallState::HI){
            _Maze->GetCellDir(_buf_cell_dir, cur_x + 1, cur_y);

            if(_buf_cell_dir.is_def_cell_dir != DirectionState::DEF){
                _Maze->SetCellDir(Direction::W, cur_x + 1, cur_y);
                _queue.pushBack(cur_cell_ind + 1); // cur_x++
            }
        }

        if(cur_cell.south_wall != WallState::HI){
            _Maze->GetCellDir(_buf_cell_dir, cur_x, cur_y + 1);

            if(_buf_cell_dir.is_def_cell_dir != DirectionState::DEF){
                _Maze->SetCellDir(Direction::N, cur_x, cur_y + 1);
                _queue.pushBack(cur_cell_ind + MAZE_SIDE_LENGTH); // cur_y++
            }
        }

        if(cur_cell.west_wall != WallState::HI){
            _Maze->GetCellDir(_buf_cell_dir, cur_x - 1, cur_y);

            if(_buf_cell_dir.is_def_cell_dir != DirectionState::DEF){
                _Maze->SetCellDir(Direction::E, cur_x - 1, cur_y);
                _queue.pushBack(cur_cell_ind - 1); // cur_x--
            }
        }
    }

    _Maze->UndefCell(x_f, y_f);

    calc_path(ind_s, ind_f);
}