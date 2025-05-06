#include "Maze.h"

void PRINT_N_SPACES(uint8_t n){
    for (size_t i = 0; i < n; i++)
    {
        Serial.print(" ");
    }
}

bool Maze::cell_request_is_out_of_range_cell_blocks(const Vec2& v) const{
    return v.x >= MAZE_SIDE_LENGTH || v.y >= MAZE_SIDE_LENGTH;
}

void Maze::print_cell_north_wall(const Vec2 v) const{
    Serial.print(ANGLE);
    if(_cell_blocks[v.x + v.y * (MAZE_SIDE_LENGTH_ADD_ONE)].s_wall == WallState::HI){
        Serial.print(HORIZ_WALL);
    }
    else{
        Serial.print(BLANK_HORIZ_WALL);
    }
}

void Maze::print_cell_middle_walls(const Vec2 v) const{
    if(_cell_blocks[MAZE_SIDE_LENGTH + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].e_wall == WallState::HI){
        Serial.print(VERTIC_WALL);
    }
    else{
        Serial.print(BLANK_VERTIC_WALL);
    }   
}

void Maze::print_cell_south_wall(const Vec2 v) const{
    Serial.print(ANGLE);
    if(_cell_blocks[MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].s_wall == WallState::HI){
        Serial.print(HORIZ_WALL);
    }
    else{
        Serial.print(BLANK_HORIZ_WALL);
    }
}

void Maze::print_cell_path(const Vec2 v) const{
    if(_buf_direction_store.is_def_cell_dir == DirectionState::DEF){
        switch (_buf_direction_store.cell_dir)
        {
        case Direction::N:
            Serial.print(N_DIR);
            break;
        
        case Direction::E:
            Serial.print(E_DIR);
            break;
        
        case Direction::S:
            Serial.print(S_DIR);
            break;
            
        case Direction::W:
            Serial.print(W_DIR);
            break;
        }
    }
    else{
        Serial.print(UNDEF_DIR);
    }
}

void Maze::SetCell(const Cell set_cell, const Vec2 v){
    if(cell_request_is_out_of_range_cell_blocks(v)) return;

    if(set_cell.north_wall == WallState::HI){
        _cell_blocks[v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].s_wall = set_cell.north_wall;
    }

    if(set_cell.west_wall == WallState::HI){
        _cell_blocks[MAZE_SIDE_LENGTH + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].e_wall = set_cell.west_wall;
    }

    if(set_cell.east_wall == WallState::HI){
        _cell_blocks[MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].e_wall = set_cell.east_wall;
    }
        
    if(set_cell.south_wall == WallState::HI){
        _cell_blocks[MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].s_wall = set_cell.south_wall;
    }
}

void Maze::GetCell(Cell& get_cell, const Vec2 v) const{
    if(cell_request_is_out_of_range_cell_blocks(v)) return;

    get_cell.north_wall = _cell_blocks[v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].s_wall;

    get_cell.west_wall = _cell_blocks[MAZE_SIDE_LENGTH + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].e_wall;

    get_cell.east_wall = _cell_blocks[MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].e_wall;  

    get_cell.south_wall = _cell_blocks[MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].s_wall;
}

void Maze::PrimaryFill(){
    for(uint16_t i = 0; i < MAZE_TOTAL_SIZE; i++){
        _cell_blocks[i].is_def_cell_dir = DirectionState::UNDEF;
    }

    for(uint16_t i = 0; i < MAZE_TOTAL_SIZE; i++){
        _cell_blocks[i].is_cell_passed = false;
    }

    /*fill north wall fronts*/
    for(uint16_t i = 0; i < MAZE_SIDE_LENGTH; i++){
        _cell_blocks[i].s_wall = WallState::HI;
    }

    /*fill west wall fronts*/
    for(uint16_t i = MAZE_SIDE_LENGTH; i <= MAZE_SIDE_LENGTH_ADD_ONE * MAZE_SIDE_LENGTH; i+=MAZE_SIDE_LENGTH_ADD_ONE){
        _cell_blocks[i].e_wall = WallState::HI;
    }
    
    /*fill east wall fronts*/
    for(uint16_t i = MAZE_SIDE_LENGTH_ADD_ONE + (MAZE_SIDE_LENGTH - 1); 
        i <= (MAZE_SIDE_LENGTH_ADD_ONE + (MAZE_SIDE_LENGTH) * MAZE_SIDE_LENGTH_ADD_ONE); i += MAZE_SIDE_LENGTH_ADD_ONE){
        _cell_blocks[i].e_wall = WallState::HI;   
    }

    /*fill south wall fronts*/
    for(uint16_t i = (MAZE_TOTAL_SIZE - MAZE_SIDE_LENGTH); i < MAZE_TOTAL_SIZE; i++){
        _cell_blocks[i].s_wall = WallState::HI;   
    }
}

void Maze::Clear(){
    PrimaryFill();
    ClearPath();
}

void Maze::UndefDirs(){
    for(uint8_t i = 0; i < MAZE_TOTAL_SIZE; i++){
        _cell_blocks[i].is_def_cell_dir = DirectionState::UNDEF;
    }
}

void Maze::SetCellDir(const Direction direction, const Vec2 v){
    if(cell_request_is_out_of_range_cell_blocks(v)) return;

    _buf_cell_ptr = (_cell_blocks + MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE);
    
    _buf_cell_ptr->is_def_cell_dir = DirectionState::DEF;
    _buf_cell_ptr->cell_dir = direction;
}

void Maze::GetCellDir(DirectionStore& direction_store, const Vec2 v) const{
    if(cell_request_is_out_of_range_cell_blocks(v)) return;

    _buf_cell_ptr = const_cast<RawCellStore*>(_cell_blocks + MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE);

    direction_store.is_def_cell_dir = _buf_cell_ptr->is_def_cell_dir;
    direction_store.cell_dir = _buf_cell_ptr->cell_dir;
}

void Maze::UndefCell(const Vec2 v){
    if(cell_request_is_out_of_range_cell_blocks(v)) return;

    _buf_cell_ptr = (_cell_blocks + MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE);
    _buf_cell_ptr->is_def_cell_dir = DirectionState::UNDEF;
}

void Maze::PushBackPathDir(const Direction dir){
    if(_path_ind + 1 < MAZE_PATH_SIZE){
        _cell_blocks[_path_ind++].path_dir = static_cast<RawCellStore::PathDirStore>((static_cast<uint8_t>(dir) & 0b10) >> 1);
        _cell_blocks[_path_ind++].path_dir = static_cast<RawCellStore::PathDirStore>((static_cast<uint8_t>(dir) & 0b1));
    }
}

void Maze::SetPathDir(const Direction dir, uint8_t ind){
    if(ind < MAZE_PATH_SIZE){
        _cell_blocks[ind*2].path_dir     = static_cast<RawCellStore::PathDirStore>((static_cast<uint8_t>(dir) & 0b10) >> 1);
        _cell_blocks[ind*2 + 1].path_dir = static_cast<RawCellStore::PathDirStore>((static_cast<uint8_t>(dir) & 0b1));
    }
}

void Maze::GetPathDir(Direction& dir, uint8_t ind) const{
    if(ind < MAZE_PATH_SIZE){
        dir = static_cast<Direction>(
            (static_cast<uint8_t>(_cell_blocks[ind*2].path_dir) << 1) | 
             static_cast<uint8_t>(_cell_blocks[ind*2 + 1].path_dir));
    }
}

Direction Maze::GetPathDir(uint8_t ind) const{
    if(ind >= MAZE_PATH_SIZE) return {};
    
    _buf_path_direction_store = static_cast<Direction>(
        (static_cast<uint8_t>(_cell_blocks[ind*2].path_dir) << 1) | 
        static_cast<uint8_t>(_cell_blocks[ind*2 + 1].path_dir));
    return _buf_path_direction_store;
}

bool Maze::CellIsPassed(const Vec2 v) const{
    if(cell_request_is_out_of_range_cell_blocks(v)) return true;

    return _cell_blocks[MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].is_cell_passed;
}

void Maze::PassCell(const Vec2 v) noexcept{
    if(cell_request_is_out_of_range_cell_blocks(v)) return;

    _cell_blocks[MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].is_cell_passed = true;
}


void Maze::ClearPath(){
    _path_ind = 0;
}

uint8_t Maze::GetPathSize() const{
    return _path_ind / 2;
}

void Maze::PrintDirPath() const{
    Serial.print("path: ");   
    
    for(uint8_t i = 0; i < _path_ind / 2; i ++){
        _buf_path_direction_store = static_cast<Direction>(
                                   (static_cast<uint8_t>(_cell_blocks[i * 2].path_dir) << 1) |
                                    static_cast<uint8_t>(_cell_blocks[i * 2 + 1].path_dir));
              
        switch (_buf_path_direction_store)
        {
        case Direction::N:
            Serial.print("N");   
            break;

        case Direction::E:
            Serial.print("E");
            break;

        case Direction::S:
            Serial.print("S");
            break;
        
        case Direction::W:
            Serial.print("W");
            break;
            
        }
        Serial.print(" ");
    }
    Serial.println();
}

void Maze::PrintCell(const Vec2 v) const{
    /*print north wall front*/
    Serial.print(ANGLE);
    if(_cell_blocks[v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].s_wall == WallState::HI){
        Serial.print(HORIZ_WALL);
    }
    else{
        Serial.print(BLANK_HORIZ_WALL);
    }

    Serial.println(ANGLE);

    /*print west wall front*/
    if(_cell_blocks[MAZE_SIDE_LENGTH + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].e_wall == WallState::HI){
        Serial.print(VERTIC_WALL);
    }
    else{
        Serial.print(BLANK_VERTIC_WALL);
    }

    print_cell_path(v);

    /*print east wall front*/
    if(_cell_blocks[MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].e_wall == WallState::HI){
        Serial.println(VERTIC_WALL);
    }
    else{
        Serial.println(BLANK_VERTIC_WALL);
    }

    Serial.print(ANGLE);

    /*print north wall front*/
    if(_cell_blocks[MAZE_SIDE_LENGTH_ADD_ONE + v.x + v.y * MAZE_SIDE_LENGTH_ADD_ONE].s_wall == WallState::HI){
        Serial.print(HORIZ_WALL);
    }
    else{
        Serial.print(BLANK_HORIZ_WALL);
    }

    Serial.println(ANGLE);
}

void Maze::Print() const{
    Serial.print("   ");
    for(uint8_t i = 0; i < MAZE_SIDE_LENGTH; i++){
        Serial.print("   " + String(i) + "  ");
    }

    Serial.println();
    Serial.print(VOID);
    for(uint8_t x = 0; x < MAZE_SIDE_LENGTH; x++){
        print_cell_north_wall({x, 0});
    }

    Serial.print(ANGLE);
    Serial.println();

    for(uint8_t y = 0; y < MAZE_SIDE_LENGTH; y++){
        PRINT_N_SPACES(3 - String(y * MAZE_SIDE_LENGTH).length());
        Serial.print(y * MAZE_SIDE_LENGTH);

        for(uint8_t x = 0; x < MAZE_SIDE_LENGTH; x++){
            print_cell_middle_walls({x, y});
            
            GetCellDir(_buf_direction_store, {x, y});
            print_cell_path({x, y});
        }            
        
        print_cell_middle_walls({MAZE_SIDE_LENGTH, y});
        Serial.println();

        Serial.print(VOID);
        for(uint8_t x = 0; x < MAZE_SIDE_LENGTH; x++){
            print_cell_south_wall({x, y});
        }
        Serial.print(ANGLE);
        Serial.println();
    }
    Serial.println();
}

uint8_t Maze::Vec2ToInd(Vec2 v){
    return v.x + v.y * MAZE_SIDE_LENGTH;
}

Vec2 Maze::IndToVec2(uint8_t ind){
    return {ind % MAZE_SIDE_LENGTH, ind / MAZE_SIDE_LENGTH};
}