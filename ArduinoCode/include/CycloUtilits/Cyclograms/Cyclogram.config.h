#ifndef _CYCLOGRAM_CONFIG_H_
#define _CYCLOGRAM_CONFIG_H_

/*==GENERAL CYCLOGRAM CONSTS BEGIN==*/
#define FORWARD_SPEED 0.25
#define CELL_SIZE 0.18
/*==GENERAL CYCLOGRAM CONSTS END==*/

/*==SEARCH CONSTS BEGIN==*/
#define SEARCH_TURN_RADIUS 0.05
/*==SEARCH CONSTS END==*/

/*==SMART CYCLOGRAM CONSTS BEGIN==*/
#define SS90S_TURN_RADIUS 0.04
#define DD90S_TURN_RADIUS 0.06 // максимум - CELL_SIZE / M_SQRT2
#define SD45S_FORW_DIST (CELL_SIZE / 3) // путь до начала поворота, максимум - CELL_SIZE / 2
#define SD135S_TURN_RADIUS 0.07
#define SS180S_FORW_DIST 0.11
/*==SMART CYCLOGRAM CONSTS END==*/

//*==ANGLE SPEED REGULATOR BEGIN==*/
#define ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K (0.035 + (FORWARD_SPEED * 0.00125)) // [analogValue -> рад/с]
#define ANGLLE_SPEED_OPTOCOUPLER_TWOSEN_REG_K (0.035 + (FORWARD_SPEED * 0.00125)) // [analogValue -> рад/с]  
//*==ANGLE SPEED REGULATOR END==*/

/*==ALIGNMENT CONSTS BEGIN==*/
#define ROBOT_HEIGHT 0.120
#define FROM_WHEEL_TO_SIDE 0.035

#define FROM_BACK_ALIGNMENT_TO_CENTER (HALF(CELL_SIZE) - FROM_WHEEL_TO_SIDE)
#define FROM_FORWARD_ALIGNMENT_TO_CENTER (HALF(CELL_SIZE) - (ROBOT_HEIGHT - FROM_WHEEL_TO_SIDE))

#define FROM_ZERO_WALL_TO_SIDE 0.084 // 0.088

#define BACK_ALIGNMENT_TIME (HALF(CELL_SIZE)/FORWARD_SPEED * 1000 + 500) // ms
#define FORWARD_ALIGNMENT_TIME (HALF(CELL_SIZE)/FORWARD_SPEED * 1000 + 300) // ms
/*==ALIGNMENT CONSTS END==*/

#endif // !_CYCLOGRAM_CONFIG_H_