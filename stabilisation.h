#include <sys/types.h>


extern uint8_t gain_P_X;
extern uint8_t gain_i_X;
extern uint8_t gain_D_X;

extern uint8_t gain_P_Y;
extern uint8_t gain_i_Y;
extern uint8_t gain_D_Y;

extern uint8_t gain_P_Z;
extern uint8_t gain_i_Z;

extern float x_control;
extern float y_control;
extern float z_control;


extern void PID_stabilisation(double delta_t);
