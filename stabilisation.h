#include <sys/types.h>


extern uint8_t gain_P_X;
extern uint8_t gain_i_X;
extern uint8_t gain_D_X;

extern uint8_t gain_P_Y;
extern uint8_t gain_i_Y;
extern uint8_t gain_D_Y;

extern uint8_t gain_P_Z;
extern uint8_t gain_i_Z;


extern void PID_stabilisation(double delta_t);
