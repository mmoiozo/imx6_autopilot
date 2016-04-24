#include <sys/types.h>


extern uint8_t gain_P_X;
extern uint8_t gain_i_X;
extern uint8_t gain_D_X;

extern uint8_t gain_P_Y;
extern uint8_t gain_i_Y;
extern uint8_t gain_D_Y;

extern uint8_t gain_P_Z;
extern uint8_t gain_i_Z;

extern float pitch_control;
extern float roll_control;
extern float z_control;

extern float i_cmd_pitch;
extern float i_cmd_roll;


extern void PID_stabilisation(double delta_t);
extern void PID_cascaded(double delta_t);
