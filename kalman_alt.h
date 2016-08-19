#include <sys/types.h>


void print_arr(int m[2][2],int row,int col);//proto
void print_vec(int vec[2],int n);
void print_arr_f(float m[2][2],int row,int col);
void print_vec_f(float vec[2],int n);
void transpose_arr(float m1[2][2],float m2[2][2],int row,int col);
void mul_arr(float m1[2][2],float m2[2][2],float m3[2][2],int row,int col);
void add_arr(float m1[2][2],float m2[2][2],float m3[2][2],int row,int col);
void sub_arr(float m1[2][2],float m2[2][2],float m3[2][2],int row,int col);
void mul_mat_vec(float m1[2][2],float v1[2],float v2[2],int row,int col);
void sub_vec(float v1[2],float v2[2],float v3[2]);
void add_vec(float v1[2],float v2[2],float v3[2]);
void mul_vec(float v1[2],float v2[2],float c);
void mat_inv(float m1[2][2],float m2[2][2]);

void vec_vec_t(float v1[2],float v2[2],float m1[2][2],int row,int col);
float vec_t_vec(float v1[2],float v2[2]);

extern double alt;
extern int temp_deg;
extern int press_pa;
extern void get_alt(double dt);
void calc_alt();

extern float opt_alt;
extern float opt_v_speed;