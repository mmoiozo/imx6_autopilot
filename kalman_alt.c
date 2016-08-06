#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <math.h>
#include <string.h> 
#include "AHRS.h"

double alt = 0;
int temp_deg = 0;
int press_pa = 0;

float opt_alt = 0;
float opt_v_speed = 0;


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
void add_vec(float v1[2],float v2[2],float v3[2])
void mul_vec(float v1[2],float v2[2],float c);
void mat_inv(float m1[2][2],float m2[2][2]);

void vec_vec_t(float v1[2],float v2[2],float m1[2][2],int row,int col);
float vec_t_vec(float v1[2],float v2[2]);


   
   float vec_1[2] = {1,2};
   float vec_2[2] = {2,1};
   float trans_mat[2][2];
   
   float res_vec_1[2];// = {0,0,0};
   float res_vec_2[2];
   float res_vec_3[2];
   float res_mat_1[2][2];
   float res_mat_2[2][2];
   float res_mat_3[2][2];
   
   //KALMAN FILTER MATRICES
   
   float x_pred[2] = {0,0};//[alt speed]
   float x_opt[2] = {0,0};//[alt speed]
   float K[2] = {0,0};
   float H[2] = {1,0};
   float GAMMA[2] = {0,0};//[0 dt]//update each timestep
   float PSI[2] = {0,0};//[0 dt]//update each timestep
   float Q = 0.25;
   float R = 0.35;
   
   float PHI[2][2] = {//rows / columns  //A initiallized with zeroes and updated after each itteration with specific time 
   {1, 0} ,     //[1 dt;
   {0, 1}     };// 0 1];  
   
   float P_1[2][2] = {//rows / columns  
   {0, 0} ,     
   {0, 0}     };  
   
   float P[2][2] = {//rows / columns initialize with variances on main diagonal?  
   {1.5, 0} ,     
   {0,   1}     }; 
   
   float eye[2][2] = {//rows / columns initialize with variances on main diagonal?  
   {1, 0} ,     
   {0, 1}     }; 
   
   
void get_alt(double dt)
{
  long temp = 0;
  long press = 0;
  double po = 102100;//de bilt //101325;//icao
  
    /*
    if(acc_count < 1500)
    {
    z_acc_comp = -((((dot/angle_vec)/15700)*9.81)-9.81);//in m/s2//16384
    z_speed_comp += z_acc_comp*dt;
    acc_count +=1;
    }
    else
    {
      //z_acc_comp /= acc_count;
      //printf("Z acc comp: %f\n",z_acc_comp);
      //printf("Z speed comp: %f\n",z_speed_comp);
      z_speed_comp = 0;
      acc_count = 0;
      //z_acc_comp = 0;
    }
  */
    bmp_get(&temp,&press);
    if(press > 0)
    {
    alt = 44330*( 1-pow(((double)press/po),0.19029)); //in meters
    temp_deg = temp;
    press_pa = press;
    z_acc_av /= delta_t
    //calc_alt();
    z_acc_av = 0;
    delta_t  = 0;
    }
  
}
   
   //Kalman filter altitude and vertical speed calculation
   void calc_alt()
   {
     PHI[1][1] = delta_t;
     PSI[1]    = delta_t;
     GAMMA[1]  = delta_t;
     
     //state prediction
     mul_vec(PSI,res_vec_1,z_acc_av);
     mul_mat_vec(PHI,x_opt,res_vec_2,2,2);
     add_vec(res_vec_1,res_vec_2,x_pred);
     
     //covariance prediction
     transpose_arr(PHI,res_mat_1,2,2);
     mul_arr(P,res_mat_1,res_mat_2,2,2);
     mul_arr(PHI,res_mat_2,res_mat_1,2,2);
     
     mul_vec(GAMMA,res_vec_1,Q);
     vec_vec_t(res_vec_1,GAMMA,res_mat_2,2,2);
     
     add_arr(res_mat_1,res_mat_2,P_1,2,2);
     
     //kalman gain calculation
     mul_mat_vec(P_1,H,res_vec_1,2,2);
     float res = vec_t_vec(H,res_vec_1);
     float inv = 1/(res+R);
     
     mul_vec(H,res_vec_1,inv);
     mul_mat_vec(P_1,res_vec_1,K,2,2);
     
     //optimal state
     float diff = alt-x_opt[0];
     mul_vec(K,res_vec_1,diff);
     add_vec(res_vec_1,x_pred,x_opt);
     
     opt_alt     = x_opt[0];
     opt_v_speed = x_opt[1];
     
     //optimal covariance update
     vec_vec_t(K,H,res_mat_1,2,2);
     sub_arr(eye,res_mat_1,res_mat_2,2,2);
     mul_arr(res_mat_1,P_1,P,2,2);
     
   }
  /*
   printf("print test:\n");
   print_arr_f(mat,2,2);
   
   transpose_arr(mat,trans_mat,2,2);
   printf("\nTranspose test:\n");
   print_arr_f(trans_mat,2,2);
   
   printf("Multiplication [mat_1]*[mat_2] test:\n");
   printf("mat_1:\n");
   print_arr_f(mat_1,2,2);
   printf("mat_2:\n");
   print_arr_f(mat_2,2,2);
   printf("vec_1:\n");
   print_vec_f(vec_1,2);
   printf("vec_2:\n");
   print_vec_f(vec_2,2);
   mul_arr(mat_1,mat_2,res_mat,2,2);
   printf("Result:\n");
   print_arr_f(res_mat,2,2);
   
   printf("Addition [mat_1]+[mat_2] test:\n");
   add_arr(mat_1,mat_2,res_mat,2,2);
   printf("Result:\n");
   print_arr_f(res_mat,2,2);
   
   printf("Substraction [mat_1]-[mat_2] test:\n");
   sub_arr(mat_1,mat_2,res_mat,2,2);
   printf("Result:\n");
   print_arr_f(res_mat,2,2);
   
   printf("Multiplication [mat_1]*[vec_1] test:\n");
   mul_mat_vec(mat_1,vec_1,res_vec,2,2);
   printf("Result:\n");
   print_vec_f(res_vec,2);
   
   printf("Substraction [vec_1]-[vec_2] test:\n");
   sub_vec(vec_1,vec_2,res_vec);
   printf("Result:\n");
   print_vec_f(res_vec,2);
   
   printf("Vector addition [vec_1]+[vec_2] test:\n");
   add_vec(vec_1,vec_2,res_vec);
   printf("Result:\n");
   print_vec_f(res_vec,2);
   
   //mat_inv(mat_1,res_mat_f);
   //printf("Result:\n");
   //print_arr_f(res_mat_f,2,2);
   
   printf("Multiplication [vec_1]*[vec_2]T test:\n");
   vec_vec_t(vec_1,vec_2,res_mat,2,2);
   printf("Result:\n");
   print_arr_f(res_mat,2,2);
   
   printf("Multiplication vec constant [vec_1]*5 test:\n");
   mul_vec(vec_1,res_vec,5);
   printf("Result:\n");
   print_vec_f(res_vec,2);
   
   printf("Multiplication [vec_1]T*[vec_2] test:\n");
   float res = vec_t_vec(vec_1,vec_2);
   printf("Result:%f\n",res);
   
   */


//Matrix vector multiplication
void mul_mat_vec(float m1[2][2],float v1[2],float v2[2],int row,int col)
{
    int i,j,k;
    memset(v2, 0,sizeof(v2)*sizeof(v2[0]));//set res vector to zero
    
    for(i=0;i<row;i++)
    {
      for(j=0;j<col;j++)
      {
	v2[i] = v2[i] + (m1[i][j] * v1[j]);
      }
    }
}

//Matrix matrix multiplication 2x2
void mul_arr(float m1[2][2],float m2[2][2],float m3[2][2],int row,int col)
{
    int i,j,k;
    memset(m3, 0, row*col*sizeof m3[0][0]);//set res matrix to zero
    for(i=0;i<row;i++)
    {
      for(j=0;j<col;j++)
      {
	for (k=0;k<row;k++)
	{
	m3[i][j] = m3[i][j] + (m1[i][k] * m2[k][j]);
	}
      }
    }
}

void vec_vec_t(float v1[2],float v2[2],float m1[2][2],int row,int col)
{
    int i,j,k;
    memset(m1, 0, row*col*sizeof m1[0][0]);//set res matrix to zero
    for(i=0;i<row;i++)
    {
      for(j=0;j<col;j++)
      {
	m1[i][j] = v1[i]*v2[j];
      }
    }
}

//mul_arr(m1,m2,m2,row,col);

//Matrix matrix addition 2x2

void add_arr(float m1[2][2],float m2[2][2],float m3[2][2],int row,int col)
{
    int i,j;
    for(i=0;i<row;i++)
    {
    for(j=0;j<col;j++)
    {
    m3[i][j] =  (m1[i][j] + m2[i][j]);
    }
    }
}

//Matrix substraction 2x2
void sub_arr(float m1[2][2],float m2[2][2],float m3[2][2],int row,int col)
{
    int i,j;
    for(i=0;i<row;i++)
    {
    for(j=0;j<col;j++)
    {
    m3[i][j] =  (m1[i][j] - m2[i][j]);
    }
    }
}

//Vector constant multiplication

void mul_vec(float v1[2],float v2[2],float c)
{
    int i;
    for(i=0;i<2;i++)
    {
     v2[i] =  v1[i]*c;
    }
}

// Transpose vector times vector
float vec_t_vec(float v1[2],float v2[2])
{
    int i;
    float res = 0;
    for(i=0;i<2;i++)
    {
     res += v1[i]*v2[i];
    }
    return res;
}

//Vector vector substraction

void sub_vec(float v1[2],float v2[2],float v3[2])
{
    int i;
    for(i=0;i<2;i++)
    {
     v3[i] =  (v1[i] - v2[i]);
    }
}

//Vector vector addition

void add_vec(float v1[2],float v2[2],float v3[2])
{
    int i;
    for(i=0;i<2;i++)
    {
     v3[i] =  (v1[i] + v2[i]);
    }
}

//Matrix inverse
void mat_inv(float m1[2][2],float m2[2][2])
{
 int a[2][2],i,j;
 float determinant = 0;
 
   for(i=0;i<2;i++)
      determinant = determinant + (m1[0][i]*(m1[1][(i+1)%2]*m1[2][(i+2)%2] - m1[1][(i+2)%2]*m1[2][(i+1)%2]));
 
   printf("\nInverse of matrix is: \n\n");
   for(i=0;i<2;i++){
      for(j=0;j<2;j++)
	
	m2[i][j] = ((m1[(i+1)%2][(j+1)%2] * m1[(i+2)%2][(j+2)%2]) - (m1[(i+1)%2][(j+2)%2]*m1[(i+2)%2][(j+1)%2]))/ determinant;
           printf("%.2f\t",((m1[(i+1)%2][(j+1)%2] * m1[(i+2)%2][(j+2)%2]) - (m1[(i+1)%2][(j+2)%2]*m1[(i+2)%2][(j+1)%2]))/ determinant);
       printf("\n");
   }
}



//Matrix transpose 2x2

void transpose_arr(float m1[2][2],float m2[2][2],int row,int col)
{
    int i,j;
    for(i=0;i<row;i++)
    {
    for(j=0;j<col;j++)
    {
    m2[i][j] = m1[j][i];
    }
    }
}

//testing
//print 2x2 mat
void print_arr(int m[2][2],int row,int col)
{
    int i,j;
    for(i=0;i<row;i++)
        {
        for(j=0;j<col;j++)
        {
            printf("%d ",m[i][j]);
         }
        printf("\n");
        }
}

void print_arr_f(float m[2][2],int row,int col)
{
    int i,j;
    for(i=0;i<row;i++)
        {
        for(j=0;j<col;j++)
        {
            printf("%f ",m[i][j]);
         }
        printf("\n");
        }
}

void print_vec(int vec[2],int n)
{
    int i;
    for(i=0;i<n;i++)
        {
            printf("%d ",vec[i]);
        printf("\n");
        }
}

void print_vec_f(float vec[2],int n)
{
    int i;
    for(i=0;i<n;i++)
        {
            printf("%f ",vec[i]);
        printf("\n");
        }
}


