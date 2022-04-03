#ifndef LASERL1_H
#define LASERL1_H
/*---------------------INCLUDES----------------------*/
#include "arm_math.h"

/*---------------------VARIABLES---------------------*/


const float p_0 = 1.01325f;//1��׼����ѹ
const float T_0 = 273.15f;
const float C = 0.47f;//�������ϵ��
const float MS = 0.0032f;//17mm������
const float DS = 0.017f;//17mm
const float MB_N = 0.041f;//42mm������
const float DB_N = 0.042f;//42mm������
const float MB_F = 0.043f;//42mm����
const float DB_F = 0.042f;//42mm����

#define SERVO_ERROR 12

#define iLD_1 7
#define iLD_0 6
#define iCM 2
#define iACM 3
#define iFACM 4
#define iHALT 5
#define iINT 10

#define LASER_BUF_NUM 8
#define LASER_BUF_LEN 8
#define LASER_FIFO_BUF_NUM 32
#define LASER_FIFO_BUF_LEN 32

#define AMMOG 9.7803f //�������ٶ�
#define AMMO1ONE 1.0f  //1
#define AMMOHUNDRED 100.0f  //100
#define AMMOTHOUSAND 1000.0f //1000
#define AMMOLEVEL 30.0f //ˮƽ
#define AMMOPI 3.14159265358979f //��
#define AMMO180 180.0f //180


//180
#ifndef PI
#define PI 3.14159265358979f
#endif
//90
#ifndef PIp2
#define PIp2 1.57079632679489f
#endif
//45
#ifndef PIp4
#define PIp4 0.78539816339744f
#endif
//-90
#ifndef PIp2_negative
#define PIp2_negative -1.5707963267948f
#endif
//-15
#ifndef PIp12_negative
#define PIp12_negative -0.2617993877991f
#endif
//30
#ifndef PIp6
#define PIp6 0.52359877559829f
#endif
//-30
#ifndef PIp6_negative
#define PIp6_negative -0.5235987755982f
#endif
//1 rad
#ifndef ONERAD
#define ONERAD 57.2957795130823f
#endif

/*---------------------FUNCTIONS---------------------*/
//���ݽṹ��


typedef struct  
{
 const float p_0;
 const float T_0;
 const float C;
 const float MS;
 const float DS;
 const float MB_N;
 const float DB_N;
 const float MB_F;
 const float DB_F;
	
 float p_t;//�ֳ���ѹ
 float T_t;//�ֳ��¶�
 float row;//�����ܶ�
 float m;//�ӵ�����
 float D;//�ӵ�ֱ��
 float area;//�������
 float k;//���
 int Choose;
}trajecyory_constant ;

typedef struct
{

//  float GL_angel;        //�����ˮƽ�Ƕ� �Ƕ�ֵ	
  float Caluculat_angel; //�����,�ڹܶ�ˮƽ �Ƕ���
  float TOA;             //�ڵ�����ʱ��

  float L1_Distance;  //�����õĳ���
  float L1_Angel;     //����������ʱ��ˮƽ�ļн� �Ƕ�ֵ
  float L1_AmmoSpeed; //����
  uint8_t Server;     //������

  float GB_angel;       //ǹ�ܶ�ˮƽ�Ƕ� �Ƕ�ֵ
  float BL_angel;       //�����ǹ�ܽǶȣ���λ90�㣩 �Ƕ�ֵ
  float Delta_B;        //ʵʱ���� �����ڹܽǶ���ʵ�ʽǶȲ� ����
  uint8_t L1_ACM_State; //�����Ƿ����
  uint8_t L1_Operation; //ָ��

  uint8_t E_FPS; //��Ч֡��
  uint8_t tE_FPS;

} L1_DATA_T;
/*�˽ṹ����Ҫ������ϵ���ʹ��*/
typedef struct
{
  uint16_t TotalCalcu_Numbers;      //�������
  uint16_t TotalCalcu_last_numbers; //�ϴμ������

  uint16_t Flag_timer;
  uint16_t Beat_timer;

  uint8_t Calcu_flag;

  float precision;
  uint16_t fps;
  uint16_t Pfps;

} L1_ITERATION_T;








extern L1_DATA_T L1_Data;
extern L1_ITERATION_T L1_Iteration;

void Angel_approx(L1_DATA_T *L1_Data, L1_ITERATION_T *L1_Iteration,trajecyory_constant *constant);
void init(trajecyory_constant *constant);
float angel_change(L1_DATA_T *L1_Data);
#endif
