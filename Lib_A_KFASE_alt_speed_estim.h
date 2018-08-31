/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


#ifndef LIB_A_KFASE_KF_ALT_SPEED_ESTIM_H_
#define LIB_A_KFASE_KF_ALT_SPEED_ESTIM_H_


/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
enum
{
	KFASE_KALMAN_GAIN_ALT = 0,
	KFASE_KALMAN_GAIN_SPEED,
};

enum
{
	KFASE_ESTIMATE_ALT = 0,
	KFASE_ESTIMATE_SPEED,
};
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
typedef struct
{
	/* Пространство состояний системы, а именно высота и вертикальная скорость */
	float states_x_a[2];

	/* Период интегрирования показаний акселерометра (сек) */
	float dT;

	/* Квадрат периода интегрирования показаний акселерометра (сек) */
	float dTdT;

	float accCovarianse_Q_a2[2][2],
		  baroCovarianse_R_a2[2][2];

	float covarianse_P_a[2][2];

	float kalmanGain_K_a[2];
} kfase_alt_speed_estimate_s;
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
extern void
KFASE_Init_KF(
	kfase_alt_speed_estimate_s *p_s,
	float acc_Q_Noise,
	float baro_R_Noise,
	float covar_P_Noise,
	float alt,
	float speed,
	float dT);

extern void
KFASE_GetPredictWithCorrect(
	kfase_alt_speed_estimate_s *p_s,
	float accWorldFrame,
	float altBaro);

extern float
KFASE_GetAltEstimate(
	kfase_alt_speed_estimate_s *p_s);

extern float
KFASE_GetVerticalSpeedEstimate(
	kfase_alt_speed_estimate_s *p_s);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* LIB_A_KFALT_KF_ALTITUDE_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
