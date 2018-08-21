/** 
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version	
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


#ifndef LIB_A_KFALT_KF_ALTITUDE_H_
#define LIB_A_KFALT_KF_ALTITUDE_H_


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
	float states_x_a[2];

	float dT;

	float accCovarianse_Q_a2[2][2],
	baroCovarianse_R_a2[2][2];

	float covarianse_P_a[2][2];

	float kalmanGain_K_a[2];
}kfase_alt_speed_estimate_s;
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/

extern void
KFASE_GetPredict (
	kfase_alt_speed_estimate_s *p_s,
	float accWorldFrame,
	float dt);

extern void
KFASE_HalfCovarUpdate (
	kfase_alt_speed_estimate_s *p_s,
	float dt);

extern void
KFASE_CalcKalmanGain (
	kfase_alt_speed_estimate_s *p_s);

extern void
KFASE_UpdateEstimate (
	kfase_alt_speed_estimate_s *p_s,
	float altBaro);

extern void
KFASE_FullCovarUpdate (
	kfase_alt_speed_estimate_s *p_s);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* LIB_A_KFALT_KF_ALTITUDE_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
