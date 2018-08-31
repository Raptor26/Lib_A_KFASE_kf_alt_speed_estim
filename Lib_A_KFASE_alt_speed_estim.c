/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "Lib_A_KFASE_alt_speed_estim.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
static void
KFASE_InitMatrixCovarianseP (
	kfase_alt_speed_estimate_s *p_s,
	float eye);

static void
KFASE_InitMatrixCovarianseQ(
	kfase_alt_speed_estimate_s *p_s,
	float eye);

static void
KFASE_InitMatrixCovarianseR(
	kfase_alt_speed_estimate_s *p_s,
	float eye);

static void
KFASE_InitStates(
	kfase_alt_speed_estimate_s *p_s,
	float alt,
	float speed);

static void
KFASE_GetPredict (
	kfase_alt_speed_estimate_s *p_s,
	float accWorldFrame);

static void
KFASE_HalfCovarUpdate (
	kfase_alt_speed_estimate_s *p_s);

static void
KFASE_CalcKalmanGain (
	kfase_alt_speed_estimate_s *p_s);

static void
KFASE_UpdateEstimate (
	kfase_alt_speed_estimate_s *p_s,
	float altBaro);

static void
KFASE_FullCovarUpdate (
	kfase_alt_speed_estimate_s *p_s);
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
void
KFASE_Init_KF(
	kfase_alt_speed_estimate_s *p_s,
	float acc_Q_Noise,
	float baro_R_Noise,
	float covar_P_Noise,
	float alt,
	float speed,
	float dT)
{
	/* Инициализация матрицы ковариаций P */
	KFASE_InitMatrixCovarianseP(
		p_s,
		covar_P_Noise);

	/* Инициализация матрицы ковариаций акселерометра */
	KFASE_InitMatrixCovarianseQ(
		p_s,
		acc_Q_Noise);

	/* Инициализация матрицы ковариаций барометра */
	KFASE_InitMatrixCovarianseR(
		p_s,
		baro_R_Noise);

	/* Инициализация высоты и скорости */
	KFASE_InitStates(
		p_s,
		alt,
		speed);
	
	/* Инициализация периода интегрирования показаний акселерометра */
	p_s->dT = dT;
	p_s->dTdT = dT * dT;
}

float
KFASE_GetAltEstimate(
	kfase_alt_speed_estimate_s *p_s)
{
	return (p_s->states_x_a[KFASE_ESTIMATE_ALT]);
}

float
KFASE_GetVerticalSpeedEstimate(
	kfase_alt_speed_estimate_s *p_s)
{
	return (p_s->states_x_a[KFASE_ESTIMATE_SPEED]);
}

void
KFASE_GetPredictWithCorrect (
	kfase_alt_speed_estimate_s *p_s,
	float accWorldFrame,
	float altBaro)

{
	/* Обновление оценки состояний */
	KFASE_GetPredict (
		p_s,
		accWorldFrame);

	/* Обновление коэффициентов усиления фильтра Калмана */
	KFASE_CalcKalmanGain(
		p_s);

	/* Обновление оценки с помощью измерений */
	KFASE_UpdateEstimate(
		p_s,
		altBaro);

	/* Полное обновление матрицы ковариаций */
	KFASE_FullCovarUpdate(
		p_s);
}


/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/

void
KFASE_InitMatrixCovarianseP (
	kfase_alt_speed_estimate_s *p_s,
	float eye)
{
	/* Инициализация матрицы ковариаций как диагональную */
	p_s->covarianse_P_a[0][0] = eye;
	p_s->covarianse_P_a[0][1] = 0.0f;
	p_s->covarianse_P_a[1][0] = 0.0f;
	p_s->covarianse_P_a[1][1] = eye;
}

void
KFASE_InitMatrixCovarianseQ(
	kfase_alt_speed_estimate_s *p_s,
	float eye)
{
	/* Инициализация матрицы ковариаций как диагональную */
	p_s->accCovarianse_Q_a2[0][0] = eye;
	p_s->accCovarianse_Q_a2[0][1] = 0.0f;
	p_s->accCovarianse_Q_a2[1][0] = 0.0f;
	p_s->accCovarianse_Q_a2[1][1] = eye;
}

void
KFASE_InitMatrixCovarianseR(
	kfase_alt_speed_estimate_s *p_s,
	float eye)
{
	/* Инициализация матрицы ковариаций как диагональную */
	p_s->baroCovarianse_R_a2[0][0] = eye;
	p_s->baroCovarianse_R_a2[0][1] = 0.0f;
	p_s->baroCovarianse_R_a2[1][0] = 0.0f;
	p_s->baroCovarianse_R_a2[1][1] = eye;
}

void
KFASE_InitStates(
	kfase_alt_speed_estimate_s *p_s,
	float alt,
	float speed)
{
	p_s->states_x_a[KFASE_ESTIMATE_ALT] 	= alt;
	p_s->states_x_a[KFASE_ESTIMATE_SPEED] 	= speed;
}

void
KFASE_GetPredict (
	kfase_alt_speed_estimate_s *p_s,
	float accWorldFrame)
{
	/* Обновление оценки высоты */
	p_s->states_x_a[KFASE_ESTIMATE_ALT] =
		((accWorldFrame * p_s->dTdT) * 0.5f)
		+ p_s->states_x_a[KFASE_ESTIMATE_SPEED] * p_s->dT
		+ p_s->states_x_a[KFASE_ESTIMATE_ALT];

	/* Обновление оценки скорости */
	p_s->states_x_a[KFASE_ESTIMATE_SPEED] =
		p_s->states_x_a[KFASE_ESTIMATE_SPEED] + accWorldFrame * p_s->dT;

	/* Частичное обновление матрицы ковариаций */
	KFASE_HalfCovarUpdate(
		p_s);
}

void
KFASE_HalfCovarUpdate (
	kfase_alt_speed_estimate_s *p_s)
{
	/* Временная переменная для матрицы ковариаций */
	float P_temp[2][2] =
	{
		{p_s->covarianse_P_a[0][0], p_s->covarianse_P_a[0][1]},
		{p_s->covarianse_P_a[1][0], p_s->covarianse_P_a[1][1]}
	};

	p_s->covarianse_P_a[0][0] =
		p_s->accCovarianse_Q_a2[0][0]
		+ P_temp[0][0]
		+ p_s->dT * (P_temp[0][1] + P_temp[1][1] * p_s->dT)
		+ P_temp[1][0] * p_s->dT;

	p_s->covarianse_P_a[0][1] =
		p_s->accCovarianse_Q_a2[0][1]
		+ P_temp[0][1]
		+ P_temp[1][1] * p_s->dT;

	p_s->covarianse_P_a[1][0] =
		p_s->accCovarianse_Q_a2[1][0]
		+ P_temp[1][0]
		+ P_temp[1][1] * p_s->dT;

	p_s->covarianse_P_a[1][1] =
		p_s->accCovarianse_Q_a2[1][1]
		+ P_temp[1][1];
}

void
KFASE_CalcKalmanGain (
	kfase_alt_speed_estimate_s *p_s)
{
	p_s->kalmanGain_K_a[KFASE_KALMAN_GAIN_ALT] =
		(p_s->covarianse_P_a[0][0] + p_s->covarianse_P_a[1][0] * p_s->dT) / p_s->covarianse_P_a[0][0];
//		+ 1.0f;

	p_s->kalmanGain_K_a[KFASE_KALMAN_GAIN_SPEED] =
		p_s->covarianse_P_a[1][0] / p_s->covarianse_P_a[0][0];
}

void
KFASE_UpdateEstimate (
	kfase_alt_speed_estimate_s *p_s,
	float altBaro)
{
	p_s->states_x_a[KFASE_ESTIMATE_ALT] =
		p_s->states_x_a[KFASE_ESTIMATE_ALT]
		- p_s->kalmanGain_K_a[KFASE_KALMAN_GAIN_ALT] * (p_s->states_x_a[KFASE_ESTIMATE_ALT] - altBaro);

	p_s->states_x_a[KFASE_ESTIMATE_SPEED] =
		p_s->states_x_a[KFASE_ESTIMATE_SPEED]
		- p_s->kalmanGain_K_a[KFASE_ESTIMATE_SPEED] * (p_s->states_x_a[KFASE_ESTIMATE_ALT] - altBaro);
}

void
KFASE_FullCovarUpdate (
	kfase_alt_speed_estimate_s *p_s)
{
	/* TODO избавиться от временных переменных матрицы ковариаций
	 * (после того как будет проверена работоспособность фильтра) */
	/* Временная переменная для матрицы ковариаций */
	float P_temp[2][2] =
	{
		{p_s->covarianse_P_a[0][0], p_s->covarianse_P_a[0][1]},
		{p_s->covarianse_P_a[1][0], p_s->covarianse_P_a[1][1]}
	};

	p_s->covarianse_P_a[0][0] -=
		(P_temp[0][0] * (p_s->kalmanGain_K_a[KFASE_KALMAN_GAIN_ALT]));

	p_s->covarianse_P_a[0][1] -=
		(P_temp[0][1] * (p_s->kalmanGain_K_a[KFASE_KALMAN_GAIN_ALT]));

	p_s->covarianse_P_a[1][0] -=
		P_temp[0][0] * p_s->kalmanGain_K_a[KFASE_KALMAN_GAIN_SPEED];

	p_s->covarianse_P_a[1][1] -=
		P_temp[0][1] * p_s->kalmanGain_K_a[KFASE_KALMAN_GAIN_SPEED];
}
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
