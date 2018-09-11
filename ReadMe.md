Lib_A_KFASE_alt_speed_estim

`KFASE` - Kalman filter altitude speed estimate

Программный модуль, реализующий оценку вертикальной скорости и высоты по показаниям акселерометра (вдоль вертикальной оси) и барометрического высотомера с помощью линейного фильтра Калмана

Пример исходного кода для работы с библиотекой:

**Замечание:** после подключения программного модуля в настройках среды разработки необходимо объявить `__KFASE_FLOAT_POINT_TYPE__` **равным** `float` или `double`

```C
/* Подключение программного модуля */
#include "../Lib_A_KFASE_kf_alt_speed_estim/Lib_A_KFASE_alt_speed_estim.h" 

/* Объявление структуры для работы с программным 
 * модулем фильтра Калмана */
kfase_alt_speed_estimate_s altSpeedEstim_KF_s;

int main(void)
{
	...
	/* Инициализация фильтра Калмана начальными значениями */
	KFASE_Init_KF
	(
		&altSpeedEstim_KF_s,						// Указатель на структуру
		(__KFASE_FLOAT_POINT_TYPE__) acc_Q_Noise,	// Шумовая характеристика акселерометра ()
		(__KFASE_FLOAT_POINT_TYPE__) baro_R_Noise,	// Шумовая характеристика барометра
		(__KFASE_FLOAT_POINT_TYPE__) covar_P_Noise,	// Значение, которым будет 
													// проинициализирована 
													// диагональ матрицы 
													// ковариаций
		(__KFASE_FLOAT_POINT_TYPE__) alt,			// Начальное значение 
													// высоты
		(__KFASE_FLOAT_POINT_TYPE__) speed,			// Начальное значение 
													// вертикальной скорости
		(__KFASE_FLOAT_POINT_TYPE__) dT,			// Период интегрирования 
													// показаний акселерометра 
													// для обновления высоты и 
													// вертикальной скорости
	)
	...
	while(1)
	{
		...
		/* Если готовы только показания акселерометра и не готовы показания барометра */
		if ((accMeasureReady_flag == 1) && (baroMeasureReady_flag == 0))
		{
			KFASE_GetPredict
			(
				&altSpeedEstim_KF_s, 
				verticalAcceleration	// Вертикальное ускорения должно 
										// быть выражено в м/сек^2
			);
		}

		/* Если готовы и показания акселерометра и показания барометра */
		else if ((accMeasureReady_flag == 1) && (baroMeasureReady_flag == 1))
		{
			KFASE_GetPredictWithCorrect
			(
				&altSpeedEstim_KF_s, 
				verticalAcceleration, 	// Вертикальное ускорения должно 
										// быть выражено в м/сек^2
				baroAltMeasure			// Высота должна быть выражена в метрах
			);
		}
		...
		/* Получить оценку высоты в метрах */
		float altEstim = 
			(float) KFASE_GetAltEstimate(
				&altSpeedEstim_KF_s);
		...
		/* Получить оценку вертикальной скорости в м/сек */
		float verticalSpeedEstim = 
			(float) KFASE_GetVerticalSpeedEstimate(
				&altSpeedEstim_KF_s);
		...
		}
		return 0;
	}
```