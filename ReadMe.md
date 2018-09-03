Lib_A_KFASE_alt_speed_estim

`KFASE` - Kalman filter altitude speed estimate

Программный модуль, реализующий оценку вертикальной скорости и высоты по показаниям акселерометра (вдоль вертикальной оси) и барометрического высотомера с помощью фильтра Калмана

Пример исходного кода для работы с библиотекой:

	...
	/* Объявление структуры для работы с программным 
	 * модулем фильтра Калмана */
	kfase_alt_speed_estimate_s altSpeedEstim_KF_s;
	...
	
	int main (void)
	{
	...
	/* Инициализация фильтра Калмана начальными значениями */
	KFASE_Init_KF
	(
		&altSpeedEstim_KF_s,						// Указатель на структуру
		(__KFASE_FLOAT_POINT_TYPE__) acc_Q_Noise,	// Шумовая характеристика акселерометра ()
		(__KFASE_FLOAT_POINT_TYPE__) baro_R_Noise,	// Шумовая характристика барометра
		(__KFASE_FLOAT_POINT_TYPE__) covar_P_Noise,	// Значение, которым будет проинициализирована 
													// диагональ матрицы ковариаций
		(__KFASE_FLOAT_POINT_TYPE__) alt,			// Начальное значение высоты
		(__KFASE_FLOAT_POINT_TYPE__) speed,			// Начальное значение векртикальной скорости
		(__KFASE_FLOAT_POINT_TYPE__) dT,			// Период интегрирования показаний акселерометра 
													// для обновления высоты и вертикальной скорости
	)
	...
		while(1)
		{
			...
			/* Если готовы только показания акселерометра и не готовы показания барометра */
			if (accMeasureReady_flag == 1)
			{
				KFASE_GetPredict(
					&altSpeedEstim_KF_s, 
					verticalAcceleration);
			}

			/* Если готовы и показания акселерометра и показания барометра */
			if ((accMeasureReady_flag == 1) && (baroMeasureReady_flag == 1))
			{
				KFASE_GetPredictWithCorrect(
					&altSpeedEstim_KF_s, 
					verticalAcceleration, 
					baroAltMeasure);
			}
			...
			/* Получить оценку высоты */
			float altEstim = 
				(float) KFASE_GetAltEstimate(
					&altSpeedEstim_KF_s);
			...
			/* Получить оценку вертикальной скорости */
			float verticalSpeedEstim = 
				(float) KFASE_GetVerticalSpeedEstimate(
					&altSpeedEstim_KF_s);
			...
		}
		return 0;
	}