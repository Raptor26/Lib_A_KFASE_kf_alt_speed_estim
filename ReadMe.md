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
		&altSpeedEstim_KF_s,	// Указатель на структуру
		(float) acc_Q_Noise,	// Шумовая характеристика акселерометра ()
		(float) baro_R_Noise,	// Шумовая характристика барометра
		(float) covar_P_Noise,	// Значение, которым будет проинициализирована 
								// диагональ матрицы ковариаций
		(float) alt,			// Начальное значение высоты
		(float) speed,			// Начальное значение векртикальной скорости
		(float) dT,				// Период интегрирования показаний акселерометра 
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
					KFASE_GetAltEstimate(
						&altSpeedEstim_KF_s);
			...
			/* Получить оценку вертикальной скорости */
			float verticalSpeedEstim = 
					KFASE_GetVerticalSpeedEstimate(
						&altSpeedEstim_KF_s);
			...
		}
	
	}