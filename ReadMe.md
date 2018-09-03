Lib_A_KFASE_alt_speed_estim

`KFASE` - Kalman filter altitude speed estimate

Программный модуль, реализующий оценку вертикальной скорости и высоты по показаниям акселерометра (вдоль вертикальной оси) и барометрического высотомера с помощью фильтра Калмана

Как работать с библиотекой:

Прежде всего необходимо вызвать функцию `KFASE_Init_KF`


	...
	/* Объявление структуры для работы с программным 
	 * модулем фильтра Калмана */
	kfase_alt_speed_estimate_s altSpeedEstim_KF_s;
	...
	
	int main (void)
	{
	...
	KFASE_Init_KF
	(
		&altSpeedEstim_KF_s,	// Указатель на структуру
		acc_Q_Noise,			// Шумовая характеристика акселерометра ()
		baro_R_Noise,			// Шумовая характристика барометра
		covar_P_Noise,			// Значение, которым будет проинициализирована 
								// диоганаль матрицы ковариаций
		alt,					// Начальное значение высоты
		speed,					// Начальное значение векртикальной скорости
		dT,						// Период интегрирования показаний акселерометра 
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
			if (accMeasureReady_flag == 1 && baroMesureReady_flag == 1)
			{
				KFASE_GetPredictWithCorrect(
					&altSpeedEstim_KF_s, 
					verticalAcceleration, 
					baroAltMeasure);
			}
			...ё
		}
	
	}