/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#include "vd628x_fft_utils.h"
#include <stdio.h>

#define LOG printf

void perform_fft(int16_t *flk, float complex *ffti, float complex *ffto, int nb, int is_dc_remove)
{
	int i;
	float dc = 0;

	if (is_dc_remove) {
		for(i = 0; i < nb; i++)
			dc += flk[i];
		dc = dc / nb;
	}

	for(i = 0; i < nb; i++)
		ffti[i] = flk[i] - dc;

	fft(ffti, ffto, nb);
}

void find_flk_freq_2(
		int fe,
		float complex *ffto,
		int nb,
		float * firstMaximaPeakFrequency,
		float * firstMaximaPeakAmplitude,
		float * SecondMaximaPeakFrequency,
		float * SecondMaximaPeakAmplitude,
		float * avgFiveHighestAmplitude)
{
#ifdef LOG_FFT
	static uint32_t count=0;
#endif
	int index_max[5] = {0,0,0,0,0};
	float max_value[5] = {-1,-1,-1,-1,-1};

	int i;

	for(i=1; i < nb / 2; i++) {
		if (cabsf(ffto[i]) > max_value[0]) {
			index_max[0] = i;
			max_value[0] = cabsf(ffto[i]);
		}
	}
	for(i=1; i < nb / 2; i++) {
		if ((cabsf(ffto[i]) > max_value[1]) &&
				(cabsf(ffto[i]) != max_value[0])) {
			index_max[1] = i;
			max_value[1] = cabsf(ffto[i]);
		}
	}
	for(i=1; i < nb / 2; i++) {
		if ((cabsf(ffto[i]) > max_value[2]) &&
				(cabsf(ffto[i]) != max_value[0]) &&
				(cabsf(ffto[i]) != max_value[1])
				) {
			index_max[2] = i;
			max_value[2] = cabsf(ffto[i]);
		}
	}
	for(i=1; i < nb / 2; i++) {
		if ((cabsf(ffto[i]) > max_value[3]) &&
				(cabsf(ffto[i]) != max_value[0]) &&
				(cabsf(ffto[i]) != max_value[1]) &&
				(cabsf(ffto[i]) != max_value[2])
				) {
			index_max[3] = i;
			max_value[3] = cabsf(ffto[i]);
		}
	}
	for(i=1; i < nb / 2; i++) {
		if ((cabsf(ffto[i]) > max_value[4]) &&
				(cabsf(ffto[i]) != max_value[0]) &&
				(cabsf(ffto[i]) != max_value[1]) &&
				(cabsf(ffto[i]) != max_value[2]) &&
				(cabsf(ffto[i]) != max_value[3])
				) {
			index_max[4] = i;
			max_value[4] = cabsf(ffto[i]);
		}
	}

#ifdef LOG_FFT
	for (i=1; i < nb / 2; i++) {
		LOG("fft,%d,%d,%d,%f\n",count,fe,i*fe/nb, cabsf(ffto[i]/nb));
	}
	count++;
#endif
	*firstMaximaPeakFrequency = (index_max[0] * fe) / nb;
	*firstMaximaPeakAmplitude = max_value[0]/nb;
	*SecondMaximaPeakFrequency = (index_max[1] * fe) / nb;
	*SecondMaximaPeakAmplitude = max_value[1]/nb;
	*avgFiveHighestAmplitude =   (*firstMaximaPeakAmplitude + *SecondMaximaPeakAmplitude + max_value[2]/nb + max_value[3]/nb +max_value[4]/nb)/5;
}

