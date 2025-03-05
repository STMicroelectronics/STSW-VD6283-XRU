/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#ifndef __FFT_UTILS__
#define __FFT_UTILS__ 1

#include <stdint.h>
#include "fft.h"

void perform_fft(int16_t *flk, float complex *ffti, float complex *ffto, int nb, int is_dc_remove);

void find_flk_freq_2(
		int fe,
		float complex *ffto,
		int nb,
		float * firstMaximaFrequency,
		float * firstMaximaAmplitude,
		float * SecondMaximaFrequency,
		float * SecondMaximaAmplitude,
		float * avgFiveHighestAmplitude);
#endif
