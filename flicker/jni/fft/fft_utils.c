#include "fft_utils.h"
#include <stdio.h>

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

void find_flk_freq_with_info(int fe, float complex *ffto, int nb, int *freq, int *amplitude)
{
	int index_max = 0;
	float max_value = -1;
	int i;
	int res;

	for(i = 0; i < nb / 2; i++) {
		if (cabsf(ffto[i]) > max_value) {
			index_max = i;
			max_value = cabsf(ffto[i]);
		}
	}
	res = (index_max * fe) / nb;

	if (freq)
		*freq = res;
	if (amplitude)
		*amplitude = max_value / nb;
}

int find_flk_freq(int fe, float complex *ffto, int nb)
{
	int freq;

	find_flk_freq_with_info(fe, ffto, nb, &freq, NULL);

	return freq;
}

