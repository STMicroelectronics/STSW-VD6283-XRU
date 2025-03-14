#ifndef __FFT_UTILS__
#define __FFT_UTILS__ 1

#include <stdint.h>
#include "fft.h"

void perform_fft(int16_t *flk, float complex *ffti, float complex *ffto, int nb, int is_dc_remove);
int find_flk_freq(int fe, float complex *ffto, int nb);
void find_flk_freq_with_info(int fe, float complex *ffto, int nb, int *freq, int *amplitude);

#endif
