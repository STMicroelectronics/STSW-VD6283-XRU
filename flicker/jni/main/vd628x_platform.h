/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#ifndef __VD628X_PLATFORM__
#define __VD628X_PLATFORM__ 1

#ifdef __cplusplus
extern "C" {
#endif

void *platform_get_client(/*int i2c_address_in_7_bits*/);
void platform_put_client(void *client);

int platform_spi_start(void *client, uint32_t sampling_frequency);
int platform_get_samples_stats(void *client,
			int16_t * samples,
			uint16_t * pavgRawFlickerData,
			uint16_t * pmaxRawFlickerData,
			uint16_t * pminRawFlickerData,
			uint16_t * psamples_nb,
			uint16_t * pactualSpiFrequency,
			uint16_t * pdefaultSpiFrequency
			);

int platform_chunck_transfer_and_get_samples(void * client, int16_t * samples);
int platform_spi_stop(void *client);
int platform_start_next_transfer(void *client);
int platform_set_fft_info(void *client, uint32_t sampling_frequency);
int platform_get_samples_nb(uint16_t * psamples_nb);

#ifdef __cplusplus
}
#endif

#endif
