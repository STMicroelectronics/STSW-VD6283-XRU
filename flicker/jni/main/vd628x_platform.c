/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include "vd628x_platform.h"
#include "vd628x_adapter_ioctl.h"

#define LOG printf

#define MIN(a,b) ((a)<(b)?(a):(b))

// DEFAULT_SPI_FREQUENCY is the default SPI frequency, assumming that actual spi frequency MUST NOT be > DEFAULT_SPI_FREQUENCY Khz
// device tree of the vd6281 must provide spi_frequency value that must not be over this value
// But actual sp frequency can be lowered and linux does not provide a way to get this value.
// This driver therefore provides the LOCALLY_MEASURED_SPI_FREQUENCY option that calculates the SPI frequency
// flicker frequencies are calculated for spi frequency = DEFAULT_SPI_FREQUENCY Khz, and is then just adjusted with the locally measured frequency
#define DEFAULT_SPI_FREQUENCY           (4*1024*1024) // in Hz
// spi buffer size for 1 second data.
#define SPI_BUFFER_SIZE_1_SEC_DATA      (DEFAULT_SPI_FREQUENCY/8)  // corresponds to 5.24 Mhz.
// actual buffer size (this is for 1 second, actually)
#define SPI_BUFFER_SIZE	                (SPI_BUFFER_SIZE_1_SEC_DATA)

struct spi {
	int fd;
	uint32_t sampling_frequency;
	uint16_t pdm_data_sample_width_in_bytes;
	uint32_t chunk_size;
	uint8_t index; // aimed to collect 0.25 then 0.5 then 1 second of data
	uint32_t samples_number[3];
	uint16_t max_transfers[3];
	uint16_t samples_nb_per_chunk;
	//char raw[SPI_BUFFER_SIZE]; // SPI_BUFFER_SIZE must be a multiple of chunk_size
	//char * raw;
	int transfers_done;  // in chunk_size transfers
	pthread_mutex_t platform_mutex;
	uint32_t spi_max_frequency;
	uint32_t spi_speed_hz;
#ifdef LOCALLY_MEASURED_SPI_FREQUENCY
	struct timespec transfer_start_time;
	struct timespec transfer_end_time;
#endif
	uint16_t measured_spi_frequency;

};


//
// client
// structure of data storing the data related to i2c and spi buses
//
struct client {
	struct spi spi;
};


//
// spi_grab
// low level funtion responsible for capture raw data from spi bus in internally allocated buffer
//
int platform_chunck_transfer_and_get_samples(void *client, int16_t *samples)
{
	struct client *c = client;
	struct spi *spi = &c->spi;
	int ret;
	time_t dif_sec = 0;
	uint64_t dif_nsec = 0;

	if (spi->transfers_done == (spi->max_transfers[spi->index])) {
		// not supposed to happen with this implementation : see client flk_detect.c
		return -1;
	}

	ret = ioctl(spi->fd, VD628x_IOCTL_GET_CHUNK_SAMPLES, &samples[spi->transfers_done * spi->samples_nb_per_chunk]);
	if (ret)
		return -1;

#ifdef LOCALLY_MEASURED_SPI_FREQUENCY
	if (spi->transfers_done == 0) {
		// get time at the end of the first transfer
		clock_gettime(CLOCK_REALTIME, &spi->transfer_start_time);
		//LOG("Measured SPI frequency. Start = %lu\n", spi->transfer_start_time.tv_nsec);
	}
	else if (spi->transfers_done == (spi->max_transfers[spi->index]-1)) {
		// get time at the end of the last but one transfer
		clock_gettime(CLOCK_REALTIME, &spi->transfer_end_time);
		dif_sec = spi->transfer_end_time.tv_sec - spi->transfer_start_time.tv_sec;
		//LOG("Measured SPI frequency. dif_sec = %lu\n", dif_sec);
		//LOG("Measured SPI frequency. End = %lu\n", spi->transfer_end_time.tv_nsec);
		dif_nsec = (uint64_t)dif_sec*1000000000 + spi->transfer_end_time.tv_nsec - spi->transfer_start_time.tv_nsec;
		//LOG("Measured SPI frequency. dif_nsec = %lu\n", dif_nsec);
		spi->measured_spi_frequency = (uint16_t)(((uint64_t)spi->max_transfers[spi->index]-1)*(spi->chunk_size)*8*1000000 / dif_nsec);
		LOG("FLICKER : max, speed, measured : local, %d, %d, %d\n", spi->spi_max_frequency/1000, spi->spi_speed_hz/1000, spi->measured_spi_frequency);
	}
#else
	if (spi->transfers_done == (spi->max_transfers[spi->index]-1))
		LOG("FLICKER : max, speed, default : fixed, %d, %d, %d\n", spi->spi_max_frequency/1000, spi->spi_speed_hz/1000, spi->measured_spi_frequency);
#endif


	//if (spi->transfers_done == (spi->max_transfers[spi->index])) {
	if (spi->transfers_done == (spi->max_transfers[spi->index])-1) {
		return 1;
	}
	spi->transfers_done += 1;

	return 0;
}


//
// get_min_max_avg_remove_dc
// Function called when spi raw data buffer is filled up
// Counts the number of 1 within each PDM sample to generate sample for FFT
//
static void get_min_max_avg_remove_dc(void * client,
		int16_t * samples,
		uint16_t * pavgRawFlickerData,
		uint16_t * pmaxRawFlickerData,
		uint16_t * pminRawFlickerData)
{
	uint32_t s;
	uint32_t avg = 0;
	struct client *c = client;
	struct spi *spi = &c->spi;

#ifdef LOG_SAMPLES
	static uint32_t count = 0;
#endif
	*pmaxRawFlickerData = 0;
	*pminRawFlickerData = 0xFFFF;
	*pavgRawFlickerData = 0;

	// find the min, and max values of the samples
	// also calculate the sum for a further average calcultion
	for(s = 0; s < spi->samples_number[spi->index]; s++) {
		if (*samples > *pmaxRawFlickerData)
			*pmaxRawFlickerData = *samples;
		if (*samples < *pminRawFlickerData)
			*pminRawFlickerData = *samples;
		avg += *samples;
		samples++;
	}

	// calculate average
	avg /= spi->samples_number[spi->index];
	if (avg <= 0xFFFF)
		*pavgRawFlickerData = (uint16_t)(avg);

	// remove DC. this makes search of frequency peaks much easier once fft is performed
	samples -= spi->samples_number[spi->index];
	for(s = 0; s < spi->samples_number[spi->index]; s++) {
		*samples -= *pavgRawFlickerData;
#ifdef LOG_SAMPLES
		LOG("sample %d,%d,%d,%d\n", count, s, spi->samples_number[spi->index], *samples);
		count++;
#endif
		samples++;
	}
}


//
// STALS_lib_delay
// function waiting delay_ms to make utils being OS agnostic
//
void STALS_lib_delay(uint32_t delay_ms)
{
	usleep(delay_ms * 1000);
}


// platform_get_client
// function allocating structure to store coms bus related info : i2c and spi
void *platform_get_client()
{
	struct client *res;

	res = (struct client *) malloc(sizeof(struct client));
	if (!res)
		goto malloc_error;

	return (void *) res;

malloc_error:
	return NULL;
}

//
// platform_put_client
// function freeing structure to store coms bus related info : i2c and spi
//
void platform_put_client(void *client)
{
	struct client *c = client;

	if (!client)
		return;

	//close(c->f);
	free(c);
}

//
// platform_set_fft_info
// function providing sampling frequency to be applied on PDM data.
//
int platform_set_fft_info(void *client, uint32_t sampling_frequency) {
	struct client *c = client;
	struct spi *spi = &c->spi;
	struct vd628x_spi_params spi_params;
	int err;

	// lock
	pthread_mutex_lock(&spi->platform_mutex);

	spi->sampling_frequency = sampling_frequency;
	spi->pdm_data_sample_width_in_bytes = SPI_BUFFER_SIZE_1_SEC_DATA/sampling_frequency;

	// 3 values of samples_number to handle FFT on 0.25, then 0.5 then 1 second of data
	spi->samples_number[2] = SPI_BUFFER_SIZE/spi->pdm_data_sample_width_in_bytes;
	spi->samples_number[1] = spi->samples_number[2]/2;
	spi->samples_number[0] = spi->samples_number[1]/2;

	spi->samples_nb_per_chunk = sampling_frequency / (SPI_BUFFER_SIZE_1_SEC_DATA / spi->chunk_size);

	spi_params.speed_hz = spi->spi_speed_hz;
	spi_params.samples_nb_per_chunk = spi->samples_nb_per_chunk;
	spi_params.pdm_data_sample_width_in_bytes = spi->pdm_data_sample_width_in_bytes;
	err = ioctl(spi->fd, VD628x_IOCTL_SET_SPI_PARAMS, &spi_params);
	if (err) {
		LOG("FATAL error : error returned by VD628x_IOCTL_SET_SPI_PARAMS\n");
		return -1;
	}

	// platform_set_fft_info can be called dynamically because of client
	// is allowed to provide a new sampling frequency dynamically
	// set spi->index = 0 so that we restart flicker detect on 0.25, then 0.5 then 1s
	spi->index = 0;
	spi->transfers_done = 0;

	// unlock
	pthread_mutex_unlock(&spi->platform_mutex);

	LOG("FLICKER FFT INFO for 1 second of PDM data \n");
	LOG("        SPI buffer size : 0x%x\n", SPI_BUFFER_SIZE);
	LOG("        Sampling frequency in Hz : %d\n", sampling_frequency);
	LOG("        PDM Data Sample in bits : %d\n", spi->pdm_data_sample_width_in_bytes*8);
	LOG("        PDM Data Sample in Bytes : %d\n", spi->pdm_data_sample_width_in_bytes);
	LOG("        Samples Number : %d\n", spi->samples_number[2]);

	return 0;
}

//
// platform_spi_start
// function initalizing the data needed to start grabbing data from spi
//
int platform_spi_start(void *client, uint32_t sampling_frequency)
{
	struct client *c = client;
	struct spi *spi = &c->spi;
	struct vd628x_spi_info spi_info;
	int err;

	spi->fd = open("/dev/vd628x_spi", O_RDONLY);
	if (spi->fd < 0) {
		LOG("FATAL error : Could not open /dev/vd628x_spi\n");
		return -1;
	}

	// set read from device in blocking mode
	err = fcntl(spi->fd, F_SETFL, fcntl(spi->fd, F_GETFL, 0) &~O_NONBLOCK);
	if (err) {
		LOG("ERROR : Could not open /dev/vd6281_spi fe in read only \n");
		close(spi->fd);
		return -1;
	}

	err = ioctl(spi->fd, VD628x_IOCTL_GET_SPI_INFO, &spi_info);
	if (err) {
		LOG("FATAL error : error returned by VD628x_IOCTL_GET_SPI_INFO\n");
		close(spi->fd);
		return -1;
	}
	LOG("spi chunk size : %d\n", spi_info.chunk_size);

	if (spi_info.spi_max_frequency == 0) {
		LOG("Error. Got 0 for spi frequency\n");
		//free(spi->raw);
		close(spi->fd);
		return -1;
	}
	LOG("spi_max_frequency : %d\n", spi_info.spi_max_frequency);

	spi->spi_max_frequency = spi_info.spi_max_frequency;
	// if DEFAULT_SPI_FREQUENCY less that the max one, then
	// set the SPI bus with our preferred one.
	if (DEFAULT_SPI_FREQUENCY < spi_info.spi_max_frequency)
		spi->spi_speed_hz = DEFAULT_SPI_FREQUENCY;
	else
		spi->spi_speed_hz = spi_info.spi_max_frequency;

	// set SPI speed with the required one.
#ifndef LOCALLY_MEASURED_SPI_FREQUENCY
	// If spi speed not locally measured, it is assumed it is the one set to the SPI bus
	spi->measured_spi_frequency = spi->spi_speed_hz/1000;
#endif

	// check chuck size fits the basic requiements
	if (spi_info.chunk_size == 0) {
		LOG("Error. Got 0 for spi chunk size\n");
		close(spi->fd);
		return -1;
	}

	//spi->raw = (char *)malloc(spi_info.chunk_size);
	//if (spi->raw == NULL) {
	//	LOG("Error. Could not allocated buffer for PDM raw data\n");
	//	close(spi->fd);
	//	return -1;
	//}

	if ((SPI_BUFFER_SIZE < spi_info.chunk_size) || (SPI_BUFFER_SIZE % spi_info.chunk_size != 0)) {
		LOG("Error. chunk size not compatible with flicker detect requirements\n");
		//free(spi->raw);
		close(spi->fd);
		return -1;
	}

	// init and lock mutex
	pthread_mutex_init(&spi->platform_mutex, NULL);

	// init spi struct internal fields
	spi->chunk_size = spi_info.chunk_size;
	spi->max_transfers[2] = (uint16_t)(SPI_BUFFER_SIZE / spi_info.chunk_size);
	spi->max_transfers[1] = spi->max_transfers[2]/2;
	spi->max_transfers[0] = spi->max_transfers[1]/2;

	// init spi struct internal fields that may have to be updated dynamically
	// if top level client changes sampling frequency
	err = platform_set_fft_info(client, sampling_frequency);
	if (err) {
		//free(spi->raw);
		close(spi->fd);
		return -1;
	}

	//LOG("Flicker channel : platform spi started. spi chunk size = %d. max_transfers = %d\n", spi_info.chunk_size, spi->max_transfers[2]);
	return 0;
}


//
// platform_start_next_transfer
// function initalizing the data needed to start a new transfer.
// This is called after FFT on the previous raw data of the spi buffer is completed
//
int platform_start_next_transfer(void * client) {

	struct client *c = client;
	struct spi *spi = &c->spi;

	if (spi == NULL) {
		LOG("FATAL Error. spi = null\n");
		return -1;
	}
	// lock
	pthread_mutex_lock(&spi->platform_mutex);
	// transfers_done to 0 re-enables the transfer to spi buffer
	spi->transfers_done = 0;
	// unlock
	pthread_mutex_unlock(&spi->platform_mutex);

	return 0;
}

//
// platform_analyze_samples
// function generating the samples from the raw data of the spi buffer
// and returning info so that upper layer can performed FFT on the sampples
//
int platform_get_samples_stats(void *client,
		int16_t * data,
		uint16_t * pavgRawFlickerData,
		uint16_t * pmaxRawFlickerData,
		uint16_t * pminRawFlickerData,
		uint16_t * psamples_nb,
		uint16_t * pactualSpiFrequency,
		uint16_t * pdefaultSpiFrequency
		)
{
	struct client *c = client;
	struct spi *spi = &c->spi;

	// lock
	pthread_mutex_lock(&spi->platform_mutex);

	// if buffer not filled, exit
	if (spi->transfers_done < (spi->max_transfers[spi->index]-1)) {
		pthread_mutex_unlock(&spi->platform_mutex);
		return -1;
	}

	// if buffer filled, process
	get_min_max_avg_remove_dc(client, data, pavgRawFlickerData, pmaxRawFlickerData, pminRawFlickerData);

	// once process is completed. we can enable transfers again to spi buffer
	// lets cheat with the FFT so that we give data as if it was always 1 second of data
	// the 2 very first time, only 1/4 and 1/2 of the samples are real, the other are 0. This is the zero padding trick.
	// but in case of good signal we should get the right flicker frequency with 1Hz accuracy
	//*psamples_nb = spi->samples_number[spi->index];
	*psamples_nb = spi->samples_number[2]; // 2 instead of [spi->index] see comment above

	if (spi->index<2)
		spi->index++;

	// unlock
	pthread_mutex_unlock(&spi->platform_mutex);

	*pactualSpiFrequency = spi->measured_spi_frequency;
	*pdefaultSpiFrequency = DEFAULT_SPI_FREQUENCY/1000;

	return 0;
}


//
// platform_spi_stop
// freeing platform ressources needed for spi grab data
//
int platform_spi_stop(void *client)
{
	struct client *c = client;
	struct spi *spi = &c->spi;

	pthread_mutex_destroy(&spi->platform_mutex);
	//free(spi->raw);
	close(spi->fd);

	return 0;
}
