/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <inttypes.h>
#include <complex.h>
#include <pthread.h>

#include "vd628x_platform.h"
#include "vd628x_fft_utils.h"

#include "vd628x_flk_detect.h"

#define UNUSED(p)  ((void)(p))

#define LOG printf

//
// vd628x_flk_detect_info
// structure for information
//
struct vd628x_flk_detect_info {
	// flicker detect thread
	pthread_t flicker_detect_thread;
	uint8_t flicker_detect_runs;
	// buffers for flicker detect
	int samplingFrequency;
	int newSamplingFrequency;
	int16_t * flk_data;
	float complex * fft_in;
	float complex * fft_out;
	// platform client
	void * client;
	// STALS handle
	//void * handle;
	// primary channel
	//enum STALS_Channel_Id_t primaryChannelId;
	// ficker and fft results
	struct vd628x_flk_detect_fftResults fftResults;
	int (*send_fftResults)(void *);
};


static struct vd628x_flk_detect_info * pFLKDI = NULL;


//
// allocate_fft_resources
// allocation of resources needed for fft to be performed
//
static int allocate_fft_resources() {

	// malloc data for fft and reset to 0 all samples so that 1Hz accuracy
	//can be worked with less that 1 second of samples
	pFLKDI->flk_data = (int16_t *)calloc(pFLKDI->samplingFrequency, sizeof(int16_t));
	if (pFLKDI->flk_data == NULL) {
		return -1;
	}

	pFLKDI->fft_in = (float complex *)malloc(pFLKDI->samplingFrequency*sizeof(float complex));
	if (pFLKDI->fft_in == NULL) {
		free(pFLKDI->flk_data);
		return -1;
	}
	pFLKDI->fft_out = (float complex *)malloc(pFLKDI->samplingFrequency*sizeof(float complex));
	if (pFLKDI->fft_out == NULL) {
		free(pFLKDI->fft_in);
		free(pFLKDI->flk_data);
		return -1;
	}

	return 0;
}

//
// free_fft_resources
// disallocation of resources needed for fft to be performed
//
static void free_fft_resources() {

	free(pFLKDI->flk_data);
	free(pFLKDI->fft_in);
	free(pFLKDI->fft_out);
}


//
// flicker_detect_routine
// routing executing the flicker detect thread
// when flicker_detect_routine is being started, platform_spi_started has already been started
//
static void *flicker_detect_routine(void * dummy)
{
	int err;
	uint16_t default_spi_frequency, actual_spi_frequency;
	uint16_t samples_nb;

	UNUSED(dummy);

	// error if not opened
	if (pFLKDI == NULL)
		return NULL;

	// launch auto gain search

	//err = STALS_LIB_flk_autogain(pFLKDI->handle, pFLKDI->primaryChannelId, 35, &pFLKDI->fftResults.flickerChannelGain);
	//if (err != STALS_NO_ERROR) {
	//	LOG("Start flicker channel auto gain failed\n");
	//	return NULL;
	//}
	//LOG("Channel %d gain set to 0x%04x\n", pFLKDI->primaryChannelId, pFLKDI->fftResults.flickerChannelGain);

	// start flicker
	//LOG("Calling STALS_Start(mode_flicker) ...\n");
	//err = STALS_Start(pFLKDI->handle, STALS_MODE_FLICKER, pFLKDI->primaryChannelId);
	//if (err != STALS_NO_ERROR) {
	//	LOG("Start flicker channel failed\n");
	//	return NULL;
	//}

	while (pFLKDI->flicker_detect_runs) {

		// platform_get_samples returns
		// -1 in case of error
		// 0 is spi buffer transfer on going
		// 1 if transfer completed
		err = platform_chunck_transfer_and_get_samples(pFLKDI->client, pFLKDI->flk_data);
		if (err < 0 ) {
			LOG("FATAL error : spi_grab failed !\n");
			goto stop_and_exit;
		}
		else if (err == 1) {

			// samples_nb is proportionnal to the time over which the data are captured
			// and is sampling frequency only if fft runs on 1 second of data
			err = platform_get_samples_stats(pFLKDI->client,
				pFLKDI->flk_data,
				&pFLKDI->fftResults.avgRawFlickerData,
				&pFLKDI->fftResults.maxRawFlickerData,
				&pFLKDI->fftResults.minRawFlickerData,
				&samples_nb,
				&actual_spi_frequency,
				&default_spi_frequency
				);

			if (err) {
				LOG("FATAL error : spi_grab failed !\n");
				goto stop_and_exit;
			}
			else {
				// stop flicker
				//err = STALS_Stop(pFLKDI->handle, STALS_MODE_FLICKER);
				//if (err != STALS_NO_ERROR) {
				//	LOG("Stop flicker channel failed\n");
				//	return NULL;
				//}

				//LOG("Flicker channel : Start FFT on processed data\n");
				perform_fft(pFLKDI->flk_data, pFLKDI->fft_in, pFLKDI->fft_out, samples_nb, 0);
				//LOG("Flicker channel : FFT completed\n");

				find_flk_freq_2(pFLKDI->samplingFrequency,
					pFLKDI->fft_out,
					samples_nb,
					&pFLKDI->fftResults.firstMaximaPeakFrequency,
					&pFLKDI->fftResults.firstMaximaPeakAmplitude,
					&pFLKDI->fftResults.secondMaximaPeakFrequency,
					&pFLKDI->fftResults.secondMaximaPeakAmplitude,
					&pFLKDI->fftResults.avgFlickerFreqAmplitude);

				//LOG("Flicker channel : found frequency peaks\n");
				pFLKDI->fftResults.firstMaximaPeakFrequency *= ((float)actual_spi_frequency/default_spi_frequency);
				pFLKDI->fftResults.secondMaximaPeakFrequency *= ((float)actual_spi_frequency/default_spi_frequency);
				pFLKDI->fftResults.configuredSamplingFlickerFreq = pFLKDI->samplingFrequency;
				pFLKDI->send_fftResults((void *)(&pFLKDI->fftResults));

				// check if new samling frequency has been dynmically provided
				if ((pFLKDI->newSamplingFrequency != 0) && (pFLKDI->newSamplingFrequency != pFLKDI->samplingFrequency)) {
					free_fft_resources();
					pFLKDI->samplingFrequency = pFLKDI->newSamplingFrequency;
					err = allocate_fft_resources();
					if (err) {
						LOG("New Sampling frequency : Error. Can not allocate resources\n");
						return NULL;
					}
					platform_set_fft_info(pFLKDI->client, pFLKDI->samplingFrequency);
				}

				// launch auto gain search
				//err = STALS_LIB_flk_autogain(pFLKDI->handle, pFLKDI->primaryChannelId, 35, &pFLKDI->fftResults.flickerChannelGain);
				//if (err != STALS_NO_ERROR) {
				//	LOG("Start flicker channel auto gain failed\n");
				//	return NULL;
				//}
				//LOG("Channel %d gain set to 0x%04x\n", pFLKDI->primaryChannelId, pFLKDI->fftResults.flickerChannelGain);

				// start flicker
				//LOG("Calling STALS_Start(mode_flicker) ...\n");
				//err = STALS_Start(pFLKDI->handle, STALS_MODE_FLICKER, pFLKDI->primaryChannelId);
				//if (err != STALS_NO_ERROR) {
				//	LOG("Start flicker channel failed\n");
				//	return NULL;
				//}

				// fft completed. enable data collection for next transfer
				platform_start_next_transfer(pFLKDI->client);
			}
		}
	}

stop_and_exit:
	// stop flicker
	//err = STALS_Stop(pFLKDI->handle, STALS_MODE_FLICKER);
	//if (err != STALS_NO_ERROR) {
	//	LOG("Stop flicker channel failed\n");
	//	return NULL;
	//}

	return NULL;
}



//
// vd628x_flickerDetectStart
// allocation of resources necessary to run FFT on clear channel raw data
// and starts internal thread responsible for capturing data from spi and performing FFT
//
int vd628x_flickerDetectStart(void * client, /*void * handle, enum STALS_Channel_Id_t primaryChannelId,*/ uint32_t samplingFrequency, int (* send_fftResults)(void * fftResults)) {

	int err;

	if (pFLKDI != NULL) {
		LOG("pFLKDI != NULL. flicker thread already started ?\n");
		return -1;
	}

	if (send_fftResults == NULL) {
		LOG("FATAL : send_fftResults = NULL\n");
		return -1;
	}

	pFLKDI = (struct vd628x_flk_detect_info *)malloc(sizeof(struct vd628x_flk_detect_info));
	if (pFLKDI == NULL) {
		LOG("malloc pFLKDI failed\n");
		return -1;
	}

	// init structure
	memset(pFLKDI, 0, sizeof(struct vd628x_flk_detect_info));

	// store info from client locally
	pFLKDI->client = client;
	//pFLKDI->handle = handle;
	//pFLKDI->primaryChannelId = primaryChannelId;
	pFLKDI->samplingFrequency = samplingFrequency;

	// allocated resources needed for fft to run
	err = allocate_fft_resources();
	if (err) {
		free(pFLKDI);
		return -1;
	}

	// platform_spi_start opens /dev/vd628x_spi and starts a thread that capture spi data
	err = platform_spi_start(pFLKDI->client, samplingFrequency);
	if (err != 0) {
		LOG("ERROR : Error in starting spi capture\n");
		free_fft_resources();
		free(pFLKDI);
		return -1;
	}
	LOG("capture from spi started.\n");

	// start a thread that gets the ALS values and the spi buffers to run FFT on
	// flicker detect thread
	pFLKDI->flicker_detect_runs = 1;
	pFLKDI->send_fftResults = send_fftResults;
	err = pthread_create(&pFLKDI->flicker_detect_thread, NULL, flicker_detect_routine, NULL);
	if (err) {
		LOG("flicker thread create failed\n");
		pFLKDI->flicker_detect_runs = 0;
		free_fft_resources();
		free(pFLKDI);
		return -1;
	}
	LOG("flicker thread created.\n");
	LOG("STALS_Start(mode_flicker) done.\n");

	return 0;
}


//
// vd628x_flickerDetectNewSamplingFrequency
// Function aimed to support dynamic update of sampling frequency from application
//
int vd628x_flickerDetectNewSamplingFrequency(uint16_t samplingFrequency) {
	// a new frequency is provided by the client
	// lets abort current calculation and reset data
	if (pFLKDI == NULL) {
		LOG("FATAL error pFLKDI == NULL\n");
		return -1;
	}

	pFLKDI->newSamplingFrequency = samplingFrequency;

	return 0;
}

//
// vd628x_flickerDetectStop
// Stop of grab of raw data from SPI, stop and deletion internal thread.
//
int vd628x_flickerDetectStop() {
	void *retval;

	// ensure the ending of the thread
	// to be added : mutex to protect flicker_detect_runs
	pFLKDI->flicker_detect_runs = 0;

	// wait for the thread completion
	pthread_join(pFLKDI->flicker_detect_thread, &retval);

	// stop platform
	platform_spi_stop(pFLKDI->client);

	// free resources
	free_fft_resources();
	free(pFLKDI);
	pFLKDI = NULL;

	return 0;
}

