/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#ifndef __VD628X_FLK_DETECT__
#define __VD628X_FLK_DETECT__

#ifdef __cplusplus
extern "C" {
#endif

struct vd628x_flk_detect_fftResults {
	float firstMaximaPeakFrequency;
	float firstMaximaPeakAmplitude;
	float secondMaximaPeakFrequency;
	float secondMaximaPeakAmplitude;
	float avgFlickerFreqAmplitude;
	uint16_t avgRawFlickerData;
	uint16_t maxRawFlickerData;
	uint16_t minRawFlickerData;
	uint16_t flickerChannelGain;
	uint16_t configuredSamplingFlickerFreq;
};

int vd628x_flickerDetectStart(void * client, /*void * handle, enum STALS_Channel_Id_t primaryChannelId, */uint32_t samplingFrequency, int (* send_fftResults)(void * fftResults));
int vd628x_flickerDetectNewSamplingFrequency(uint16_t samplingFrequency);
int vd628x_flickerDetectStop();

#ifdef __cplusplus
}
#endif

#endif
