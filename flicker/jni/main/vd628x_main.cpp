/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <inttypes.h>
#include <complex.h>
#include <pthread.h>

#include "vd628x_interface.h"

#include "vd628x_platform.h"
#include "vd628x_flk_detect.h"

#define UNUSED(p)  ((void)(p))

#define VENDOR "STMicroelectronics"
#define DRIVER_VERSION 23

#ifdef VD6282
#define DEVICE_ID "VD6282"
#define HARDWARE_VERSION "Cut2.2"
#else
#ifdef VD6283
#define DEVICE_ID "VD6283"
#define HARDWARE_VERSION "Cut2.3"
#endif
#endif


#define STARTED 1
#define STOPPED 0

#define MIN_EXPOSURE_TIME_IN_US 2000    // = 2 ms
#define MAX_EXPOSURE_TIME_IN_US 1600000 // = 1.6 s

#define CALCULATION_TIME_IN_US 6000

#define MIN_TIMING_BUDGET_IN_US (MIN_EXPOSURE_TIME_IN_US + CALCULATION_TIME_IN_US)
#define DEFAULT_TIMING_BUDGET_IN_US (10000 + CALCULATION_TIME_IN_US)
#define MAX_TIMING_BUDGET_IN_US (MAX_EXPOSURE_TIME_IN_US + CALCULATION_TIME_IN_US)

#define MIN_TIMING_BUDGET_IN_MS (MIN_TIMING_BUDGET_IN_US/1000)
#define MAX_TIMING_BUDGET_IN_MS (MAX_TIMING_BUDGET_IN_US/1000)

#define GAIN 0x0A00

#define MAX_DATA_MULTI_SPECTRAL_SENSOR 5

#define LOG printf

#define DEFAULT_SAMPLING_FREQUENCY_INDEX 1 // so that 2048 is the value by default
static uint16_t sampling_frequencies[] = {4096, 2048, 1024, 512};  // decreasing order must not be conserved
static uint8_t sampling_frequencies_table_size = sizeof(sampling_frequencies)/sizeof(sampling_frequencies[0]);


//
// commands identifiers implementing asynchronousity of the functions
// start, stop and close.
//
enum vd628x_Command {
	commandNone = 0,
	commandStart = 2,
	commandStop = 3,
	commandClose = 4
};

//
// vd628x_Info
// structure for information
//
struct vd628x_Info {
	// client
	void * client;
	// state of the vd628x camx driver : STARTED or STOPPED
	uint8_t state;
	// info about channels
	uint32_t samplingFrequency;
	// Main Data Structure that contains Spectral Sensor Data
	int8_t dataMultiSpectralSensorAlsInfoIndex;
	int8_t dataMultiSpectralSensorFlickerInfoIndex;
	struct NCSDataMultiSpectralSensor dataMultiSpectralSensor[MAX_DATA_MULTI_SPECTRAL_SENSOR];
	// thread to make start and stop blocking
	uint8_t mainThreadRuns;
	uint8_t mainThreadStarted;
	enum vd628x_Command pendingCommand;
	pthread_t mainThread;
	// api mutex
	pthread_mutex_t mutexApi;
	// mutexes to protect data
	pthread_mutex_t mutexAls;
	pthread_mutex_t mutexFlicker;
	// mutex and condition to deblock pollSensorData
	pthread_cond_t conditionToUnblockPoll;
	pthread_mutex_t conditionToUnblockPollMutex;
	// mutex and condition to deblock main threadpoll
	pthread_cond_t conditionToUnblockMainThread;
	pthread_mutex_t conditionToUnblockMainThreadMutex;
};

static struct vd628x_Info * pVCI = NULL;

//
// vd628x driver information
// XYZ means X.Y.Z and shall be increased in case any any update in platform, bare driver or adaptation layer adaptation
//
static const struct DriverInformation &vd628x_driverInformation {
	DEVICE_ID,
	VENDOR,
	HARDWARE_VERSION,
	DRIVER_VERSION
};

//
// Attributes array for vd628x devices
//
static struct Attribute vd628x_attributes[] =
{
	{
		"exposureTime",
		{
			MIN_TIMING_BUDGET_IN_US,
			MAX_TIMING_BUDGET_IN_US
		},
	},
	{
		"samplingFrequency",
		{
			(float)sampling_frequencies[sampling_frequencies_table_size-1],
			(float)sampling_frequencies[0]
		}
	}
};

//
// Sensor Attribute structure
//
static struct SensorAttribute vd628x_sensorAttribute =
{
	(sizeof(vd628x_attributes)/sizeof(struct Attribute)),
	vd628x_attributes
};


//
// fftResults_callback
// callback called at each new flicker frequency is calculated
//
static int fftResults_callback(void * fftResults)
{
	struct vd628x_flk_detect_fftResults * pFFTR = (struct vd628x_flk_detect_fftResults *)fftResults;
	struct SpectralFlickerFrequencyInfo * pflickerInfo;

	// error if not opened
	if ((pVCI == NULL) || (pFFTR == NULL))
		return -1;

	// mutex lock
	pthread_mutex_lock(&pVCI->mutexFlicker);

	pVCI->dataMultiSpectralSensorFlickerInfoIndex++;
	if (pVCI->dataMultiSpectralSensorFlickerInfoIndex == MAX_DATA_MULTI_SPECTRAL_SENSOR)
		pVCI->dataMultiSpectralSensorFlickerInfoIndex = 0;

	//LOG("--------------- FLICKER DETECT : fft results call back called\n");
	pflickerInfo = &pVCI->dataMultiSpectralSensor[pVCI->dataMultiSpectralSensorFlickerInfoIndex].flickerInfo;
	pflickerInfo->isValid = TRUE;
	pflickerInfo->firstMaximaPeak.frequency = pFFTR->firstMaximaPeakFrequency;
	pflickerInfo->firstMaximaPeak.amplitude = pFFTR->firstMaximaPeakAmplitude;
	pflickerInfo->secondMaximaPeak.frequency = pFFTR->secondMaximaPeakFrequency;
	pflickerInfo->secondMaximaPeak.amplitude = pFFTR->secondMaximaPeakAmplitude;
	pflickerInfo->avgFlickerFreqAmplitude = pFFTR->avgFlickerFreqAmplitude;

	pflickerInfo->avgRawFlickerData = pFFTR->avgRawFlickerData;
	pflickerInfo->maxRawFlickerData = pFFTR->maxRawFlickerData;
	pflickerInfo->minRawFlickerData = pFFTR->minRawFlickerData;

	pflickerInfo->expGainOfFlickerChannel = pFFTR->flickerChannelGain;
	pflickerInfo->configuredSamplingFlickerFreq =  pFFTR->configuredSamplingFlickerFreq;

	// mutex unlock
	pthread_mutex_unlock(&pVCI->mutexFlicker);

	// signal to unlock poll that can be possibly waiting
	pthread_cond_signal(&pVCI->conditionToUnblockPoll);

	return 0;
}


//
// Start
// Static function starting flicker and ALS
// Called asynchronoulsy, immediately after StartSensor is called by application
//
static int Start() {

	int err;

	LOG("Starting FLICKER .... \n");
	// start a thread that captures spi buffers to run FFT on
	err = vd628x_flickerDetectStart(pVCI->client, pVCI->samplingFrequency, fftResults_callback);
	if (err) {
		LOG("Start failed. vd628x_flickerDetectStart failed\n");
		return -1;
	}

	LOG("FLICKER started\n");
	pVCI->state = STARTED;
	return 0;
}



//
// Stop
// Static function called by main thread to process a stop command
// Called asynchronoulsy, immediately after SopSensor is called by application
//
static int Stop() {
	int err = 0;

	LOG("Stopping FLICKER .... \n");
	// stop flicker detection thread. vd628x_flickerDetectStop is blocking and waits nice ending of the flicker detection thread
	err = vd628x_flickerDetectStop();
	if (err != 0) {
		LOG("Stop failed. Error in stopping flicker detection thread\n");
		return -1;
	}
	LOG("FLICKER Stopped\n");

	pVCI->state = STOPPED;

	// signal to unlock poll that can be possibly waiting ==> Agreed to be un
	// ==> Agreed to be useless is client is commited to stop Polling when calling stop
	// pthread_cond_signal(&pVCI->conditionToUnblockPoll);

	LOG("Stop ALS Device OK\n");
	return 0;
}


//
// main thread, waiting signals sent by the StartSensor, StopSensor, and CloseSensor
// and performing effective Start, Stop and Close asynchronoulsy
//
static void *mainRoutine(void * dummy) {

	struct timespec timeout;
	int err;

	UNUSED(dummy);

	do {
		pthread_mutex_lock(&pVCI->mutexApi);
		pVCI->mainThreadStarted = 1;
		pthread_mutex_unlock(&pVCI->mutexApi);

		// wait for condition
		pthread_mutex_lock(&pVCI->conditionToUnblockMainThreadMutex);
	        if (clock_gettime(CLOCK_REALTIME, &timeout) != 0) {
	                LOG("Warning : Can't get system time");
	        }
	        timeout.tv_sec += 1;
		//pthread_cond_wait(&pVCI->conditionToUnblockMainThread, &pVCI->conditionToUnblockMainThreadMutex);
		pthread_cond_timedwait(&pVCI->conditionToUnblockMainThread, &pVCI->conditionToUnblockMainThreadMutex, &timeout);
		pthread_mutex_unlock(&pVCI->conditionToUnblockMainThreadMutex);

		//  start
		pthread_mutex_lock(&pVCI->mutexApi);
		if (pVCI->pendingCommand == commandNone) {
			// can happen since conditionToUnblockMainThread expires after 1 second time out
			// in order to manage consecutive calls of API functions sending commands
			// Example stop just after start, or close just after stop
		}
		else if (pVCI->pendingCommand == commandStart) {
			LOG("Processing Start Command\n");
			err = Start();
			if (err)
				LOG("Error in Starting als sensor\n");

		}
		else if (pVCI->pendingCommand == commandStop) {
			LOG("Processing Stop Command\n");
			err = Stop();
			if (err)
				LOG("Error in Stopping als sensor\n");
		}
		else if (pVCI->pendingCommand == commandClose) {
			LOG("Finishing main thread\n");
			// command to finish main thread, posted by CloseSensor
			pVCI->mainThreadRuns = 0;
		}

		pVCI->pendingCommand = commandNone;
		pthread_mutex_unlock(&pVCI->mutexApi);

	} while (pVCI->mainThreadRuns);

	return NULL;
}

//
// QuerySensorInfo
// query sensor's characteristics
// Can be called whatever the device is opened or closed, whatever the availibility of the sensor
//
static void QuerySensorInfo(QueryInfo* pQuery) {

	if (pQuery->queryType == DriverInfo) {
		pQuery->pData = (void *)&vd628x_driverInformation;
		pQuery->size = sizeof(struct DriverInformation);
	}
	else if (pQuery->queryType == SensorAttributes) {
		pQuery->pData = (void *)&vd628x_sensorAttribute;
		pQuery->size = sizeof(vd628x_attributes);
	}
}



//
// Configure
// Send Configurations to be applied
//
static int Configure(const ConfigureParameters* pConfig, const unsigned int numOfConfigParams) {

	uint8_t i, j;
	const ConfigureParameters* pC = pConfig;

	// error if not opened
	if (pVCI == NULL) {
		LOG("Configure sensor failed. Sensor not opened\n");
		return -1;
	}

	// check input params
	if (pConfig == NULL) {
		LOG("Configure sensor failed. Wrong input params\n");
		goto fail;
	}

	// protect concurrent api calls
	pthread_mutex_lock(&pVCI->mutexApi);

	// error if state is STARTED
	if (pC->configType != SamplingFrequency) { // Client requests to have bew SamplingFrequency supported dynamically
		if (pVCI->state != STOPPED)  {
			LOG("Configure sensor failed. Sensor already started\n");
			goto fail;
		}
	}

	for (j=0; j<numOfConfigParams; j++) {
		if (pC->configType == SamplingFrequency) {
			if ((pC->configPayload.samplingFrequency <= sampling_frequencies[0]) &&
				(pC->configPayload.samplingFrequency >= sampling_frequencies[sampling_frequencies_table_size-1]))
			{
				for (i=(sampling_frequencies_table_size-1); i >= 0; i--) {
					if (pC->configPayload.samplingFrequency <= sampling_frequencies[i]) {
						pVCI->samplingFrequency = sampling_frequencies[i];
						LOG("SensorConfigure samplingFrequency = %d. Configured sampling frequency = %d\n", pC->configPayload.samplingFrequency, pVCI->samplingFrequency);
						if (pVCI->state == STARTED)
							vd628x_flickerDetectNewSamplingFrequency(pVCI->samplingFrequency);
						goto success;
					}
				}
			}
			LOG("SensorConfigure failed. Sampling frequency is out of supported range\n");
			goto fail;
		}
		else {
			LOG("SensorConfigure failed. Wrong input params\n");
			goto fail;
		}

		pC++;
	}

success:
	LOG("Configure ALS Device OK\n");
	pthread_mutex_unlock(&pVCI->mutexApi);
	return 0;

fail:
	LOG("Error in Configuring ALS Device\n");
	pthread_mutex_unlock(&pVCI->mutexApi);
	return -1;

}

//
// OpenSensor
// Opens a link to the spectral sensor if it is present.
// SpectralSensorInterface operations can be performed if 1 returned
//
static int OpenSensor() {

	int err = 0;
	uint8_t i = 0;

	// ckeck if already opened
	if (pVCI != NULL) {
		LOG("OpenSensor failed. sensor already opened\n");
		return -1;
	}

	// look if /dev/vd628x_spi can be opened, if not sensor is not here
	int dev_vd628x_spi;
	dev_vd628x_spi = open("/dev/vd628x_spi", O_RDWR);
	if (dev_vd628x_spi < 0) {
		// sensor not here
		LOG("OpenSensor failed. open /dev/vd628x_spi failed\n");
		return -2;
	}
	close(dev_vd628x_spi);

	// allocate internal structure info
	pVCI = new(struct vd628x_Info);
	if (pVCI == NULL) {
		LOG("OpenSensor failed. Can not allocate ressources\n");
		return  -1;
	}

	// init all fields to 0
	memset(pVCI, 0, sizeof(struct vd628x_Info));

	// init fields of the struct info with default values
	pVCI->client = NULL;
	pVCI->state = STOPPED;
	pVCI->samplingFrequency = sampling_frequencies[DEFAULT_SAMPLING_FREQUENCY_INDEX];

	// reset all data to be polled
	pVCI->dataMultiSpectralSensorFlickerInfoIndex = -1; // -1 this means that the table is empty.
	pVCI->dataMultiSpectralSensorAlsInfoIndex = -1;

	// for now on Clear channel is supported
	for (i=0 ; i<MAX_DATA_MULTI_SPECTRAL_SENSOR; i++)
		pVCI->dataMultiSpectralSensor[i].flickerInfo.channel = ClearChannel1;

	// init STALS
	pVCI->client = platform_get_client();


	// init the mutexes
	pthread_mutex_init(&pVCI->mutexApi, NULL);
	pthread_mutex_init(&pVCI->mutexAls, NULL);
	pthread_mutex_init(&pVCI->mutexFlicker, NULL);

	// init mutex and conditions permitting to have poll blocking
	pthread_cond_init(&pVCI->conditionToUnblockPoll, NULL);
	pthread_mutex_init(&pVCI->conditionToUnblockPollMutex, NULL);
	pthread_cond_init(&pVCI->conditionToUnblockMainThread, NULL);
	pthread_mutex_init(&pVCI->conditionToUnblockMainThreadMutex, NULL);

	// start main thread
	pVCI->mainThreadRuns = 1;
	err = pthread_create(&pVCI->mainThread, NULL, mainRoutine, NULL);
	if (err) {
		LOG("camx main thread create failed\n");
		pthread_mutex_destroy(&pVCI->conditionToUnblockPollMutex);
		pthread_cond_destroy(&pVCI->conditionToUnblockPoll);
		pthread_mutex_destroy(&pVCI->conditionToUnblockMainThreadMutex);
		pthread_cond_destroy(&pVCI->conditionToUnblockMainThread);
		pthread_mutex_destroy(&pVCI->mutexAls);
		pthread_mutex_destroy(&pVCI->mutexFlicker);
		pthread_mutex_destroy(&pVCI->mutexApi);
		free(pVCI);
		pVCI = NULL;
		return -1;
	}

	// lets wait that main thread is active before exiting.
	// othewise an immediate call to StartSensor from the client may end in having
	// a signal send but not caught bu the main thread since not created yet
	while (1) {
		// wait for api mutex
		pthread_mutex_lock(&pVCI->mutexApi);

		if (!pVCI->mainThreadStarted) {
			pthread_mutex_unlock(&pVCI->mutexApi);
			usleep(1000);
		}
		else
			break;
	}
	pthread_mutex_unlock(&pVCI->mutexApi);

	LOG("Open ALS Device OK\n");
	return 0;
}

//
// PollSensorData
// Function blocking until new data is available. even if device is not started yet
// Client MUST not call this any more after calling Stop, otherwise PollSensorData
// may be waiting for a signal that would happen when the driver is re-started
//
static int PollSensorData(const uint8_t numSamples, void * pSensorData) {

	int8_t i;
	uint8_t k, actualNumSamples;
	struct NCSDataMultiSpectralSensor * pSD;
	struct timespec timeout;

	// error if not opened or not started
	if (pVCI == NULL) {
		LOG("PollSensorData failed. Device not opened.\n");
		return -1;
	}

	// can't provide more data than the max we can do
	if (numSamples > MAX_DATA_MULTI_SPECTRAL_SENSOR)
		actualNumSamples = MAX_DATA_MULTI_SPECTRAL_SENSOR;
	else
		actualNumSamples = numSamples;

	// ARRANGE the memcpy so that it copies the right number of info
	// first of all make all the status IsValid to false, as an initialization of the data to be provided
	memset(pSensorData, 0, actualNumSamples * sizeof(struct NCSDataMultiSpectralSensor));

	// even if device not started yet, Poll Sensor msut be blocking until new data is available
	// wait for condition
	pthread_mutex_lock(&pVCI->conditionToUnblockPollMutex);
	if (clock_gettime(CLOCK_REALTIME, &timeout) != 0) {
		LOG("Warning : Can't get system time");
	}
	timeout.tv_sec += 1;
	pthread_cond_timedwait(&pVCI->conditionToUnblockPoll, &pVCI->conditionToUnblockPollMutex, &timeout);
	pthread_mutex_unlock(&pVCI->conditionToUnblockPollMutex);

	// wait for api mutex
	pthread_mutex_lock(&pVCI->mutexApi);

	pthread_mutex_lock(&pVCI->mutexFlicker);

	// 2. flicker
	k = 0;
	if (pVCI->dataMultiSpectralSensorFlickerInfoIndex > -1) {

		i = pVCI->dataMultiSpectralSensorFlickerInfoIndex;
		k = 0;
		pSD = (struct NCSDataMultiSpectralSensor *)pSensorData;
		while (pVCI->dataMultiSpectralSensor[i].flickerInfo.isValid) {
			memcpy(&pSD->flickerInfo, &pVCI->dataMultiSpectralSensor[i].flickerInfo, sizeof(struct SpectralFlickerFrequencyInfo));
			if (i == 0)
				i = MAX_DATA_MULTI_SPECTRAL_SENSOR-1;
			else
				i--;

			pSD++;
			k++;
			if (k == actualNumSamples)
				break;
		}
	}

	// mutex unlock
	pthread_mutex_unlock(&pVCI->mutexFlicker);

	// unlock api protections
	pthread_mutex_unlock(&pVCI->mutexApi);

	LOG("PollData from ALS Device OK\n");
	return (k);
}


//
// StartSensor
// Asynchronous. Posts a command to internal main thread.
//
static int StartSensor() {

	// error if not opened
	if (pVCI == NULL)
		return -1;

	// wait for api mutex
	pthread_mutex_lock(&pVCI->mutexApi);

	// error if already started
	if (pVCI->state == STARTED) {
		LOG("StartSensor failed. Device already started\n");
		pthread_mutex_unlock(&pVCI->mutexApi);
		return -1;
	}

	// can not process a command while the previous given is still being processed
	if (pVCI->pendingCommand != commandNone) {
		LOG("StartSensor failed. A command is being processed\n");
		pthread_mutex_unlock(&pVCI->mutexApi);
		return -1;
	}

	// store start command
	pVCI->pendingCommand = commandStart;

	// signal to unlock poll that can be possibly waiting
	pthread_cond_signal(&pVCI->conditionToUnblockMainThread);

	// release mutex api
	pthread_mutex_unlock(&pVCI->mutexApi);

	return 0;
}

//
// StopSensor
// Asynchronous. Posts a command to internal main thread.
// CloseSensor should be called after the command stop is processed
//
static int StopSensor() {
	// error if not opened
	if (pVCI == NULL)
		return -1;

	LOG("Stopping Sensor\n");

	// wait for api mutex
	pthread_mutex_lock(&pVCI->mutexApi);

	// error if not opened or not started
	if (pVCI->state == STOPPED) {
		LOG("StopSensor failed. Device not started\n");
		pthread_mutex_unlock(&pVCI->mutexApi);
		return -1;
	}

	// can not process a command while the previous given is still being processed
	if (pVCI->pendingCommand != commandNone) {
		LOG("StopSensor failed. A command is being processed\n");
		pthread_mutex_unlock(&pVCI->mutexApi);
		return -1;
	}

	// store start command
	pVCI->pendingCommand = commandStop;

	// signal to unlock main thread that can be possibly waiting
	LOG("StopSensor : Wake main Thread\n");
	pthread_cond_signal(&pVCI->conditionToUnblockMainThread);

	// release api mutex
	pthread_mutex_unlock(&pVCI->mutexApi);

	return 0;
}

//
// CloseSensor
// Closing the instance of ALS and flicker detection
//
static int CloseSensor() {

	void * retval;

	// error if not opened
	if (pVCI == NULL) {
		LOG ("CloseSensor failed. Sensor is already closed. \n");
		return -1;
	}

	// as far as stop is asynchronous, CloseSensor shall be blocking, waiting internal things to be actually stopped
	// lets be blocking waiting for the pVCI->state to be stopped
	while (1) {
		pthread_mutex_lock(&pVCI->mutexApi);
		if (pVCI->state == STARTED) {
			pthread_mutex_unlock(&pVCI->mutexApi);
			//LOG("Waiting Stop command to processed ....\n");
			usleep(100);
		}
		else
			break;
	}

	platform_put_client(pVCI->client);
	pVCI->client = NULL;

	// ensure no command is being processed
	if (pVCI->pendingCommand != commandNone) {
		LOG("CloseSensor failed. A command is being processed\n");
		pthread_mutex_unlock(&pVCI->mutexApi);
		return -1;
	}

	// set command to finish main thread
	pVCI->pendingCommand = commandClose;
	// signal to unblock main thread
	pthread_cond_signal(&pVCI->conditionToUnblockMainThread);

	pthread_mutex_unlock(&pVCI->mutexApi);

	// wait for main thread to finish
	pthread_join(pVCI->mainThread, &retval);
	// reset the flag that is useful to make OpenSensor waiting for
	// the main thread to be started before existing nicely
	pVCI->mainThreadStarted = 0;

	// destroy mutexes and cnditions
	pthread_mutex_destroy(&pVCI->conditionToUnblockPollMutex);
	pthread_cond_destroy(&pVCI->conditionToUnblockPoll);
	pthread_mutex_destroy(&pVCI->conditionToUnblockMainThreadMutex);
	pthread_cond_destroy(&pVCI->conditionToUnblockMainThread);
	pthread_mutex_destroy(&pVCI->mutexAls);
	pthread_mutex_destroy(&pVCI->mutexFlicker);
	pthread_mutex_destroy(&pVCI->mutexApi);

	// free allocated memory
	free(pVCI);
	pVCI = NULL;

	LOG("Close ALS Device OK\n");
	return 0;
}


//
// vd628x_SpectralSensorInterface
// Implementation of the SpectralSensorInterface for vd628x
//
static SpectralSensorInterface vd628x_SpectralSensorInterface =
{
	QuerySensorInfo,
	OpenSensor,
	Configure,
	StartSensor,
	PollSensorData,
	StopSensor,
	CloseSensor,
};

//
// vd628x_SpectralSensorInterface
// Enry point of the vd628x driver library
//
VISIBILITY_PUBLIC void GetSpectralSensorInterface(
   SpectralSensorInterface** ppInterfaceObject)
{
	*ppInterfaceObject = &vd628x_SpectralSensorInterface;
}
