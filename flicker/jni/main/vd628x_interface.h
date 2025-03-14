/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#ifndef VD628X_INTERFACE_H
#define VD628X_INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#if defined(__GNUC__)
#define VISIBILITY_PUBLIC __attribute__ ((visibility ("default")))
#define CDK_PACK_SIZE 8
#endif // defined(__GNUC__)

const uint16_t MaxStringSize = 256;

#if !defined (TRUE)
#define TRUE                1
#endif // !defined (TRUE)

#if !defined (FALSE)
#define FALSE               0
#endif // !defined (FALSE)

/// @brief Data structures that are exposed to OEM must be packed to have the expected layout.
#pragma pack(push, CDK_PACK_SIZE)

// @brief Range structure in float
struct RangeFloat
{
    float min;    ///< Minimum
    float max;    ///< Maximum
};
// @brief Single Sensor Attribute
struct Attribute
{
    const char attributeName[MaxStringSize];  ///< Lux, Frequency, CCT, Channel Strength, power, samplingRate etc
    union
    {
        RangeFloat  range;       ///< Range of the attribute. Min=Max for an attribute with no range
    } data;
};

// @breif Sensor Attribute array
struct SensorAttribute
{
    uint8_t            length;             ///< length of array
    Attribute*       pAttributes;        ///< Array of quantities
};


// @breif Driver Info
struct DriverInformation
{
    const char      name[MaxStringSize];               ///< Name of this sensor
    const char      vendor[MaxStringSize];             ///< Vendor of this sensor
    const char      hardwareVersion[MaxStringSize];    ///< Hardware Versions e.g VD6282, VD6283
    uint16_t          driverVersion;                     ///< Software Driver version
};


// @brief Flicker Channel Configured by Sensor
enum FlickerChannel
{
    InvalidChannel = -1,   ///< Invalid
    RedChannel     = 0,    ///< Red
    GreenChannel,          ///< Green
    BlueChannel,           ///< Blue
    ClearChannel1,         ///< Clear
    ClearChannel2,         ///< Clear
    InfraredChannel,       ///< IR
    MaxChannel = 0xFFFF    ///< Max Value
};



// @brief Given Flicker Frequency and Amplitude in FFT
struct SpectralFrequencyInfo
{
    float amplitude;     ///< Amplitude from Fast Fourier Transform (FFT) on Sensor Flicker Data
    float frequency;     ///< Flicker frequency peak value
};


// @brief Flicker Frequencies related inforamtion
struct SpectralFlickerFrequencyInfo
{
    bool                  isValid;                      ///< Data is Valid or Not
    FlickerChannel        channel;                      ///< Primary Flicker Channel Configured
    SpectralFrequencyInfo firstMaximaPeak;              ///< First maximum freq amplitude in FFT
    SpectralFrequencyInfo secondMaximaPeak;             ///< Second maximum freq amplitude in FFT
    uint16_t                configuredSamplingFlickerFreq; ///< Sampling Flicker Frequency configured by the sensor
    uint16_t                avgRawFlickerData;            ///< The average value for the raw of flicker channel
    uint16_t                maxRawFlickerData;            ///< The max value for the raw of flicker channel
    uint16_t                minRawFlickerData;            ///< The min value for the raw of flicker channel
    uint16_t                avgFlickerFreqAmplitude;    ///< Average of 5 different flicker frequency amplitude calculated by FFT
    float                 expGainOfFlickerChannel;      ///< Exposure Gain of the flicker channel
    float                 expTimeOfFlickerChannel;      ///< Exposure Time of the flicker channel if available otherwise -1.0
};


// @brief Main Data Structure that contains Spectral Sensor Data
struct NCSDataMultiSpectralSensor
{
    uint64_t                        timestamp;      ///< Time Stamp of the data
    SpectralFlickerFrequencyInfo  flickerInfo;    ///< Flicker Information
};



// @brief Configuration Type Enum
enum QueryPayloadType
{
    DriverInfo,         ///< Sensor Type and Vendor Information.
                        ///  Payload: DriverInformation
    SensorAttributes,   ///< List of sensor attributes
                        ///  Payload: SensorAttribute
    MaxPayloadTypeCount ///<  Maximum
};

// To@Do Generalize
// @brief Configuration Type Enum
enum ConfigurationType
{
    SamplingFrequency, ///< Configure Sampling Frequency of the flicker Channel in Hertz
                       ///  Payload: UINT32
    SamplingTime,      ///< Configure Sampling time of CCT and Lux on non-flicker channels in micro sec
                       ///  Payload: UINT32
    QTimeStamp,        ///< Sets the QTimer timestamp to the driver to synchronize clocks and reduce clock drift.
                       ///  Camera runs w.r.t Qtimer. will call it after regular intervals.
                       ///  Payload: uint64_t_t Qtimer timestamp
    MaxConfigType      ///<  Maximum
};

// @brief QueryInfo structure to query into any Sensor Driver supporting this Spectral Sensor Interface
struct QueryInfo
{
   QueryPayloadType queryType;   ///< Type of query
   void*            pData;       ///< Payload will be typecasted w.r.t queryType
   uint16_t           size;        ///< Size of the structure pData. Needs to be written by Vendor driver
};

// @brief Parameters to be configuredinto any Sensor Driver supporting this Spectral Sensor Interface

struct ConfigureParameters
{
    ConfigurationType configType;   ///< Type Of Configuration
    union
    {
        uint32_t            samplingTime;       ///< Sampling Time of the spectral sensor for CCT/LUX
        uint32_t            samplingFrequency;  ///< Sampling Frequency of the flicker channel
        uint64_t            timestamp;          ///< Qtimer Time Stamp
    } configPayload;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Interface for Exeternal Sensor
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct SpectralSensorInterface
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// QuerySensorInfo
    ///
    /// @brief  This method can be called to query sensor's characteristics
    ///
    /// @param  pCaps  Pointer to SensorCapability to be filled by OEM
    ///
    /// @return None
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void (*QuerySensorInfo)(
        QueryInfo* pQuery);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// OpenSensor
    ///
    /// @brief  Opens a link to the Spectral sensor if it is present. SpectralSensorInterface operations
    ///         can be performed if 1 returned
    ///
    /// @return -1 for error.  -2 for sensor not present . 0 for success
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int (*OpenSensor)();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Configure
    ///
    /// @brief  Send Configurations to be applied
    ///
    /// @param  pConfig             pointer to config structure
    /// @param  numOfConfigParams   Number of configuration parameters. pConfig array length
    ///
    /// @return sucess is 0
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int (*Configure)(
        const ConfigureParameters* pConfig,
        const unsigned int                 numOfConfigParams);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// StartSensor
    ///
    /// @brief  Once the link is open. And the configurations are applied, Start sensor.
    ///
    /// @return sucess is 0
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int (*StartSensor)();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// PollSensorData
    ///
    /// @brief  Blocking Call to poll sensor data.
    ///
    /// @param  numSamples   Number of samples requested. 1 for single smaple. > 1 for Batched sample
    /// @param  pSensorData  Pointer to Sensor Data. Can be typecasted to expected sensor data structure based on
    ///                      NCSSensorType enum defined and Queried from sensor
    ///
    /// @return -1 if failed OR number of samples returned
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int (*PollSensorData)(
        const uint8_t   numSamples,
        void*         pSensorData);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// StopSensor
    ///
    /// @brief  Stop the sensor streaming
    ///
    /// @return sucess is 0
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int (*StopSensor)();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// CloseSensor
    ///
    /// @brief  Turn off sensor
    ///
    /// @return sucess is 0
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int (*CloseSensor)();

} SpectralSensorInterface;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// GetSpectralSensorInterface
///
/// @brief  This function is the entry point to the Spectral sensor driver. will call into OEM's sensor driver lib using
///         "GetSpectralSensorInterface" string. Sensor will be then operated using
///         SpectralSensorInterface structure as defined above.
///
/// @param  ppInterfaceObject    Double Pointer to structure. Pointer from will be passed by reference to the OEM driver
///
/// @return None
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
VISIBILITY_PUBLIC void GetSpectralSensorInterface(
    SpectralSensorInterface** ppInterfaceObject);

#pragma pack(pop)

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // VD628X_INTERFACE_H
