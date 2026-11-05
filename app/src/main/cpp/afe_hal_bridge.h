#ifndef AFE_HAL_BRIDGE_H
#define AFE_HAL_BRIDGE_H

#include <jni.h>
#include <string>
#include <vector>
#include <cstdint>

/**
 * Underlying Hardware Abstraction Layer (HAL) data structures
 * for direct DAC negotiation and OEM audio restriction bypass.
 */
struct KernelAlsaDacDescriptor {
    int32_t cardIndex = -1;
    int32_t deviceIndex = -1;
    std::string cardId;
    std::string cardName;
    std::string driverName;
    std::string hardwareChip;
    int32_t minSampleRateHz = 44100;
    int32_t maxSampleRateHz = 192000;
    int32_t maxBitDepth = 24;
    bool supportsDirectPcm = true;
    bool supportsMmap = false;
    std::vector<int32_t> supportedSampleRates;
    std::vector<int32_t> supportedBitDepths;
};

struct DirectHalNegotiationOutput {
    int32_t requestedSampleRateHz = 44100;
    int32_t requestedBitDepth = 16;
    int32_t negotiatedSampleRateHz = 44100;
    int32_t negotiatedBitDepth = 16;
    bool oemRestrictionBypassed = false;
    bool mmapExclusiveEngaged = false;
    std::string activeHalProfile;
    std::string siliconCapability;
    std::string routingPath;
};

struct NativeHalTelemetryData {
    bool isStreamOpen = false;
    bool isExclusiveMode = false;
    bool isMmapUsed = false;
    int32_t sampleRate = 0;
    int32_t bitDepth = 0;
    int32_t channelCount = 0;
    int32_t framesPerBurst = 0;
    int32_t bufferCapacityInFrames = 0;
    int32_t xRunCount = 0;
    int32_t deviceId = 0;
    int64_t hardwareTimestampMs = -1;
    std::string halStateDescription;
};

#endif // AFE_HAL_BRIDGE_H
