#pragma once

#include <jni.h>
#include <cstdint>
#include <vector>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <linux/usbdevice_fs.h>
#include "lock_free_ring_buffer.h"

// Ring buffer capacity for USB streaming (128K samples = ~64K stereo frames)
constexpr size_t USB_RING_BUFFER_CAPACITY = 131072;

// Maximum isochronous URB buffers in flight for zero-drop double/triple buffering
constexpr int NUM_ISO_URBS = 4;
constexpr int PACKETS_PER_URB = 8;

enum class UacVersion {
    UNKNOWN = 0,
    UAC1 = 1,
    UAC2 = 2
};

enum class UsbDsdMode {
    NONE = 0,
    DOP = 1,
    NATIVE_DSD = 2
};

struct AudioStreamingEndpointInfo {
    uint8_t endpointAddress = 0;
    uint8_t attributes = 0;
    uint16_t maxPacketSize = 0;
    uint8_t interval = 1;
};

struct AudioFormatDescriptorInfo {
    uint8_t formatType = 1;
    uint8_t channelCount = 2;
    uint8_t subframeBytes = 2; // 2, 3, or 4 bytes
    uint8_t bitResolution = 16; // 16, 24, or 32 bits
    std::vector<uint32_t> sampleRates; // 44100 to 768000 Hz
    bool supportsDsd = false;
};

struct AudioStreamingInterfaceInfo {
    int interfaceNumber = 0;
    int altSetting = 0;
    UacVersion uacVersion = UacVersion::UAC2;
    AudioStreamingEndpointInfo endpoint;
    AudioFormatDescriptorInfo format;
};

struct UsbDacCapabilities {
    bool isValid = false;
    UacVersion version = UacVersion::UAC2;
    std::string manufacturer;
    std::string product;
    int vendorId = 0;
    int productId = 0;
    std::vector<AudioStreamingInterfaceInfo> streamingInterfaces;
    std::vector<uint32_t> allSupportedSampleRates;
    std::vector<uint8_t> allSupportedBitDepths;
    uint8_t clockSourceId = 1;
    bool supportsNativeDsd = false;
    bool supportsDoP = false;
};

class UsbDescriptorParser {
public:
    static UsbDacCapabilities parseDevice(int fd);
    static bool setInterfaceAltSetting(int fd, int interfaceNumber, int altSetting);
    static bool setUac1SampleRate(int fd, uint8_t endpointAddress, uint32_t sampleRate);
    static bool setUac2SampleRate(int fd, int clockId, uint32_t sampleRate);
    static bool claimInterface(int fd, int interfaceNumber);
    static bool releaseInterface(int fd, int interfaceNumber);
};

class UsbIsochronousStreamer {
public:
    UsbIsochronousStreamer();
    ~UsbIsochronousStreamer();

    bool start(
        int fd,
        int interfaceNumber,
        int altSetting,
        uint8_t endpointAddress,
        uint32_t sampleRate,
        uint8_t bitDepth,
        uint8_t channels
    );

    void stop();
    size_t writePcmBytes(const uint8_t* data, size_t size);
    size_t writePcm16(const int16_t* data, size_t sampleCount);
    size_t writePcmFloats(const float* data, size_t count, uint8_t targetBitDepth);
    size_t writeDsdDoP(const uint8_t* dsdL, const uint8_t* dsdR, size_t sampleCount);
    size_t writeNativeDsd(const uint8_t* dsdData, size_t size);

    int64_t getFramesStreamed() const { return mFramesStreamed.load(std::memory_order_relaxed); }
    int64_t getTransfersSubmitted() const { return mTransfersSubmitted.load(std::memory_order_relaxed); }
    int64_t getTransfersCompleted() const { return mTransfersCompleted.load(std::memory_order_relaxed); }
    int32_t getXRunCount() const { return mXRuns.load(std::memory_order_relaxed); }
    bool isStreaming() const { return mIsStreaming.load(std::memory_order_relaxed); }
    uint32_t getSampleRate() const { return mSampleRate; }
    uint8_t getBitDepth() const { return mBitDepth; }
    bool isAudioDataDetected() const { return mAudioDataDetected.load(std::memory_order_relaxed); }
    float getPeakAudioLevel() const { return mPeakLevel.load(std::memory_order_relaxed); }
    void resetAudioDataDetection() {
        mAudioDataDetected.store(false, std::memory_order_relaxed);
        mPeakLevel.store(0.0f, std::memory_order_relaxed);
    }

private:
    void streamingThreadLoop();
    void reapUrbs();

    int mFd = -1;
    int mInterfaceNumber = 0;
    int mAltSetting = 0;
    uint8_t mEndpointAddress = 0;
    uint32_t mSampleRate = 48000;
    uint8_t mBitDepth = 24;
    uint8_t mChannels = 2;
    size_t mBytesPerFrame = 4;
    size_t mBytesPerPacket = 192;

    // DoP marker alternating state (0x05 and 0xFA per DoP 1.1 spec)
    uint8_t mDopMarker = 0x05;

    std::atomic<bool> mIsStreaming{false};
    std::atomic<bool> mStopRequested{false};
    std::atomic<int64_t> mFramesStreamed{0};
    std::atomic<int64_t> mTransfersSubmitted{0};
    std::atomic<int64_t> mTransfersCompleted{0};
    std::atomic<int32_t> mXRuns{0};
    std::atomic<bool> mAudioDataDetected{false};
    std::atomic<float> mPeakLevel{0.0f};

    std::thread mStreamThread;

    // Double/quad-buffered URBs and payload memories
    usbdevfs_urb* mUrbs[NUM_ISO_URBS];
    uint8_t* mUrbBuffers[NUM_ISO_URBS];
    size_t mUrbBufferSize = 0;

    LockFreeAudioRingBuffer<uint8_t, USB_RING_BUFFER_CAPACITY> mRingBuffer;
};

enum class UacMuteState {
    UNSUPPORTED = 0,
    UNMUTED = 1,
    MUTED = 2,
    UNKNOWN = 3
};

enum class UacVolumeState {
    UNSUPPORTED = 0,
    REPORTED = 1,
    UNKNOWN = 2
};

struct UsbVolumeInfo {
    UacVolumeState state = UacVolumeState::UNKNOWN;
    float currentDb = 0.0f;
    float minDb = 0.0f;
    float maxDb = 0.0f;
    float resDb = 0.0f;
    int16_t rawCur = 0;
};

class UsbControlInspector {
public:
    static UacMuteState readMuteState(int fd, int ctrlIface, int unitId, int channel, UacVersion version);
    static bool safeUnmute(int fd, int ctrlIface, int unitId, int channel, UacVersion version);
    static UsbVolumeInfo readVolume(int fd, int ctrlIface, int unitId, int channel, UacVersion version);
};


