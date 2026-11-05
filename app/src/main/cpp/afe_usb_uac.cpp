#include "afe_usb_uac.h"
#include <android/log.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <vector>

#define TAG "AFE_NativeUsbUac"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

// Standard USB Descriptor Types
#define USB_DT_DEVICE            0x01
#define USB_DT_CONFIG            0x02
#define USB_DT_STRING            0x03
#define USB_DT_INTERFACE         0x04
#define USB_DT_ENDPOINT          0x05
#define USB_DT_CS_INTERFACE      0x24
#define USB_DT_CS_ENDPOINT       0x25

// USB Audio Class constants
#define USB_CLASS_AUDIO          0x01
#define USB_SUBCLASS_AUDIOCONTROL 0x01
#define USB_SUBCLASS_AUDIOSTREAMING 0x02

// UAC1 / UAC2 Class-Specific Subtypes
#define UAC_AS_GENERAL           0x01
#define UAC_FORMAT_TYPE          0x02
#define UAC2_CLOCK_SOURCE        0x0B

// Control transfer requests
#define USB_REQ_GET_DESCRIPTOR   0x06
#define USB_REQ_SET_INTERFACE    0x0B
#define USB_TYPE_STANDARD        (0x00 << 5)
#define USB_TYPE_CLASS           (0x01 << 5)
#define USB_RECIP_INTERFACE      0x01
#define USB_RECIP_ENDPOINT       0x02
#define USB_DIR_IN               0x80
#define USB_DIR_OUT              0x00

static bool usbControlTransfer(int fd, uint8_t reqType, uint8_t request, uint16_t value, uint16_t index, void* data, uint16_t length) {
    if (fd < 0) return false;
    struct usbdevfs_ctrltransfer ctrl;
    std::memset(&ctrl, 0, sizeof(ctrl));
    ctrl.bRequestType = reqType;
    ctrl.bRequest = request;
    ctrl.wValue = value;
    ctrl.wIndex = index;
    ctrl.wLength = length;
    ctrl.timeout = 2000;
    ctrl.data = data;

    int ret = ioctl(fd, USBDEVFS_CONTROL, &ctrl);
    if (ret < 0) {
        LOGW("USBDEVFS_CONTROL failed (reqType: 0x%02x, req: 0x%02x): %s", reqType, request, strerror(errno));
        return false;
    }
    return true;
}

bool UsbDescriptorParser::setInterfaceAltSetting(int fd, int interfaceNumber, int altSetting) {
    if (fd < 0) return false;
    struct usbdevfs_setinterface setif;
    setif.interface = interfaceNumber;
    setif.altsetting = altSetting;
    int ret = ioctl(fd, USBDEVFS_SETINTERFACE, &setif);
    if (ret < 0) {
        LOGW("USBDEVFS_SETINTERFACE failed for iface %d alt %d: %s", interfaceNumber, altSetting, strerror(errno));
        return false;
    }
    LOGI("USB interface %d set to alternate setting %d successfully", interfaceNumber, altSetting);
    return true;
}

bool UsbDescriptorParser::setUac1SampleRate(int fd, uint8_t endpointAddress, uint32_t sampleRate) {
    if (fd < 0) return false;
    uint8_t data[3];
    data[0] = static_cast<uint8_t>(sampleRate & 0xFF);
    data[1] = static_cast<uint8_t>((sampleRate >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((sampleRate >> 16) & 0xFF);

    uint8_t reqType = USB_TYPE_CLASS | USB_RECIP_ENDPOINT | USB_DIR_OUT;
    uint8_t request = 0x01; // SET_CUR
    uint16_t value = (0x01 << 8); // SAMPLING_FREQ_CONTROL
    uint16_t index = endpointAddress;

    return usbControlTransfer(fd, reqType, request, value, index, data, 3);
}

bool UsbDescriptorParser::setUac2SampleRate(int fd, int clockId, uint32_t sampleRate) {
    // UAC2 SAM_FREQ_CONTROL: CS request to clock source entity
    uint8_t data[4];
    data[0] = static_cast<uint8_t>(sampleRate & 0xFF);
    data[1] = static_cast<uint8_t>((sampleRate >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((sampleRate >> 16) & 0xFF);
    data[3] = static_cast<uint8_t>((sampleRate >> 24) & 0xFF);

    uint8_t reqType = USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT;
    uint8_t request = 0x01; // CUR attribute
    uint16_t value = (0x01 << 8); // CS_SAM_FREQ_CONTROL (0x01)
    uint16_t index = (clockId << 8); // Clock entity ID

    return usbControlTransfer(fd, reqType, request, value, index, data, 4);
}

bool UsbDescriptorParser::claimInterface(int fd, int interfaceNumber) {
    if (fd < 0) return false;
    int iface = interfaceNumber;
    int ret = ioctl(fd, USBDEVFS_CLAIMINTERFACE, &iface);
    if (ret < 0) {
        LOGW("USBDEVFS_CLAIMINTERFACE failed for interface %d: %s", interfaceNumber, strerror(errno));
        return false;
    }
    LOGI("USBDEVFS_CLAIMINTERFACE success for interface %d", interfaceNumber);
    return true;
}

bool UsbDescriptorParser::releaseInterface(int fd, int interfaceNumber) {
    if (fd < 0) return false;
    int iface = interfaceNumber;
    int ret = ioctl(fd, USBDEVFS_RELEASEINTERFACE, &iface);
    if (ret < 0) {
        LOGW("USBDEVFS_RELEASEINTERFACE failed for interface %d: %s", interfaceNumber, strerror(errno));
        return false;
    }
    LOGI("USBDEVFS_RELEASEINTERFACE success for interface %d", interfaceNumber);
    return true;
}


UsbDacCapabilities UsbDescriptorParser::parseDevice(int fd) {
    UsbDacCapabilities caps;
    caps.isValid = false;
    if (fd < 0) return caps;

    // Read full configuration descriptor (standard max 4096 bytes)
    uint8_t header[9];
    if (!usbControlTransfer(fd, USB_DIR_IN | USB_TYPE_STANDARD | 0x00, USB_REQ_GET_DESCRIPTOR, (USB_DT_CONFIG << 8), 0, header, 9)) {
        LOGW("Could not fetch USB configuration descriptor header");
        return caps;
    }

    uint16_t totalLength = static_cast<uint16_t>(header[2] | (header[3] << 8));
    if (totalLength < 9 || totalLength > 8192) {
        totalLength = 1024;
    }

    std::vector<uint8_t> buffer(totalLength);
    if (!usbControlTransfer(fd, USB_DIR_IN | USB_TYPE_STANDARD | 0x00, USB_REQ_GET_DESCRIPTOR, (USB_DT_CONFIG << 8), 0, buffer.data(), totalLength)) {
        LOGW("Could not fetch full USB configuration descriptor of length %d", totalLength);
        return caps;
    }

    caps.isValid = true;
    caps.version = UacVersion::UAC2; // Default assume modern UAC2

    size_t offset = 0;
    int currentInterface = -1;
    int currentAltSetting = 0;
    bool isAudioStreaming = false;
    AudioStreamingInterfaceInfo currentStreamInfo;

    std::vector<uint32_t> standardRates = {44100, 48000, 88200, 96000, 176400, 192000, 352800, 384000, 705600, 768000};

    while (offset + 2 <= buffer.size()) {
        uint8_t len = buffer[offset];
        uint8_t type = buffer[offset + 1];

        if (len == 0 || offset + len > buffer.size()) break;

        if (type == USB_DT_INTERFACE && len >= 9) {
            uint8_t ifNumber = buffer[offset + 2];
            uint8_t alt = buffer[offset + 3];
            uint8_t ifClass = buffer[offset + 5];
            uint8_t ifSubclass = buffer[offset + 6];
            uint8_t ifProtocol = buffer[offset + 7];

            if (isAudioStreaming && currentStreamInfo.endpoint.endpointAddress != 0) {
                caps.streamingInterfaces.push_back(currentStreamInfo);
                currentStreamInfo = AudioStreamingInterfaceInfo();
            }

            currentInterface = ifNumber;
            currentAltSetting = alt;
            isAudioStreaming = (ifClass == USB_CLASS_AUDIO && ifSubclass == USB_SUBCLASS_AUDIOSTREAMING);

            if (isAudioStreaming) {
                currentStreamInfo.interfaceNumber = currentInterface;
                currentStreamInfo.altSetting = currentAltSetting;
                currentStreamInfo.uacVersion = (ifProtocol == 0x20) ? UacVersion::UAC2 : UacVersion::UAC1;
                caps.version = currentStreamInfo.uacVersion;
            }
        } else if (!isAudioStreaming && type == USB_DT_CS_INTERFACE && len >= 4) {
            uint8_t subtype = buffer[offset + 2];
            if (subtype == 0x0B /* UAC2_CLOCK_SOURCE */) {
                caps.clockSourceId = buffer[offset + 3];
            }
        } else if (isAudioStreaming && type == USB_DT_CS_INTERFACE && len >= 3) {
            uint8_t subtype = buffer[offset + 2];
            if (subtype == UAC_FORMAT_TYPE && len >= 7) {
                uint8_t formatType = buffer[offset + 3];
                uint8_t channels = buffer[offset + 4];
                uint8_t subframeBytes = buffer[offset + 5];
                uint8_t bitRes = buffer[offset + 6];

                currentStreamInfo.format.formatType = formatType;
                currentStreamInfo.format.channelCount = channels;
                currentStreamInfo.format.subframeBytes = subframeBytes;
                currentStreamInfo.format.bitResolution = bitRes;

                if (caps.version == UacVersion::UAC1 && len >= 8) {
                    uint8_t samFreqType = buffer[offset + 7];
                    if (samFreqType == 0 && len >= 14) {
                        // Continuous frequency range: min at offset 8..10, max at 11..13
                        uint32_t minRate = buffer[offset + 8] | (buffer[offset + 9] << 8) | (buffer[offset + 10] << 16);
                        uint32_t maxRate = buffer[offset + 11] | (buffer[offset + 12] << 8) | (buffer[offset + 13] << 16);
                        for (uint32_t sr : standardRates) {
                            if (sr >= minRate && sr <= maxRate) {
                                currentStreamInfo.format.sampleRates.push_back(sr);
                            }
                        }
                    } else if (samFreqType > 0) {
                        // Discrete sampling rates
                        for (int s = 0; s < samFreqType && (offset + 8 + s * 3 + 2 < offset + len); ++s) {
                            int base = offset + 8 + s * 3;
                            uint32_t rate = buffer[base] | (buffer[base + 1] << 8) | (buffer[base + 2] << 16);
                            currentStreamInfo.format.sampleRates.push_back(rate);
                        }
                    }
                } else if (caps.version == UacVersion::UAC2) {
                    // Modern UAC2 DACs natively handle standard sample rate matrix up to 768kHz
                    currentStreamInfo.format.sampleRates = standardRates;
                }
            }
        } else if (isAudioStreaming && type == USB_DT_ENDPOINT && len >= 7) {
            uint8_t epAddr = buffer[offset + 2];
            uint8_t epAttr = buffer[offset + 3];
            uint16_t maxPacket = buffer[offset + 4] | (buffer[offset + 5] << 8);
            uint8_t interval = buffer[offset + 6];

            // Isochronous OUT endpoint
            if ((epAttr & 0x03) == 0x01 && (epAddr & 0x80) == 0x00) {
                currentStreamInfo.endpoint.endpointAddress = epAddr;
                currentStreamInfo.endpoint.attributes = epAttr;
                currentStreamInfo.endpoint.maxPacketSize = maxPacket;
                currentStreamInfo.endpoint.interval = interval;
            }
        }

        offset += len;
    }

    if (isAudioStreaming && currentStreamInfo.endpoint.endpointAddress != 0) {
        caps.streamingInterfaces.push_back(currentStreamInfo);
    }

    // Aggregate unique sample rates and bit depths
    for (const auto& iface : caps.streamingInterfaces) {
        for (uint32_t r : iface.format.sampleRates) {
            if (std::find(caps.allSupportedSampleRates.begin(), caps.allSupportedSampleRates.end(), r) == caps.allSupportedSampleRates.end()) {
                caps.allSupportedSampleRates.push_back(r);
            }
        }
        uint8_t bd = iface.format.bitResolution;
        if (bd > 0 && std::find(caps.allSupportedBitDepths.begin(), caps.allSupportedBitDepths.end(), bd) == caps.allSupportedBitDepths.end()) {
            caps.allSupportedBitDepths.push_back(bd);
        }
    }

    // Default fallbacks if descriptors were compact
    if (caps.allSupportedSampleRates.empty()) {
        caps.allSupportedSampleRates = standardRates;
    }
    if (std::find(caps.allSupportedBitDepths.begin(), caps.allSupportedBitDepths.end(), 32) == caps.allSupportedBitDepths.end()) {
        caps.allSupportedBitDepths.push_back(32);
    }
    if (std::find(caps.allSupportedBitDepths.begin(), caps.allSupportedBitDepths.end(), 24) == caps.allSupportedBitDepths.end()) {
        caps.allSupportedBitDepths.push_back(24);
    }
    if (std::find(caps.allSupportedBitDepths.begin(), caps.allSupportedBitDepths.end(), 16) == caps.allSupportedBitDepths.end()) {
        caps.allSupportedBitDepths.push_back(16);
    }

    std::sort(caps.allSupportedSampleRates.begin(), caps.allSupportedSampleRates.end());
    std::sort(caps.allSupportedBitDepths.begin(), caps.allSupportedBitDepths.end());

    LOGI("Parsed USB Audio DAC: UAC%d, %zu streaming interfaces, %zu sample rates, max: %u Hz",
         (caps.version == UacVersion::UAC2 ? 2 : 1),
         caps.streamingInterfaces.size(),
         caps.allSupportedSampleRates.size(),
         caps.allSupportedSampleRates.empty() ? 0 : caps.allSupportedSampleRates.back());

    return caps;
}

UsbIsochronousStreamer::UsbIsochronousStreamer() {
    for (int i = 0; i < NUM_ISO_URBS; ++i) {
        mUrbs[i] = nullptr;
        mUrbBuffers[i] = nullptr;
    }
}

UsbIsochronousStreamer::~UsbIsochronousStreamer() {
    stop();
}

bool UsbIsochronousStreamer::start(
    int fd,
    int interfaceNumber,
    int altSetting,
    uint8_t endpointAddress,
    uint32_t sampleRate,
    uint8_t bitDepth,
    uint8_t channels
) {
    stop();

    if (fd < 0) {
        LOGE("Invalid start parameters for USB isochronous streamer: fd=%d", fd);
        return false;
    }

    UsbDacCapabilities caps = UsbDescriptorParser::parseDevice(fd);
    if (!caps.streamingInterfaces.empty()) {
        const auto& streamIface = caps.streamingInterfaces[0];
        if (endpointAddress == 0 || endpointAddress == 1) {
            endpointAddress = streamIface.endpoint.endpointAddress;
        }
        if (interfaceNumber <= 0) {
            interfaceNumber = streamIface.interfaceNumber;
        }
        if (altSetting <= 0) {
            altSetting = (streamIface.altSetting > 0) ? streamIface.altSetting : 1;
        }
    }
    if (endpointAddress == 0) endpointAddress = 0x01;
    if (altSetting <= 0) altSetting = 1;

    mFd = fd;
    mInterfaceNumber = interfaceNumber;
    mAltSetting = altSetting;
    mEndpointAddress = endpointAddress;
    mSampleRate = (sampleRate > 0) ? sampleRate : 48000;
    if (bitDepth == 32 || (!caps.streamingInterfaces.empty() && caps.streamingInterfaces[0].format.subframeBytes >= 4)) {
        mBitDepth = 32;
    } else {
        mBitDepth = (bitDepth > 0) ? bitDepth : 24;
    }
    mChannels = (channels > 0) ? channels : 2;

    size_t subframeBytes = (mBitDepth + 7) / 8;
    if (subframeBytes < 2) subframeBytes = 2;
    mBytesPerFrame = mChannels * subframeBytes;

    // Kernel claim interface and activate isochronous alternate setting (Alt 1)
    UsbDescriptorParser::claimInterface(mFd, mInterfaceNumber);
    UsbDescriptorParser::setInterfaceAltSetting(mFd, mInterfaceNumber, mAltSetting);

    // Set sample rate on UAC2 clock entities or UAC1 endpoint
    int clockId = (caps.clockSourceId > 0) ? caps.clockSourceId : 0x01;
    UsbDescriptorParser::setUac2SampleRate(mFd, clockId, mSampleRate);
    UsbDescriptorParser::setUac2SampleRate(mFd, 0x29, mSampleRate);
    UsbDescriptorParser::setUac2SampleRate(mFd, 0x28, mSampleRate);
    UsbDescriptorParser::setUac2SampleRate(mFd, 0x02, mSampleRate);
    UsbDescriptorParser::setUac1SampleRate(mFd, mEndpointAddress, mSampleRate);

    // Calculate packet sizing:
    // High Speed UAC2 uses 125us microframes (8000 packets/sec). Full Speed uses 1ms (1000 packets/sec).
    uint32_t packetsPerSec = (caps.version == UacVersion::UAC2) ? 8000 : 1000;
    uint32_t framesPerPacket = (mSampleRate + packetsPerSec - 1) / packetsPerSec;
    if (framesPerPacket < 1) framesPerPacket = 1;
    mBytesPerPacket = framesPerPacket * mBytesPerFrame;
    if (!caps.streamingInterfaces.empty() && caps.streamingInterfaces[0].endpoint.maxPacketSize > 0) {
        if (mBytesPerPacket > caps.streamingInterfaces[0].endpoint.maxPacketSize) {
            mBytesPerPacket = caps.streamingInterfaces[0].endpoint.maxPacketSize;
        }
    }
    if (mBytesPerPacket < mBytesPerFrame) mBytesPerPacket = mBytesPerFrame;

    // Allocate double-buffered asynchronous URBs
    mUrbBufferSize = mBytesPerPacket * PACKETS_PER_URB;
    for (int i = 0; i < NUM_ISO_URBS; ++i) {
        size_t urbStructSize = sizeof(struct usbdevfs_urb) + sizeof(struct usbdevfs_iso_packet_desc) * PACKETS_PER_URB;
        mUrbs[i] = static_cast<struct usbdevfs_urb*>(std::malloc(urbStructSize));
        mUrbBuffers[i] = static_cast<uint8_t*>(std::malloc(mUrbBufferSize));
        std::memset(mUrbBuffers[i], 0, mUrbBufferSize);
    }

    mFramesStreamed.store(0, std::memory_order_relaxed);
    mTransfersSubmitted.store(0, std::memory_order_relaxed);
    mTransfersCompleted.store(0, std::memory_order_relaxed);
    mXRuns.store(0, std::memory_order_relaxed);
    mStopRequested.store(false, std::memory_order_release);
    mIsStreaming.store(true, std::memory_order_release);

    mStreamThread = std::thread(&UsbIsochronousStreamer::streamingThreadLoop, this);
    LOGI("USB Isochronous Streaming started: %u Hz, %d-bit, %d ch, %zu bytes/packet, ep 0x%02x, iface %d alt %d",
         mSampleRate, mBitDepth, mChannels, mBytesPerPacket, mEndpointAddress, mInterfaceNumber, mAltSetting);
    return true;
}

void UsbIsochronousStreamer::stop() {
    mStopRequested.store(true, std::memory_order_release);
    if (mStreamThread.joinable()) {
        mStreamThread.join();
    }

    // Cancel and discard in-flight URBs
    if (mFd >= 0) {
        for (int i = 0; i < NUM_ISO_URBS; ++i) {
            if (mUrbs[i]) {
                ioctl(mFd, USBDEVFS_DISCARDURB, mUrbs[i]);
            }
        }
        // Set interface back to Alt 0 (zero bandwidth idle state)
        UsbDescriptorParser::setInterfaceAltSetting(mFd, mInterfaceNumber, 0);
    }

    for (int i = 0; i < NUM_ISO_URBS; ++i) {
        if (mUrbs[i]) {
            std::free(mUrbs[i]);
            mUrbs[i] = nullptr;
        }
        if (mUrbBuffers[i]) {
            std::free(mUrbBuffers[i]);
            mUrbBuffers[i] = nullptr;
        }
    }

    mIsStreaming.store(false, std::memory_order_release);
    mFd = -1;
    LOGI("USB Isochronous Streaming stopped cleanly");
}

size_t UsbIsochronousStreamer::writePcm16(const int16_t* data, size_t sampleCount) {
    if (!data || sampleCount == 0) return 0;

    float peak = 0.0f;
    for (size_t i = 0; i < sampleCount; ++i) {
        float absVal = std::abs(static_cast<float>(data[i])) / 32768.0f;
        if (absVal > peak) peak = absVal;
    }
    if (peak > 1e-4f) {
        mAudioDataDetected.store(true, std::memory_order_relaxed);
        mPeakLevel.store(peak, std::memory_order_relaxed);
    }

    if (mBitDepth == 16) {
        return mRingBuffer.write(reinterpret_cast<const uint8_t*>(data), sampleCount * sizeof(int16_t));
    } else if (mBitDepth == 24) {
        // Expand 16-bit to 24-bit packed: 3 bytes per sample, shifted left 8 bits (bit-perfect alignment)
        std::vector<uint8_t> out(sampleCount * 3);
        for (size_t i = 0; i < sampleCount; ++i) {
            int32_t s24 = static_cast<int32_t>(data[i]) << 8;
            out[i * 3 + 0] = static_cast<uint8_t>(s24 & 0xFF);
            out[i * 3 + 1] = static_cast<uint8_t>((s24 >> 8) & 0xFF);
            out[i * 3 + 2] = static_cast<uint8_t>((s24 >> 16) & 0xFF);
        }
        return mRingBuffer.write(out.data(), out.size());
    } else { // 32-bit
        // Expand 16-bit to 32-bit: 4 bytes per sample, shifted left 16 bits (bit-perfect alignment)
        std::vector<int32_t> out(sampleCount);
        for (size_t i = 0; i < sampleCount; ++i) {
            out[i] = static_cast<int32_t>(data[i]) << 16;
        }
        return mRingBuffer.write(reinterpret_cast<const uint8_t*>(out.data()), out.size() * sizeof(int32_t));
    }
}

size_t UsbIsochronousStreamer::writePcmBytes(const uint8_t* data, size_t size) {
    if (!data || size == 0) return 0;
    // If incoming data is 16-bit PCM and endpoint bitDepth is 24 or 32, expand using writePcm16
    if (mBitDepth > 16 && (size % 2 == 0)) {
        return writePcm16(reinterpret_cast<const int16_t*>(data), size / sizeof(int16_t));
    }
    bool hasNonZero = false;
    for (size_t i = 0; i < size; ++i) {
        if (data[i] != 0x00) {
            hasNonZero = true;
            break;
        }
    }
    if (hasNonZero) {
        mAudioDataDetected.store(true, std::memory_order_relaxed);
    }
    return mRingBuffer.write(data, size);
}

size_t UsbIsochronousStreamer::writePcmFloats(const float* data, size_t count, uint8_t targetBitDepth) {
    if (!data || count == 0) return 0;

    float peak = 0.0f;
    for (size_t i = 0; i < count; ++i) {
        float absVal = std::abs(data[i]);
        if (absVal > peak) peak = absVal;
    }
    if (peak > 1e-4f) {
        mAudioDataDetected.store(true, std::memory_order_relaxed);
        mPeakLevel.store(peak, std::memory_order_relaxed);
    }

    std::vector<uint8_t> pcmBytes;
    if (targetBitDepth == 16) {
        pcmBytes.resize(count * 2);
        int16_t* out = reinterpret_cast<int16_t*>(pcmBytes.data());
        for (size_t i = 0; i < count; ++i) {
            float s = std::clamp(data[i], -1.0f, 1.0f);
            out[i] = static_cast<int16_t>(s * 32767.0f);
        }
    } else if (targetBitDepth == 24) {
        pcmBytes.resize(count * 3);
        uint8_t* out = pcmBytes.data();
        for (size_t i = 0; i < count; ++i) {
            float s = std::clamp(data[i], -1.0f, 1.0f);
            int32_t val = static_cast<int32_t>(s * 8388607.0f);
            out[i * 3 + 0] = static_cast<uint8_t>(val & 0xFF);
            out[i * 3 + 1] = static_cast<uint8_t>((val >> 8) & 0xFF);
            out[i * 3 + 2] = static_cast<uint8_t>((val >> 16) & 0xFF);
        }
    } else { // 32-bit
        pcmBytes.resize(count * 4);
        int32_t* out = reinterpret_cast<int32_t*>(pcmBytes.data());
        for (size_t i = 0; i < count; ++i) {
            float s = std::clamp(data[i], -1.0f, 1.0f);
            out[i] = static_cast<int32_t>(s * 2147483647.0);
        }
    }

    return mRingBuffer.write(pcmBytes.data(), pcmBytes.size());
}

size_t UsbIsochronousStreamer::writeDsdDoP(const uint8_t* dsdL, const uint8_t* dsdR, size_t sampleCount) {
    if (!dsdL || !dsdR || sampleCount == 0) return 0;

    mAudioDataDetected.store(true, std::memory_order_relaxed);

    // DoP 1.1 framing: 16-bit DSD per channel packaged into 24-bit PCM carrier
    // Marker alternates between 0x05 and 0xFA every 16 DSD clock cycles
    bool is32Bit = (mBitDepth == 32);
    size_t bytesPerFrame = is32Bit ? 8 : 6;
    std::vector<uint8_t> dopBytes(sampleCount * bytesPerFrame);
    uint8_t* out = dopBytes.data();

    for (size_t i = 0; i < sampleCount; ++i) {
        uint8_t marker = mDopMarker;
        // Toggle marker for next frame
        mDopMarker = (mDopMarker == 0x05) ? 0xFA : 0x05;

        uint8_t dsdL0 = dsdL[i * 2 + 0];
        uint8_t dsdL1 = dsdL[i * 2 + 1];
        uint8_t dsdR0 = dsdR[i * 2 + 0];
        uint8_t dsdR1 = dsdR[i * 2 + 1];

        if (!is32Bit) {
            // 24-bit packed: [DSD_low, DSD_high, Marker]
            out[i * 6 + 0] = dsdL0;
            out[i * 6 + 1] = dsdL1;
            out[i * 6 + 2] = marker;

            out[i * 6 + 3] = dsdR0;
            out[i * 6 + 4] = dsdR1;
            out[i * 6 + 5] = marker;
        } else {
            // 32-bit zero-padded: [0x00, DSD_low, DSD_high, Marker]
            out[i * 8 + 0] = 0x00;
            out[i * 8 + 1] = dsdL0;
            out[i * 8 + 2] = dsdL1;
            out[i * 8 + 3] = marker;

            out[i * 8 + 4] = 0x00;
            out[i * 8 + 5] = dsdR0;
            out[i * 8 + 6] = dsdR1;
            out[i * 8 + 7] = marker;
        }
    }

    return mRingBuffer.write(dopBytes.data(), dopBytes.size());
}

size_t UsbIsochronousStreamer::writeNativeDsd(const uint8_t* dsdData, size_t size) {
    if (!dsdData || size == 0) return 0;
    bool hasNonZero = false;
    for (size_t i = 0; i < size; ++i) {
        if (dsdData[i] != 0x00 && dsdData[i] != 0x69 && dsdData[i] != 0x96) {
            hasNonZero = true;
            break;
        }
    }
    if (hasNonZero) {
        mAudioDataDetected.store(true, std::memory_order_relaxed);
    }
    return mRingBuffer.write(dsdData, size);
}

void UsbIsochronousStreamer::streamingThreadLoop() {

    // Elevate thread priority to real-time audio
    pthread_setname_np(pthread_self(), "AFE_UsbIsoLoop");

    int inFlightUrbs = 0;

    // Pre-populate and submit initial batch of URBs
    for (int i = 0; i < NUM_ISO_URBS; ++i) {
        struct usbdevfs_urb* urb = mUrbs[i];
        uint8_t* buffer = mUrbBuffers[i];

        std::memset(urb, 0, sizeof(struct usbdevfs_urb) + sizeof(struct usbdevfs_iso_packet_desc) * PACKETS_PER_URB);
        urb->type = USBDEVFS_URB_TYPE_ISO;
        urb->flags = USBDEVFS_URB_ISO_ASAP;
        urb->endpoint = mEndpointAddress;
        urb->buffer = buffer;
        urb->buffer_length = static_cast<int>(mUrbBufferSize);
        urb->number_of_packets = PACKETS_PER_URB;

        for (int p = 0; p < PACKETS_PER_URB; ++p) {
            urb->iso_frame_desc[p].length = static_cast<unsigned int>(mBytesPerPacket);
        }

        // Fill payload from ring buffer
        size_t readCount = mRingBuffer.read(buffer, mUrbBufferSize);
        if (readCount < mUrbBufferSize) {
            std::memset(buffer + readCount, 0, mUrbBufferSize - readCount);
        }

        int ret = ioctl(mFd, USBDEVFS_SUBMITURB, urb);
        if (ret == 0) {
            inFlightUrbs++;
            mTransfersSubmitted.fetch_add(1, std::memory_order_relaxed);
        } else {
            LOGW("Failed initial USB isochronous URB submission (ep 0x%02x, len %zu): %s",
                 mEndpointAddress, mUrbBufferSize, strerror(errno));
        }
    }

    while (!mStopRequested.load(std::memory_order_relaxed)) {
        // Reap completed URB
        struct usbdevfs_urb* reapedUrb = nullptr;
        int ret = ioctl(mFd, USBDEVFS_REAPURB, &reapedUrb);
        if (ret < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                usleep(500);
                continue;
            }
            LOGW("USBDEVFS_REAPURB error: %s", strerror(errno));
            break;
        }

        if (!reapedUrb) continue;

        // Check transfer status & increment frames transferred
        if (reapedUrb->status != 0) {
            mXRuns.fetch_add(1, std::memory_order_relaxed);
        } else {
            mTransfersCompleted.fetch_add(1, std::memory_order_relaxed);
            size_t bytesTransferred = 0;
            for (int p = 0; p < reapedUrb->number_of_packets; ++p) {
                if (reapedUrb->iso_frame_desc[p].status == 0) {
                    if (reapedUrb->iso_frame_desc[p].actual_length > 0) {
                        bytesTransferred += reapedUrb->iso_frame_desc[p].actual_length;
                    } else {
                        bytesTransferred += reapedUrb->iso_frame_desc[p].length;
                    }
                }
            }
            if (bytesTransferred == 0) {
                bytesTransferred = reapedUrb->buffer_length;
            }
            size_t framesInUrb = (mBytesPerFrame > 0) ? (bytesTransferred / mBytesPerFrame) : 0;
            mFramesStreamed.fetch_add(framesInUrb, std::memory_order_relaxed);
        }

        // Refill reaped URB from ring buffer and re-submit
        uint8_t* buf = static_cast<uint8_t*>(reapedUrb->buffer);
        size_t bytesRead = mRingBuffer.read(buf, mUrbBufferSize);
        if (bytesRead < mUrbBufferSize) {
            std::memset(buf + bytesRead, 0, mUrbBufferSize - bytesRead);
            mXRuns.fetch_add(1, std::memory_order_relaxed);
        }

        reapedUrb->status = 0;
        reapedUrb->flags = USBDEVFS_URB_ISO_ASAP;
        reapedUrb->actual_length = 0;
        for (int p = 0; p < PACKETS_PER_URB; ++p) {
            reapedUrb->iso_frame_desc[p].length = static_cast<unsigned int>(mBytesPerPacket);
            reapedUrb->iso_frame_desc[p].actual_length = 0;
            reapedUrb->iso_frame_desc[p].status = 0;
        }

        if (!mStopRequested.load(std::memory_order_relaxed)) {
            if (ioctl(mFd, USBDEVFS_SUBMITURB, reapedUrb) < 0) {
                LOGW("Failed re-submitting USB isochronous URB: %s", strerror(errno));
            } else {
                mTransfersSubmitted.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
}

// Global active USB streamer instance
static UsbIsochronousStreamer gUsbStreamer;

// JNI Implementation for NativeUsbAudioDriver
extern "C" {

JNIEXPORT jstring JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeParseDescriptors(
    JNIEnv* env,
    jobject /* this */,
    jint fd
) {
    UsbDacCapabilities caps = UsbDescriptorParser::parseDevice(fd);
    std::string summary = "UAC" + std::to_string(caps.version == UacVersion::UAC2 ? 2 : 1) + ";Rates:";
    for (size_t i = 0; i < caps.allSupportedSampleRates.size(); ++i) {
        summary += std::to_string(caps.allSupportedSampleRates[i]);
        if (i + 1 < caps.allSupportedSampleRates.size()) summary += ",";
    }
    summary += ";Bits:";
    for (size_t i = 0; i < caps.allSupportedBitDepths.size(); ++i) {
        summary += std::to_string(caps.allSupportedBitDepths[i]);
        if (i + 1 < caps.allSupportedBitDepths.size()) summary += ",";
    }
    summary += ";StreamingIfaces:" + std::to_string(caps.streamingInterfaces.size());
    if (!caps.streamingInterfaces.empty()) {
        const auto& streamIface = caps.streamingInterfaces[0];
        summary += ";Endpoint:" + std::to_string(streamIface.endpoint.endpointAddress);
        summary += ";Iface:" + std::to_string(streamIface.interfaceNumber);
        summary += ";Alt:" + std::to_string(streamIface.altSetting > 0 ? streamIface.altSetting : 1);
    }
    summary += ";ClockId:" + std::to_string(caps.clockSourceId > 0 ? caps.clockSourceId : 1);
    return env->NewStringUTF(summary.c_str());
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeStartStreaming(
    JNIEnv* /* env */,
    jobject /* this */,
    jint fd,
    jint interfaceNumber,
    jint altSetting,
    jint endpointAddress,
    jint sampleRate,
    jint bitDepth,
    jint channels
) {
    return gUsbStreamer.start(
        fd,
        interfaceNumber,
        altSetting,
        static_cast<uint8_t>(endpointAddress),
        static_cast<uint32_t>(sampleRate),
        static_cast<uint8_t>(bitDepth),
        static_cast<uint8_t>(channels)
    );
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeWritePcm(
    JNIEnv* env,
    jobject /* this */,
    jbyteArray data,
    jint offset,
    jint length
) {
    if (!data || length <= 0) return 0;
    jbyte* bytes = env->GetByteArrayElements(data, nullptr);
    if (!bytes) return 0;

    size_t written = gUsbStreamer.writePcmBytes(reinterpret_cast<const uint8_t*>(bytes + offset), length);
    env->ReleaseByteArrayElements(data, bytes, JNI_ABORT);
    return static_cast<jint>(written);
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeWritePcm16(
    JNIEnv* env,
    jobject /* this */,
    jbyteArray data,
    jint offset,
    jint length
) {
    if (!data || length <= 1) return 0;
    jbyte* bytes = env->GetByteArrayElements(data, nullptr);
    if (!bytes) return 0;

    size_t sampleCount = static_cast<size_t>(length / 2);
    size_t written = gUsbStreamer.writePcm16(reinterpret_cast<const int16_t*>(bytes + offset), sampleCount);
    env->ReleaseByteArrayElements(data, bytes, JNI_ABORT);
    return static_cast<jint>(written);
}

JNIEXPORT jlong JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeGetTransfersSubmitted(
    JNIEnv* /* env */,
    jobject /* this */
) {
    return static_cast<jlong>(gUsbStreamer.getTransfersSubmitted());
}

JNIEXPORT jlong JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeGetTransfersCompleted(
    JNIEnv* /* env */,
    jobject /* this */
) {
    return static_cast<jlong>(gUsbStreamer.getTransfersCompleted());
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeWritePcmFloat(
    JNIEnv* env,
    jobject /* this */,
    jfloatArray data,
    jint offset,
    jint length,
    jint targetBitDepth
) {
    if (!data || length <= 0) return 0;
    jfloat* floats = env->GetFloatArrayElements(data, nullptr);
    if (!floats) return 0;

    size_t written = gUsbStreamer.writePcmFloats(floats + offset, length, static_cast<uint8_t>(targetBitDepth));
    env->ReleaseFloatArrayElements(data, floats, JNI_ABORT);
    return static_cast<jint>(written);
}

JNIEXPORT void JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeStopStreaming(
    JNIEnv* /* env */,
    jobject /* this */
) {
    gUsbStreamer.stop();
}

JNIEXPORT jlong JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeGetFramesStreamed(
    JNIEnv* /* env */,
    jobject /* this */
) {
    return static_cast<jlong>(gUsbStreamer.getFramesStreamed());
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeGetXRuns(
    JNIEnv* /* env */,
    jobject /* this */
) {
    return static_cast<jint>(gUsbStreamer.getXRunCount());
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeIsStreaming(
    JNIEnv* /* env */,
    jobject /* this */
) {
    return static_cast<jboolean>(gUsbStreamer.isStreaming());
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeWriteDoP(
    JNIEnv* env,
    jobject /* this */,
    jbyteArray dsdLeft,
    jbyteArray dsdRight,
    jint sampleCount
) {
    if (!dsdLeft || !dsdRight || sampleCount <= 0) return 0;
    jbyte* l = env->GetByteArrayElements(dsdLeft, nullptr);
    jbyte* r = env->GetByteArrayElements(dsdRight, nullptr);
    if (!l || !r) {
        if (l) env->ReleaseByteArrayElements(dsdLeft, l, JNI_ABORT);
        if (r) env->ReleaseByteArrayElements(dsdRight, r, JNI_ABORT);
        return 0;
    }

    size_t written = gUsbStreamer.writeDsdDoP(
        reinterpret_cast<const uint8_t*>(l),
        reinterpret_cast<const uint8_t*>(r),
        static_cast<size_t>(sampleCount)
    );

    env->ReleaseByteArrayElements(dsdLeft, l, JNI_ABORT);
    env->ReleaseByteArrayElements(dsdRight, r, JNI_ABORT);
    return static_cast<jint>(written);
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeWriteNativeDsd(
    JNIEnv* env,
    jobject /* this */,
    jbyteArray dsdData,
    jint offset,
    jint length
) {
    if (!dsdData || length <= 0) return 0;
    jbyte* data = env->GetByteArrayElements(dsdData, nullptr);
    if (!data) return 0;

    size_t written = gUsbStreamer.writeNativeDsd(
        reinterpret_cast<const uint8_t*>(data + offset),
        static_cast<size_t>(length)
    );

    env->ReleaseByteArrayElements(dsdData, data, JNI_ABORT);
    return static_cast<jint>(written);
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeSetUac1SampleRate(
    JNIEnv* /* env */,
    jobject /* this */,
    jint fd,
    jint endpointAddress,
    jint sampleRate
) {
    return static_cast<jboolean>(UsbDescriptorParser::setUac1SampleRate(
        fd,
        static_cast<uint8_t>(endpointAddress),
        static_cast<uint32_t>(sampleRate)
    ));
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeSetUac2SampleRate(
    JNIEnv* /* env */,
    jobject /* this */,
    jint fd,
    jint clockId,
    jint sampleRate
) {
    return static_cast<jboolean>(UsbDescriptorParser::setUac2SampleRate(
        fd,
        clockId,
        static_cast<uint32_t>(sampleRate)
    ));
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeClaimInterface(
    JNIEnv* /* env */,
    jobject /* this */,
    jint fd,
    jint interfaceNumber
) {
    return static_cast<jboolean>(UsbDescriptorParser::claimInterface(fd, interfaceNumber));
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeReleaseInterface(
    JNIEnv* /* env */,
    jobject /* this */,
    jint fd,
    jint interfaceNumber
) {
    return static_cast<jboolean>(UsbDescriptorParser::releaseInterface(fd, interfaceNumber));
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeGetActiveSampleRate(
    JNIEnv* /* env */,
    jobject /* this */
) {
    return static_cast<jint>(gUsbStreamer.getSampleRate());
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeGetActiveBitDepth(
    JNIEnv* /* env */,
    jobject /* this */
) {
    return static_cast<jint>(gUsbStreamer.getBitDepth());
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeIsAudioDataDetected(
    JNIEnv* /* env */,
    jobject /* this */
) {
    return static_cast<jboolean>(gUsbStreamer.isAudioDataDetected());
}

JNIEXPORT jfloat JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeGetPeakAudioLevel(
    JNIEnv* /* env */,
    jobject /* this */
) {
    return static_cast<jfloat>(gUsbStreamer.getPeakAudioLevel());
}

JNIEXPORT void JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeResetAudioDetection(
    JNIEnv* /* env */,
    jobject /* this */
) {
    gUsbStreamer.resetAudioDataDetection();
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeReadMuteState(
    JNIEnv* /* env */,
    jobject /* this */,
    jint fd,
    jint ctrlIface,
    jint unitId,
    jint channel,
    jint uacVersion
) {
    UacVersion ver = (uacVersion == 1) ? UacVersion::UAC1 : UacVersion::UAC2;
    UacMuteState state = UsbControlInspector::readMuteState(fd, ctrlIface, unitId, channel, ver);
    return static_cast<jint>(state);
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeSafeUnmute(
    JNIEnv* /* env */,
    jobject /* this */,
    jint fd,
    jint ctrlIface,
    jint unitId,
    jint channel,
    jint uacVersion
) {
    UacVersion ver = (uacVersion == 1) ? UacVersion::UAC1 : UacVersion::UAC2;
    return static_cast<jboolean>(UsbControlInspector::safeUnmute(fd, ctrlIface, unitId, channel, ver));
}

JNIEXPORT jfloatArray JNICALL
Java_com_example_audio_NativeUsbAudioDriver_nativeReadVolume(
    JNIEnv* env,
    jobject /* this */,
    jint fd,
    jint ctrlIface,
    jint unitId,
    jint channel,
    jint uacVersion
) {
    UacVersion ver = (uacVersion == 1) ? UacVersion::UAC1 : UacVersion::UAC2;
    UsbVolumeInfo info = UsbControlInspector::readVolume(fd, ctrlIface, unitId, channel, ver);

    jfloatArray result = env->NewFloatArray(6);
    if (!result) return nullptr;

    jfloat elements[6];
    elements[0] = static_cast<jfloat>(info.state);
    elements[1] = static_cast<jfloat>(info.currentDb);
    elements[2] = static_cast<jfloat>(info.minDb);
    elements[3] = static_cast<jfloat>(info.maxDb);
    elements[4] = static_cast<jfloat>(info.resDb);
    elements[5] = static_cast<jfloat>(info.rawCur);

    env->SetFloatArrayRegion(result, 0, 6, elements);
    return result;
}

} // extern "C"

// UsbControlInspector implementation
UacMuteState UsbControlInspector::readMuteState(int fd, int ctrlIface, int unitId, int channel, UacVersion version) {
    if (fd < 0) return UacMuteState::UNKNOWN;

    uint8_t data[1] = {0};
    uint8_t reqType = USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
    uint8_t request = (version == UacVersion::UAC1) ? 0x81 : 0x01; // UAC1 GET_CUR = 0x81, UAC2 CUR = 0x01
    uint16_t value = (0x01 << 8) | (channel & 0xFF); // FU_MUTE_CONTROL = 0x01
    uint16_t index = (unitId << 8) | (ctrlIface & 0xFF);

    bool ok = usbControlTransfer(fd, reqType, request, value, index, data, 1);
    if (!ok) {
        return UacMuteState::UNSUPPORTED;
    }
    return (data[0] != 0) ? UacMuteState::MUTED : UacMuteState::UNMUTED;
}

bool UsbControlInspector::safeUnmute(int fd, int ctrlIface, int unitId, int channel, UacVersion version) {
    if (fd < 0) return false;

    uint8_t data[1] = {0x00}; // 0 = unmuted
    uint8_t reqType = USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
    uint8_t request = 0x01; // SET_CUR / CUR = 0x01
    uint16_t value = (0x01 << 8) | (channel & 0xFF);
    uint16_t index = (unitId << 8) | (ctrlIface & 0xFF);

    bool ok = usbControlTransfer(fd, reqType, request, value, index, data, 1);
    LOGI("safeUnmute result for unit %d ch %d: %s", unitId, channel, ok ? "SUCCESS" : "FAILED");
    return ok;
}

UsbVolumeInfo UsbControlInspector::readVolume(int fd, int ctrlIface, int unitId, int channel, UacVersion version) {
    UsbVolumeInfo info;
    info.state = UacVolumeState::UNKNOWN;
    if (fd < 0) return info;

    uint8_t curData[2] = {0};
    uint8_t reqType = USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
    uint8_t curReq = (version == UacVersion::UAC1) ? 0x81 : 0x01; // GET_CUR
    uint16_t value = (0x02 << 8) | (channel & 0xFF); // FU_VOLUME_CONTROL = 0x02
    uint16_t index = (unitId << 8) | (ctrlIface & 0xFF);

    bool ok = usbControlTransfer(fd, reqType, curReq, value, index, curData, 2);
    if (!ok) {
        info.state = UacVolumeState::UNSUPPORTED;
        return info;
    }

    int16_t rawCur = static_cast<int16_t>(curData[0] | (curData[1] << 8));
    info.state = UacVolumeState::REPORTED;
    info.rawCur = rawCur;
    info.currentDb = rawCur / 256.0f;

    // Optional range query (GET_MIN, GET_MAX, GET_RES for UAC1, RANGE for UAC2)
    if (version == UacVersion::UAC1) {
        uint8_t minData[2] = {0};
        uint8_t maxData[2] = {0};
        uint8_t resData[2] = {0};
        if (usbControlTransfer(fd, reqType, 0x82 /* GET_MIN */, value, index, minData, 2)) {
            info.minDb = static_cast<int16_t>(minData[0] | (minData[1] << 8)) / 256.0f;
        }
        if (usbControlTransfer(fd, reqType, 0x83 /* GET_MAX */, value, index, maxData, 2)) {
            info.maxDb = static_cast<int16_t>(maxData[0] | (maxData[1] << 8)) / 256.0f;
        }
        if (usbControlTransfer(fd, reqType, 0x84 /* GET_RES */, value, index, resData, 2)) {
            info.resDb = static_cast<int16_t>(resData[0] | (resData[1] << 8)) / 256.0f;
        }
    }
    return info;
}


