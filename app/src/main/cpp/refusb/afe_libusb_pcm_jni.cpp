// AFE adapter over the reference libusb UAC2 transport (refusb/*).
//
// The reference decodes with FFmpeg inside native code and pumps the ring itself. AFE decodes
// with MediaCodec in Kotlin, so this file adds the two JNI entry points AFE needs on top of the
// reference transport: a float32 -> negotiated-S32 ring writer (adapted from the reference's
// EngineSwapBridge.nativeWriteToRingBuffer) and a ring-fill query used for position tracking.
#include <jni.h>
#include <android/log.h>
#include <array>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <unistd.h>

#include "usb_driver_context.h"
#include "usb_handle_validation.h"
#include "pcm_wire_formatter.h"
#include "audio_gain.h"

#define ATAG "AFE_LibusbPcm"
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, ATAG, __VA_ARGS__)
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, ATAG, __VA_ARGS__)

namespace {
static inline bool is_valid_driver_handle(jlong h) noexcept {
    return is_valid_native_handle(static_cast<int64_t>(h), kUsbDriverLowestErrorSentinel);
}
constexpr jint kErrBadBuffer   = -101;
constexpr jint kErrNotReady    = -102;
constexpr jint kErrSubmit      = -103;
constexpr jint kErrTimeout     = -104;
constexpr std::size_t kChunkFloats = 4096;          // 16 KiB of float32 per conversion pass
constexpr int kBackpressureTimeoutMs = 2000;
constexpr useconds_t kPollUs = 500;

bool hasInFlight(const IsoTransferPool &pool) {
    for (const auto &slot : pool.slots()) {
        if (slot.in_flight.load(std::memory_order_acquire)) return true;
    }
    return false;
}
} // namespace

extern "C" {

/**
 * Writes `count` interleaved float32 samples (range -1..1) to the USB ISO ring after
 * converting them to the exact integer wire format negotiated from the UAC2 descriptors.
 * Starts the ISO pipeline on the first real audio so the DAC never sees an empty-ring burst.
 * @return samples consumed (== count) or a negative kErr* code.
 */
JNIEXPORT jint JNICALL
Java_com_example_audio_LibusbBridge_nativeWriteFloats(
        JNIEnv *env, jclass, jlong handle, jfloatArray data, jint offset, jint count, jfloat volume)
{
    if (!is_valid_driver_handle(handle)) return kErrNotReady;
    if (data == nullptr || count <= 0 || offset < 0 ||
        env->GetArrayLength(data) < offset + count) return kErrBadBuffer;

    auto *ctx = reinterpret_cast<UsbDriverContext *>(handle);
    if (!ctx->ring_buffer || !ctx->transfer_pool || !ctx->event_thread) return kErrNotReady;

    const double gain = ui_position_to_gain(static_cast<double>(volume));
    const PcmWireFormatter formatter(ctx->pcm_wire_format);

    std::array<float, kChunkFloats> floats{};
    std::array<uint8_t, kChunkFloats * sizeof(float)> wire{};

    bool started = hasInFlight(*ctx->transfer_pool);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kBackpressureTimeoutMs);

    jint done = 0;
    while (done < count) {
        const jint n = std::min<jint>(count - done, static_cast<jint>(kChunkFloats));
        env->GetFloatArrayRegion(data, offset + done, n, floats.data());
        const std::size_t bytes = static_cast<std::size_t>(n) * sizeof(float);
        const auto fmt = formatter.format_float32(reinterpret_cast<const uint8_t *>(floats.data()),
                                                  bytes, gain, wire.data(), wire.size());
        if (!fmt.success || fmt.bytes_written != bytes) {
            ALOGE("nativeWriteFloats: formatter rejected chunk");
            return kErrBadBuffer;
        }
        std::size_t off = 0;
        while (off < bytes) {
            const std::size_t freeBytes = ctx->ring_buffer->free_space();
            if (freeBytes == 0) {
                if (std::chrono::steady_clock::now() >= deadline) return kErrTimeout;
                usleep(kPollUs);
                continue;
            }
            const std::size_t push = std::min(freeBytes, bytes - off);
            if (!ctx->ring_buffer->push(wire.data() + off, push)) { usleep(kPollUs); continue; }
            off += push;
            deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kBackpressureTimeoutMs);
            if (!started) {
                if (ctx->transfer_pool->submit_all_transfers() < 1) return kErrSubmit;
                started = true;
                ALOGI("ISO pipeline started after %zu prefill bytes", ctx->ring_buffer->size());
            }
        }
        done += n;
    }
    return done;
}

/** Bytes currently queued in the ring (written by Kotlin, not yet consumed by ISO transfers). */
JNIEXPORT jlong JNICALL
Java_com_example_audio_LibusbBridge_nativeRingQueuedBytes(JNIEnv *, jclass, jlong handle)
{
    if (!is_valid_driver_handle(handle)) return 0;
    auto *ctx = reinterpret_cast<UsbDriverContext *>(handle);
    return ctx->ring_buffer ? static_cast<jlong>(ctx->ring_buffer->size()) : 0;
}

/** Negotiated bytes per interleaved PCM frame on the wire (channels * subslot bytes). */
JNIEXPORT jint JNICALL
Java_com_example_audio_LibusbBridge_nativeWireBytesPerFrame(JNIEnv *, jclass, jlong handle)
{
    if (!is_valid_driver_handle(handle)) return 0;
    auto *ctx = reinterpret_cast<UsbDriverContext *>(handle);
    return static_cast<jint>(ctx->pcm_wire_format.bytes_per_frame());
}

} // extern "C"
