// AFE native AAudio output core.
//
// Requests AAUDIO_SHARING_MODE_EXCLUSIVE + AAUDIO_PERFORMANCE_MODE_LOW_LATENCY. When the device/HAL
// grants an MMAP exclusive stream the app writes straight into the memory-mapped endpoint buffer
// and AudioFlinger's mixer is bypassed; isMMapUsed() reports whether that really happened so the
// UI never claims MMAP when the HAL silently fell back to a legacy/shared stream.
//
// Real-time rules inside the data callback: no allocation, no locks, no JNI.
#include <jni.h>
#include <aaudio/AAudio.h>
#include <android/log.h>
#include <dlfcn.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "lock_free_ring_buffer.h"

#define TAG "AFE_AAudioCore"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

// The library is built for API 26 (first AAudio level); anything newer is runtime-guarded.
#if defined(__ANDROID__) && defined(__clang__)
#define AFE_API_AT_LEAST_28() (__builtin_available(android 28, *))
#else
#define AFE_API_AT_LEAST_28() (true)
#endif

// 512K float samples = 2 MiB. ~2.7 s of stereo @ 96 kHz: rides out main-thread/GC/doze stalls
// (screen-off playback) without an underrun. Must be a power of two.
constexpr size_t RING_BUFFER_CAPACITY = 524288;

struct NativeAudioEngineState {
    AAudioStream* stream = nullptr;
    std::atomic<bool> isRunning{false};
    std::atomic<bool> isTearingDown{false};
    std::atomic<bool> disconnected{false};
    std::atomic<bool> paused{false};
    std::atomic<bool> flushRequested{false};
    std::atomic<int64_t> framesConsumed{0};

    int32_t deviceId = 0;
    int32_t sampleRate = 96000;
    int32_t channelCount = 2;
    int32_t framesPerBurst = 96;
    int32_t configuredBufferSize = 192;

    std::atomic<int32_t> xRunCount{0};
    std::atomic<float> softwareVolume{1.0f};

    LockFreeAudioRingBuffer<float, RING_BUFFER_CAPACITY> ringBuffer;
};

static NativeAudioEngineState gEngine;
static std::mutex gLifecycleMutex;

static aaudio_data_callback_result_t aaudioDataCallback(
        AAudioStream* /*stream*/, void* userData, void* audioData, int32_t numFrames) {
    auto* engine = static_cast<NativeAudioEngineState*>(userData);
    if (!engine || engine->isTearingDown.load(std::memory_order_relaxed)) {
        return AAUDIO_CALLBACK_RESULT_STOP;
    }

    float* output = static_cast<float*>(audioData);
    const size_t channels = static_cast<size_t>(engine->channelCount);
    const size_t requested = static_cast<size_t>(numFrames) * channels;

    // Seek/skip: the producer asks the consumer to drop queued audio.
    if (engine->flushRequested.load(std::memory_order_acquire)) {
        engine->ringBuffer.discardAll();
        engine->flushRequested.store(false, std::memory_order_release);
    }

    if (engine->paused.load(std::memory_order_relaxed)) {
        std::memset(output, 0, requested * sizeof(float));
        return AAUDIO_CALLBACK_RESULT_CONTINUE;
    }

    const size_t samplesRead = engine->ringBuffer.read(output, requested);
    if (samplesRead < requested) {
        std::memset(output + samplesRead, 0, (requested - samplesRead) * sizeof(float));
    }
    if (channels > 0) {
        engine->framesConsumed.fetch_add(static_cast<int64_t>(samplesRead / channels),
                                         std::memory_order_relaxed);
    }

    const float vol = engine->softwareVolume.load(std::memory_order_relaxed);
    if (vol < 0.999f) {
        for (size_t i = 0; i < samplesRead; ++i) output[i] *= vol;
    }
    return AAUDIO_CALLBACK_RESULT_CONTINUE;
}

static void aaudioErrorCallback(AAudioStream* /*stream*/, void* userData, aaudio_result_t error) {
    LOGW("AAudio error callback: %s (%d)", AAudio_convertResultToText(error), error);
    auto* engine = static_cast<NativeAudioEngineState*>(userData);
    if (engine && error == AAUDIO_ERROR_DISCONNECTED) {
        // Do NOT close from this thread (AAudio deadlocks). Flag it; Kotlin reacts and reopens.
        engine->disconnected.store(true, std::memory_order_release);
        engine->isRunning.store(false, std::memory_order_release);
    }
}

static void closeStreamLocked() {
    if (gEngine.stream != nullptr) {
        AAudioStream_requestStop(gEngine.stream);
        aaudio_stream_state_t next = AAUDIO_STREAM_STATE_UNINITIALIZED;
        AAudioStream_waitForStateChange(gEngine.stream, AAUDIO_STREAM_STATE_STOPPING, &next,
                                        100 * 1000000LL);
        AAudioStream_close(gEngine.stream);
        gEngine.stream = nullptr;
    }
    gEngine.isRunning.store(false, std::memory_order_release);
}

static bool openStreamLocked(int32_t deviceId, int32_t sampleRate, int32_t channelCount,
                             int32_t burstMultiplier, bool sessionNone) {
    closeStreamLocked();

    gEngine.isTearingDown.store(false, std::memory_order_relaxed);
    gEngine.disconnected.store(false, std::memory_order_relaxed);
    gEngine.paused.store(false, std::memory_order_relaxed);
    gEngine.flushRequested.store(false, std::memory_order_relaxed);
    gEngine.framesConsumed.store(0, std::memory_order_relaxed);
    gEngine.deviceId = deviceId;
    gEngine.sampleRate = sampleRate;
    gEngine.channelCount = channelCount;
    gEngine.ringBuffer.reset();   // stream is closed: no consumer is running

    AAudioStreamBuilder* builder = nullptr;
    aaudio_result_t result = AAudio_createStreamBuilder(&builder);
    if (result != AAUDIO_OK) {
        LOGE("AAudio_createStreamBuilder failed: %s", AAudio_convertResultToText(result));
        return false;
    }

    if (deviceId > 0) AAudioStreamBuilder_setDeviceId(builder, deviceId);
    AAudioStreamBuilder_setDirection(builder, AAUDIO_DIRECTION_OUTPUT);
    AAudioStreamBuilder_setSharingMode(builder, AAUDIO_SHARING_MODE_EXCLUSIVE);
    AAudioStreamBuilder_setPerformanceMode(builder, AAUDIO_PERFORMANCE_MODE_LOW_LATENCY);
    AAudioStreamBuilder_setSampleRate(builder, sampleRate);
    AAudioStreamBuilder_setChannelCount(builder, channelCount);
    AAudioStreamBuilder_setFormat(builder, AAUDIO_FORMAT_PCM_FLOAT);

    if (AFE_API_AT_LEAST_28()) {
        AAudioStreamBuilder_setUsage(builder, AAUDIO_USAGE_MEDIA);
        AAudioStreamBuilder_setContentType(builder, AAUDIO_CONTENT_TYPE_MUSIC);
        // An AudioTrack-style session id destroys MMAP exclusivity on several Qualcomm/MediaTek HALs.
        AAudioStreamBuilder_setSessionId(builder,
                                         sessionNone ? AAUDIO_SESSION_ID_NONE : AAUDIO_SESSION_ID_ALLOCATE);
    }

    AAudioStreamBuilder_setDataCallback(builder, aaudioDataCallback, &gEngine);
    AAudioStreamBuilder_setErrorCallback(builder, aaudioErrorCallback, &gEngine);

    result = AAudioStreamBuilder_openStream(builder, &gEngine.stream);
    AAudioStreamBuilder_delete(builder);
    if (result != AAUDIO_OK || gEngine.stream == nullptr) {
        LOGE("openStream failed: %s", AAudio_convertResultToText(result));
        gEngine.stream = nullptr;
        return false;
    }

    // The HAL may have changed the layout; the Kotlin side feeds exactly what was requested, so
    // refuse anything else instead of playing at the wrong pitch.
    if (AAudioStream_getChannelCount(gEngine.stream) != channelCount) {
        LOGE("Stream opened with %d channels (wanted %d)", AAudioStream_getChannelCount(gEngine.stream), channelCount);
        closeStreamLocked();
        return false;
    }

    const int32_t actualRate = AAudioStream_getSampleRate(gEngine.stream);
    const bool exclusive = AAudioStream_getSharingMode(gEngine.stream) == AAUDIO_SHARING_MODE_EXCLUSIVE;
    LOGI("Opened: device=%d rate=%d(req %d) ch=%d sharing=%s", deviceId, actualRate, sampleRate,
         channelCount, exclusive ? "EXCLUSIVE" : "SHARED");

    const int32_t burst = std::max<int32_t>(1, AAudioStream_getFramesPerBurst(gEngine.stream));
    const int32_t capacity = AAudioStream_getBufferCapacityInFrames(gEngine.stream);
    gEngine.framesPerBurst = burst;
    int32_t target = burst * std::clamp<int32_t>(burstMultiplier, 1, 16);
    if (capacity > 0 && target > capacity) target = (capacity / burst) * burst;
    if (target < burst) target = burst;
    aaudio_result_t sized = AAudioStream_setBufferSizeInFrames(gEngine.stream, target);
    gEngine.configuredBufferSize = sized > 0 ? sized : target;

    result = AAudioStream_requestStart(gEngine.stream);
    if (result != AAUDIO_OK) {
        LOGE("requestStart failed: %s", AAudio_convertResultToText(result));
        AAudioStream_close(gEngine.stream);
        gEngine.stream = nullptr;
        return false;
    }
    gEngine.isRunning.store(true, std::memory_order_release);
    return true;
}

static bool queryMMap(AAudioStream* stream) {
    // AAudioStream_isMMapUsed is exported by libaaudio but absent from the public NDK header.
    using Fn = bool (*)(AAudioStream*);
    static Fn fn = reinterpret_cast<Fn>(dlsym(RTLD_DEFAULT, "AAudioStream_isMMapUsed"));
    return (fn != nullptr && stream != nullptr) ? fn(stream) : false;
}

extern "C" {

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeAudioEngine_initAAudioStream(
        JNIEnv*, jobject, jint usbRoutingDeviceId, jint sampleRate, jint channelCount, jint /*format*/) {
    std::lock_guard<std::mutex> lock(gLifecycleMutex);
    return openStreamLocked(usbRoutingDeviceId, sampleRate, channelCount, 1, true) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeAudioEngine_initAAudioStreamEx(
        JNIEnv*, jobject, jint usbRoutingDeviceId, jint sampleRate, jint channelCount, jint /*format*/,
        jint bufferBurstMultiplier, jboolean forceSessionIdNone) {
    std::lock_guard<std::mutex> lock(gLifecycleMutex);
    return openStreamLocked(usbRoutingDeviceId, sampleRate, channelCount, bufferBurstMultiplier,
                            forceSessionIdNone == JNI_TRUE) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeAudioEngine_setBufferMultiplier(JNIEnv*, jobject, jint multiplier) {
    std::lock_guard<std::mutex> lock(gLifecycleMutex);
    if (gEngine.stream == nullptr) return 0;
    const int32_t burst = AAudioStream_getFramesPerBurst(gEngine.stream);
    const int32_t capacity = AAudioStream_getBufferCapacityInFrames(gEngine.stream);
    if (burst <= 0) return 0;
    int32_t target = burst * std::clamp<int32_t>(multiplier, 1, 16);
    if (capacity > 0 && target > capacity) target = capacity;
    target = (target / burst) * burst;
    if (target < burst) target = burst;
    const aaudio_result_t res = AAudioStream_setBufferSizeInFrames(gEngine.stream, target);
    gEngine.configuredBufferSize = res > 0 ? res : target;
    return gEngine.configuredBufferSize;
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeAudioEngine_isExclusiveMode(JNIEnv*, jobject) {
    std::lock_guard<std::mutex> lock(gLifecycleMutex);
    if (gEngine.stream == nullptr) return JNI_FALSE;
    return AAudioStream_getSharingMode(gEngine.stream) == AAUDIO_SHARING_MODE_EXCLUSIVE ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeAudioEngine_isMMapUsed(JNIEnv*, jobject) {
    std::lock_guard<std::mutex> lock(gLifecycleMutex);
    return queryMMap(gEngine.stream) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeAudioEngine_getActualSampleRate(JNIEnv*, jobject) {
    std::lock_guard<std::mutex> lock(gLifecycleMutex);
    return gEngine.stream ? AAudioStream_getSampleRate(gEngine.stream) : 0;
}

JNIEXPORT jlong JNICALL
Java_com_example_audio_NativeAudioEngine_getHardwarePresentationTimestampMs(JNIEnv*, jobject) {
    if (!gEngine.isRunning.load(std::memory_order_relaxed) || gEngine.sampleRate <= 0) return -1L;
    return static_cast<jlong>(gEngine.framesConsumed.load(std::memory_order_relaxed) * 1000LL / gEngine.sampleRate);
}

JNIEXPORT void JNICALL
Java_com_example_audio_NativeAudioEngine_instantTearDown(JNIEnv*, jobject) {
    // Safe from any thread, including the USB detach receiver: only flips flags and closes
    // the stream on a worker so the caller never blocks on the audio service.
    if (gEngine.isTearingDown.exchange(true)) return;
    std::thread([]() {
        std::lock_guard<std::mutex> lock(gLifecycleMutex);
        closeStreamLocked();
        gEngine.ringBuffer.reset();
        gEngine.isTearingDown.store(false, std::memory_order_release);
    }).detach();
}

/** Synchronous close (call from a worker thread). Guarantees the stream is gone when it returns. */
JNIEXPORT void JNICALL
Java_com_example_audio_NativeAudioEngine_closeStream(JNIEnv*, jobject) {
    std::lock_guard<std::mutex> lock(gLifecycleMutex);
    closeStreamLocked();
    gEngine.ringBuffer.reset();
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeAudioEngine_writePcmFloats(
        JNIEnv* env, jobject, jfloatArray audioData, jint offset, jint length) {
    if (length <= 0 || offset < 0 || audioData == nullptr ||
        gEngine.isTearingDown.load(std::memory_order_relaxed) ||
        !gEngine.isRunning.load(std::memory_order_relaxed)) {
        return 0;
    }
    if (env->GetArrayLength(audioData) < offset + length) return 0;
    void* raw = env->GetPrimitiveArrayCritical(audioData, nullptr);
    if (!raw) return 0;
    // Non-blocking memcpy into the ring: safe to hold the critical region.
    const size_t written = gEngine.ringBuffer.write(static_cast<const float*>(raw) + offset,
                                                    static_cast<size_t>(length));
    env->ReleasePrimitiveArrayCritical(audioData, raw, JNI_ABORT);
    return static_cast<jint>(written);
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeAudioEngine_getRingWritableSamples(JNIEnv*, jobject) {
    return static_cast<jint>(gEngine.ringBuffer.getAvailableWrite());
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeAudioEngine_getRingQueuedSamples(JNIEnv*, jobject) {
    return static_cast<jint>(gEngine.ringBuffer.getAvailableRead());
}

JNIEXPORT void JNICALL
Java_com_example_audio_NativeAudioEngine_setPaused(JNIEnv*, jobject, jboolean paused) {
    gEngine.paused.store(paused == JNI_TRUE, std::memory_order_release);
}

JNIEXPORT void JNICALL
Java_com_example_audio_NativeAudioEngine_requestFlush(JNIEnv*, jobject) {
    if (gEngine.isRunning.load(std::memory_order_relaxed)) {
        gEngine.flushRequested.store(true, std::memory_order_release);
    }
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeAudioEngine_isFlushPending(JNIEnv*, jobject) {
    return (gEngine.flushRequested.load(std::memory_order_acquire) &&
            gEngine.isRunning.load(std::memory_order_relaxed)) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jlong JNICALL
Java_com_example_audio_NativeAudioEngine_getFramesConsumed(JNIEnv*, jobject) {
    return static_cast<jlong>(gEngine.framesConsumed.load(std::memory_order_relaxed));
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeAudioEngine_isDisconnected(JNIEnv*, jobject) {
    return gEngine.disconnected.load(std::memory_order_acquire) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeAudioEngine_getXRunCount(JNIEnv*, jobject) {
    std::lock_guard<std::mutex> lock(gLifecycleMutex);
    if (gEngine.stream != nullptr) {
        const int32_t x = AAudioStream_getXRunCount(gEngine.stream);
        if (x >= 0) gEngine.xRunCount.store(x, std::memory_order_relaxed);
    }
    return gEngine.xRunCount.load(std::memory_order_relaxed);
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeAudioEngine_getFramesPerBurst(JNIEnv*, jobject) {
    return gEngine.framesPerBurst;
}

JNIEXPORT jint JNICALL
Java_com_example_audio_NativeAudioEngine_getBufferSizeInFrames(JNIEnv*, jobject) {
    return gEngine.configuredBufferSize;
}

JNIEXPORT void JNICALL
Java_com_example_audio_NativeAudioEngine_setSoftwareVolume(JNIEnv*, jobject, jfloat volume) {
    gEngine.softwareVolume.store(std::clamp(volume, 0.0f, 1.0f), std::memory_order_relaxed);
}

JNIEXPORT jboolean JNICALL
Java_com_example_audio_NativeAudioEngine_isStreamActive(JNIEnv*, jobject) {
    return gEngine.isRunning.load(std::memory_order_relaxed) ? JNI_TRUE : JNI_FALSE;
}

} // extern "C"
