cmake_minimum_required(VERSION 3.22.1)

project("afe_native" LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Built for API 26 (first level with AAudio). Newer AAudio calls are runtime-guarded and the
# Kotlin side falls back to MediaPlayer if this library cannot be loaded (API 24/25).
string(APPEND CMAKE_CXX_FLAGS_RELEASE " -O3")

# Prebuilt libusb (from the reference project, MIT) - one per ABI under src/main/jniLibs.
add_library(usb1.0 SHARED IMPORTED)
set_target_properties(usb1.0 PROPERTIES
    IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/../jniLibs/${ANDROID_ABI}/libusb1.0.so)

set(REFUSB ${CMAKE_SOURCE_DIR}/refusb)

add_library(
    afe_aaudio_core
    SHARED
    afe_aaudio_core.cpp
    afe_hal_bridge.cpp
    afe_usb_uac.cpp
    # --- reference libusb UAC1/UAC2 isochronous transport (adapted) ---
    ${REFUSB}/afe_libusb_pcm_jni.cpp
    ${REFUSB}/usb_audio_bridge.cpp
    ${REFUSB}/usb_device_controller.cpp
    ${REFUSB}/usb_iso_transfer_pool.cpp
    ${REFUSB}/usb_session_transport.cpp
    ${REFUSB}/usb_teardown.cpp
    ${REFUSB}/usb_producer_coordinator.cpp
    ${REFUSB}/usb_playback_state.cpp
    ${REFUSB}/uac2_clock_control.cpp
    ${REFUSB}/uac2_descriptor_parser.cpp
    ${REFUSB}/uac2_dsd_detector.cpp
    ${REFUSB}/spsc_ring_buffer.cpp
    ${REFUSB}/pcm_wire_formatter.cpp
    ${REFUSB}/libusb_event_thread.cpp
    ${REFUSB}/cpu_affinity_policy.cpp
    ${REFUSB}/dop_formatter.cpp
    ${REFUSB}/native_dsd_formatter.cpp
    ${REFUSB}/dsd_playback_manager.cpp
    ${REFUSB}/decoder_to_ring_bridge.cpp
)

target_include_directories(afe_aaudio_core PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${REFUSB}
    ${REFUSB}/include
)

find_library(log-lib log)
find_library(aaudio-lib aaudio)
find_library(android-lib android)

target_link_libraries(
    afe_aaudio_core
    usb1.0
    ${log-lib}
    ${aaudio-lib}
    ${android-lib}
)
