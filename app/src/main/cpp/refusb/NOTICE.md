# Third-party code in this directory

Imported from the "audiophile-music-player" reference project (MIT License) and adapted for AFE:
usb_*, uac2_*, spsc_ring_buffer, pcm_wire_formatter, libusb_event_thread, cpu_affinity_policy,
dop/native_dsd formatters, dsd_playback_manager, decoder_to_ring_bridge (+ headers).
JNI prefix renamed to com.example.audio.LibusbBridge. `afe_libusb_pcm_jni.cpp` is AFE-specific.

`include/libusb/` headers and `app/src/main/jniLibs/*/libusb1.0.so` are libusb (LGPL-2.1+),
dynamically linked as a separate shared library.
