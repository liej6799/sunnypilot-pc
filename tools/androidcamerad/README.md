# Android camerad

This app captures the Android rear camera with Camera2 and writes NV12 frames directly into the buffers allocated by Sunnypilot's `VisionIpcServer`. The Linux process remains the `camerad` service from the perspective of `modeld`, the UI, and logging.

Only `VISION_STREAM_NARROW_ROAD` is advertised. This is intentional: `modeld` accepts a narrow-only camera service, while publishing duplicate wide or cabin streams would give downstream processes incorrect camera geometry.

## Data path

1. Linux `camerad` creates 18 Venus-aligned VisionIPC buffers.
2. It passes their FDs once over a pathname `AF_UNIX` `SOCK_SEQPACKET` socket using `SCM_RIGHTS`.
3. Android maps those exact buffers and copies Camera2 `YUV_420_888` planes into NV12.
4. Android sends only the slot, frame ID, and monotonic timestamp for each frame.
5. Linux publishes that slot through VisionIPC and sends `narrowRoadCameraState`.

There is no per-frame socket payload and no Linux-side image copy. The one unavoidable image copy converts Camera2's device-dependent YUV plane layout into the aligned NV12 layout consumed by openpilot.

## Build

From the Sunnypilot checkout:

```sh
./tools/androidcamerad/gradlew -p tools/androidcamerad assembleDebug
scons openpilot/system/camerad/camerad
```

The APK is written to `tools/androidcamerad/app/build/outputs/apk/debug/app-debug.apk`.

## Android setup

Install and launch the app once to grant camera access and create its IPC directory:

```sh
adb install -r tools/androidcamerad/app/build/outputs/apk/debug/app-debug.apk
adb shell am start -n ai.sunnypilot.camerabridge/.MainActivity
```

The default capture request is 1928x1208. The app selects the supported `YUV_420_888` size nearest that request and negotiates the selected size with Linux. Optional launch extras are available for testing:

```sh
adb shell am start -n ai.sunnypilot.camerabridge/.MainActivity \
  --es camera_id 0 --ei width 1920 --ei height 1080 --ei buffer_count 18
```

Frames are published at 20 FPS, matching Sunnypilot's native camera and camera-state frequency.

The foreground service holds a partial wake lock so capture and IPC continue with the display off. While the activity is open it also keeps the display awake.

## LXC mount

Bind the app-owned IPC directory into the container at `/run/android-camerad`:

```text
/data/user/0/ai.sunnypilot.camerabridge/files/ipc -> /run/android-camerad
```

The exact mount syntax depends on the Android LXC/Droidspaces launcher. The source directory must already exist, so launch the app once before starting the container. Do not use an abstract Unix socket: Android and the container have separate network namespaces.

For the Infinix Droidspaces target, start the container with:

```sh
su -c '/data/local/Droidspaces/bin/droidspaces \
  --config /data/local/Droidspaces/Containers/infinix-lxc-openpilot/container.config \
  --bind=/data/user/0/ai.sunnypilot.camerabridge/files/ipc:/run/android-camerad start'
```

Droidspaces persists this as `bind_mounts=` in `container.config`, so later starts use the same mount automatically.

The bridge listens on `/run/android-camerad/camerad.sock`. Override it with `ANDROID_CAMERA_SOCKET` if the container uses another mount point.

If SELinux blocks the connection, label the bind-mounted directory as app data for `ai.sunnypilot.camerabridge` or add the narrow local policy needed for that app domain to connect to the socket. Do not make the entire app domain permissive.

## Runtime

The existing manager entry continues to launch `openpilot/system/camerad/camerad`. On regular Linux/aarch64 builds that binary is the Android bridge; comma hardware still builds the native Spectra implementation.

Start order is not significant. The Android service retries until the Linux socket appears, and Linux accepts a new app connection after an app or camera restart. A capture resolution change requires restarting Linux `camerad` so VisionIPC buffers can be reallocated.

Useful diagnostics:

```sh
adb logcat -s SunnypilotCamera
ls -l /run/android-camerad/camerad.sock
```
