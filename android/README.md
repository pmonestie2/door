# Cat Door Camera for Android

Native Java app for Android 5.1 (API 22) and newer, including the Nexus 6 on
Android 5.1–7.1. It uses Android's built-in camera and HTTP APIs, with no third-party runtime
libraries, Google account requirement, or rooting. The iPhone app remains available.

## Install on the phone

Copy `cat-door-camera.apk` from this directory to the phone and open it. On older
Android, allow **Settings → Security → Unknown sources** to install your own APK;
turn that setting back off afterward. On Android 8+, installation permission is
specific to the browser or file manager opening the APK. Alternatively, enable
USB debugging, connect by USB, authorize the Mac on the phone, and run:

```bash
~/Documents/code/android-tools/sdk/platform-tools/adb install -r cat-door-camera.apk
```

Allow Camera permission if asked. This app does not require Photos, storage,
microphone, or Google Photos permissions. It does not bypass an existing phone
PIN or Google account verification after factory reset.

## Use

1. Connect the phone to the Pi's Wi-Fi network and open Cat Door Camera.
2. Enter `http://<PI-IP>:8080`. Default: `http://pi3:8080`. If the phone cannot resolve `pi3`, use the Pi's numerical LAN IP instead.
3. Adjust the yellow crop rectangle over the live preview.
4. Leave the switch off for **Push** and choose an interval (1, 2, 5, 10, 30, or
   60 seconds), or enable **Pull** to
   respond to the Pi. Tap **Start**. Tap **Stop** before changing settings.
5. In Pull mode, click **Take photo** on the Pi website or run:
   `python3 /home/pi/door/pull_photo.py`.

After **Start**, capture runs in a foreground service with an ongoing notification.
The screen can sleep, the phone can be locked, and you can leave the app without
stopping Push/Pull capture. **Stop** in the app or notification releases the camera,
light, CPU wake lock, and Wi-Fi lock. Reopening the app shows the current run;
it does not start a second one. The crop preview is available while stopped; during
a run, the screen displays the latest captured crop.

On Android 6+, tap **Allow screen-off operation** once and approve the battery
optimization exemption. Without it, long idle/Doze periods may suspend network
access even though a foreground service is running. Keep the phone powered for
extended collection. Force-stop, reboot, or an Android process kill still ends
capture; open the app and tap Start again. The app does not silently restart the
camera after those events. All successful uploads are retained by
the Pi under `door/camera_images/`; the phone keeps only an in-memory thumbnail.
Nothing is added to the phone photo library or cloud backup. Settings persist.

Capture uses a full-resolution camera JPEG, not the low-resolution preview. The
app chooses the largest photo size matching the preview's aspect ratio, crops
at sensor resolution, and then limits the crop's longest side to 2560 pixels.
JPEG quality starts at 95%; it is reduced only when needed to fit the Pi's 2 MiB
upload limit. Extremely detailed scenes that still exceed the limit are reduced
in size. The upload status displays the actual output dimensions. Small crops
cannot contain more detail than the corresponding part of the sensor.

Silent still capture must be supported by the camera; otherwise the app shows
an error rather than making shutter noises. Photo capture restarts the offscreen
preview afterward, so it works with the screen locked. Check focus, crop alignment,
and orientation on the physical phone after updating.

Push skips an interval if an upload is still running. Pull uses the same waiting
HTTP connection and request IDs as iOS; there are no Pi changes required. Network
failures are shown on screen. Pull reconnects after a short delay. The app does
not operate the door or run obstruction inference.

## Build on this Mac

The Gradle launcher is deliberately outside the repository at
`~/Documents/code/gradlew`, with its companion files under
`~/Documents/code/gradle/wrapper/`. Java and the Android SDK are under
`~/Documents/code/android-tools/`. No system Java installation is required.

```bash
export JAVA_HOME="$HOME/Documents/code/android-tools/jdk/jdk-17.0.20.1+1/Contents/Home"
export ANDROID_HOME="$HOME/Documents/code/android-tools/sdk"
export GRADLE_USER_HOME="$HOME/Documents/code/android-tools/gradle-cache"
~/Documents/code/gradlew -p /Users/pierre/Documents/PI3/door/android assembleDebug
```

Output: `app/build/outputs/apk/debug/app-debug.apk`. The debug signing key is
created by the Android build tools on the Mac; keep it to update the installed
app without uninstalling. No app store or paid developer account is needed.
The project targets API 35 but keeps API 22 as its minimum, so the same APK
can run on an older phone. This is a directly installed personal app, not a
Google Play release. To use Android Studio instead,
open this directory and configure Gradle 8.9, Java 17, and Android SDK 35.

## Build verification

The debug APK was built successfully with Gradle 8.9 and Java 17. Android lint
completed with no errors (remaining warnings concern fixed portrait orientation,
English-only UI strings, and SDK metadata). APK signature verification passed;
its declared minimum is API 22 (Android 5.1). Physical camera orientation, image
quality, permissions, and Pi connectivity still need checking on your phone.

The project directory is mounted from the Pi. If Gradle stalls on that filesystem,
copy `settings.gradle`, `build.gradle`, `gradle.properties`, and the `app` sources
to a local directory such as `/tmp/catdoor-android-build`, and use that directory
with `-p` instead. This APK was built that way and copied back here.

## Overnight light

The rear camera LED turns on only while taking a photo from **18:00 through
07:59**, using phone local time. Version 1.5 gives the lit preview **1.5 seconds**
to settle its exposure before taking the photo, then keeps the light on until the
JPEG arrives. It turns off before
processing or uploading, and stays off while waiting for Pull requests or between
Push captures. Stop, capture errors, and capture timeout also turn it off. This
works with the screen off. Door position does not control the light; a requested
nighttime photo can use it whether the door is raised or lowered.

Version 1.4 fixes the idle torch remaining on overnight.

Version 1.1 changes the default server to `http://pi3:8080` and migrates the old
`http://pi3.local:8080` default. Other saved server addresses are preserved.

## Version 1.2: screen-off operation

Capture and uploads now live in `CameraService`, with an offscreen preview surface
and fresh sensor-frame callbacks. They do not depend on a visible TextureView.
The app requests a camera foreground service, a partial CPU wake lock, a Wi-Fi lock,
and (on newer Android) notification permission. No Photos/storage permissions are
added. The user can request a battery optimization exemption for reliable delivery
while idle. Background camera access on newer Android requires starting the run
from the visible app, which the Start button does.

## Version 1.3: photo quality and Push intervals

Full-resolution still capture replaces preview-frame uploads. The same saved crop
is applied before resizing. Push intervals are selectable and saved between runs;
10 seconds remains the default. Stop the run to change the interval. A slow photo
or upload skips intervening timer ticks instead of overlapping or queuing work.
Pull mode remains exclusively controlled by Pi capture requests.
