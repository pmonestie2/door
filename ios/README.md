# Cat Door Camera for iPhone

A small native app for iOS 15 or newer, including iPhone 13. No jailbreak,
third-party packages, or model on the phone. Uses the back camera, keeps the
screen awake, and sends cropped JPEGs to the Pi in either Push or Pull mode.

## Install

1. Install full Xcode on the Mac and open it once to finish installing iOS support.
2. Open `CatDoorCamera/CatDoorCamera.xcodeproj`.
3. In Xcode Settings → Accounts, sign in with your Apple account.
4. Select the CatDoorCamera target → Signing & Capabilities, select your Team,
   and leave automatic signing enabled. Change the bundle identifier if Xcode
   says it is unavailable.
5. Connect the iPhone to the Mac, trust the Mac, and enable Developer Mode on
   the phone if requested (Settings → Privacy & Security → Developer Mode).
6. Select the iPhone as the run destination and click Run. Allow Camera and
   Local Network access when prompted. No microphone or Photos access is used.

Free Personal Team signing expires after seven days: rebuild/reinstall from
Xcode when it expires. Paid developer signing has different provisioning terms.
The source project has no account or signing credentials embedded in it.

## Use

Deploy the updated `door_web.py` and new `door_camera.py` with the existing
controller modules, then restart the controller. On the phone:

1. Connect to the same Wi-Fi as the Pi.
2. Enter `http://raspberrypi.local:8080` (or the Pi's LAN IP and port).
3. Choose **Push · every 10s** for timed uploads or **Pull · Pi requests** for on-demand captures. Stop before changing modes.
4. The preview starts automatically. The yellow box marks the crop. Adjust it, then tap Start. Inspect **Last captured crop** to
   verify the exact image sent. Stop, adjust size/position, and restart as needed.
5. Keep the phone mounted, powered, and the app visible. The interface stays
   portrait; crop coordinates refer to that portrait view.

Auto-lock is disabled for the entire time the app is in the foreground, even with uploads stopped.
Leaving the app for the background stops capture/uploads and restores normal auto-lock; tap Start to resume uploads.
Calls and other camera interruptions can pause capture. Failed uploads are not
queued: the next scheduled attempt uses a new photo. Slow work skips timer ticks
rather than overlapping captures. The displayed counter counts successful
uploads since the app launched; the thumbnail is the last captured image,
including when its upload failed. Address, crop, and mode persist between launches. In Pull mode, open the Pi website
and click **Take photo (Pull mode)**. The app must be open and started; the site
waits for the matching upload and reports a timeout if it does not arrive.

On iOS 18 and newer, the app suppresses the shutter sound when the device
allows it. Older iOS versions and regions that require the sound retain system
behavior.

The app resizes photos to at most 640 pixels on their longest side and encodes
JPEG at 80% quality. Check whether this preserves narrow tails/paws before using
the images for obstruction detection. It does not save images to the Photos app.

## Pi endpoints and storage

- `POST /camera`: raw JPEG body, `Content-Type: image/jpeg`, and
  `X-Captured-At: 2026-10-05T15:00:00Z`. Pull uploads also include
  `X-Capture-Request-ID`, saved in the image metadata. Returns HTTP 201.
- `POST /camera/capture`: JSON `{}` requests a fresh photo; returns HTTP 202 with
  `request_id`. Returns 409 while another request is outstanding.
- `GET /camera/request?id=<request_id>`: `waiting`, `capturing`, `complete`, or
  `timed_out`. Completed results include matching image metadata, not just the
  latest unrelated photo. Requests expire after thirty seconds; the server keeps
  the newest 100 request results in memory, cleared on restart.
- `GET /camera/command`: the phone's command channel. Holds the HTTP request for
  up to twenty seconds, returns a capture command or HTTP 204, then the phone
  reconnects. No additional packages, ports, or WebSocket server are needed.
- `GET /camera/latest.jpg`: latest received JPEG, also linked from door controls.
- `GET /camera/latest`: JSON filename, phone capture time, and Pi receipt time.

Uploads are limited to 2 MiB and must have JPEG start/end markers; the receiver
checks framing, not full image decoding. Images and timestamp sidecars are saved
under `door/camera_images/`. Keep that directory writable. All uploaded images
and their timestamp sidecars are retained for labeling and training; uploads
never prune older images.

This uses the existing trusted-LAN server with no login. Do not expose it to the
Internet. Local HTTP is enabled through the app's local networking exception.
No uploads go to a cloud service. Pictures are held in memory on the phone and are
never added to Apple Photos or Google Photos. Uploads use an ephemeral network
session without a disk cache; the phone does not keep an image archive. Capture time comes from the phone; receipt time
comes from the Pi, so keep both clocks synchronized.

This version collects images only: no trained model or automatic closure decision
is included. Images are unlabelled; review them before treating them as clear
training examples. Future inference can run on demand against a snapshot and its
timestamps. A ten-second-old image must not be treated as current clearance.

## Validation

The project/plist and Swift syntax can be checked on a Mac, but an actual iOS SDK
build and physical-device run are required to validate camera access, crop
alignment, local networking permissions, uploads, and background/foreground
behavior. The command-line tools alone cannot build this app.

Request a picture from the Pi or another LAN client:

```bash
curl -H 'Content-Type: application/json' -d '{}' http://pi3.local:8080/camera/capture
curl 'http://pi3.local:8080/camera/request?id=REQUEST_ID_FROM_RESPONSE'
```

Pull mode supports one phone and one outstanding capture. If the phone disconnects
or a capture/upload fails, that request times out; request another photo after
reconnecting. Late images remain archived with their request ID but do not turn
expired requests into successful results. No automatic door movements request
pictures yet; use the website button or REST endpoint. Push captures have no
request ID and cannot satisfy a Pull request.

On the Pi, the helper script requests a photo and waits for the matching upload:

```bash
python3 /home/pi/door/pull_photo.py
```

It prints JSON metadata including the filename under `/home/pi/door/camera_images/`
and returns a nonzero exit code if the request fails. Use `--server URL` to target
another server. It does not start the controller or change the phone's mode.
