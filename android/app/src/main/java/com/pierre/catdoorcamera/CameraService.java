package com.pierre.catdoorcamera;

import android.annotation.SuppressLint;
import android.app.*;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ServiceInfo;
import android.graphics.*;
import android.hardware.Camera;
import android.net.wifi.WifiManager;
import android.os.*;
import org.json.JSONObject;
import java.io.ByteArrayOutputStream;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/** Owns capture and networking independently of the visible activity. */
@SuppressWarnings("deprecation")
public final class CameraService extends Service {
    static final String START = "com.pierre.catdoorcamera.START";
    static final String STOP = "com.pierre.catdoorcamera.STOP";
    private static final int NOTIFICATION = 1;
    private final Handler handler = new Handler(Looper.getMainLooper());
    private final ExecutorService worker = Executors.newFixedThreadPool(2);
    private final LocalBinder binder = new LocalBinder();
    private Camera camera;
    private SurfaceTexture surface;
    private PiClient client;
    private PowerManager.WakeLock cpuLock;
    private WifiManager.WifiLock wifiLock;
    private Listener listener;
    private boolean running, busy, pull, awaitingFrame;
    private int generation, captureSequence, width, height, rotation, uploaded;
    private float size, x, y;
    private int intervalMillis = 10000;
    private byte[] lastImage;
    private String status = "Stopped", light = "Camera light off";

    interface Listener {
        void changed(boolean running, String status, String light, byte[] image);
    }

    final class LocalBinder extends Binder {
        /** Returns:
         *     CameraService: The same-process service, without starting capture.
         */
        CameraService getService() { return CameraService.this; }
    }

    /** Returns:
     *     IBinder: A local connection for displaying state and the latest crop.
     */
    @Override public IBinder onBind(Intent intent) { return binder; }

    /** Subscribe only while the activity is visible, avoiding a retained screen. */
    void setListener(Listener listener) { this.listener = listener; notifyScreen(); }

    /** Returns:
     *     boolean: Whether a capture run is active, including while the screen is off.
     */
    boolean isRunning() { return running; }

    /** Start only from an explicit foreground user action; never silently restart after a kill. */
    @Override public int onStartCommand(Intent intent, int flags, int startId) {
        if (intent != null && STOP.equals(intent.getAction())) stopCapture("Stopped");
        else if (intent != null && START.equals(intent.getAction()) && !running) startCapture();
        return START_NOT_STICKY;
    }

    /** Publish an ongoing notification, acquire screen-off locks, and start the chosen mode. */
    @SuppressLint("WakelockTimeout") // Held only during a user-started run; stop and destruction release it.
    private void startCapture() {
        try {
            NotificationManager manager = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
            if (Build.VERSION.SDK_INT >= 26) {
                NotificationChannel channel = new NotificationChannel("camera", "Cat door capture", NotificationManager.IMPORTANCE_LOW);
                manager.createNotificationChannel(channel);
            }
            int flags = PendingIntent.FLAG_UPDATE_CURRENT;
            if (Build.VERSION.SDK_INT >= 23) flags |= PendingIntent.FLAG_IMMUTABLE;
            PendingIntent open = PendingIntent.getActivity(this, 0, new Intent(this, MainActivity.class), flags);
            PendingIntent stop = PendingIntent.getService(this, 1, new Intent(this, CameraService.class).setAction(STOP), flags);
            Notification.Builder builder = Build.VERSION.SDK_INT >= 26
                    ? new Notification.Builder(this, "camera") : new Notification.Builder(this);
            Notification notification = builder.setSmallIcon(R.drawable.camera_icon)
                    .setContentTitle("Cat door camera running")
                    .setContentText("Capture and uploads continue with the screen off")
                    .setContentIntent(open).setOngoing(true)
                    .addAction(android.R.drawable.ic_media_pause, "Stop", stop).build();
            if (Build.VERSION.SDK_INT >= 30) startForeground(NOTIFICATION, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_CAMERA);
            else startForeground(NOTIFICATION, notification);
            PowerManager power = (PowerManager) getSystemService(POWER_SERVICE);
            cpuLock = power.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "CatDoorCamera:Capture");
            cpuLock.acquire();
            WifiManager wifi = (WifiManager) getApplicationContext().getSystemService(WIFI_SERVICE);
            wifiLock = wifi.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "CatDoorCamera:Wifi");
            wifiLock.acquire();
            SharedPreferences settings = getSharedPreferences("camera", MODE_PRIVATE);
            pull = settings.getBoolean("pull", false);
            intervalMillis = Math.max(1, Math.min(60, settings.getInt("intervalSeconds", 10))) * 1000;
            size = settings.getFloat("size", 1); x = settings.getFloat("x", .5f); y = settings.getFloat("y", .5f);
            client = new PiClient(settings.getString("address", "http://pi3:8080").replaceAll("/+$", ""));
            openCamera();
            running = true;
            generation++;
            status = pull ? "Pull: waiting for Pi (screen may sleep)" : "Push: every " + (intervalMillis / 1000) + " seconds (screen may sleep)";
            checkCameraLight();
            if (pull) waitForCommand(generation); else pushTick(generation);
        } catch (Exception error) { stopCapture("Could not start: " + error.getMessage()); }
    }

    /** Cancel network work and release the camera, torch, notification, and power locks. */
    private void stopCapture(String message) {
        running = false;
        generation++;
        busy = awaitingFrame = false;
        handler.removeCallbacksAndMessages(null);
        if (client != null) { client.cancel(); client = null; }
        if (camera != null) { camera.setPreviewCallback(null); camera.release(); camera = null; }
        if (surface != null) { surface.release(); surface = null; }
        if (wifiLock != null && wifiLock.isHeld()) wifiLock.release();
        if (cpuLock != null && cpuLock.isHeld()) cpuLock.release();
        light = "Camera light off";
        status = message;
        stopForeground(true);
        notifyScreen();
        stopSelf();
    }

    /** Release resources even if Android stops the service. */
    @Override public void onDestroy() {
        listener = null;
        stopCapture("Stopped");
        worker.shutdownNow();
        super.onDestroy();
    }

    /** Open a back-camera preview without depending on any Activity or screen surface. */
    private void openCamera() throws Exception {
        Camera.CameraInfo info = new Camera.CameraInfo();
        int selected = -1;
        for (int id = 0; id < Camera.getNumberOfCameras(); id++) {
            Camera.getCameraInfo(id, info);
            if (info.facing == Camera.CameraInfo.CAMERA_FACING_BACK) { selected = id; break; }
        }
        if (selected < 0) throw new Exception("No back camera");
        camera = Camera.open(selected);
        rotation = info.orientation; // Portrait view, matching the activity's crop coordinates.
        Camera.Parameters parameters = camera.getParameters();
        Camera.Size best = null;
        for (Camera.Size candidate : parameters.getSupportedPreviewSizes()) {
            if (best == null || Math.abs(candidate.width * candidate.height - 640 * 480)
                    < Math.abs(best.width * best.height - 640 * 480)) best = candidate;
        }
        width = best.width; height = best.height;
        parameters.setPreviewSize(width, height);
        parameters.setPreviewFormat(ImageFormat.NV21);
        Camera.Size picture = null;
        double previewRatio = (double) width / height;
        for (Camera.Size candidate : parameters.getSupportedPictureSizes()) {
            if (Math.abs((double) candidate.width / candidate.height - previewRatio) < .03
                    && (picture == null || (long) candidate.width * candidate.height > (long) picture.width * picture.height))
                picture = candidate;
        }
        if (picture == null) throw new Exception("No photo size matching the preview crop");
        parameters.setPictureSize(picture.width, picture.height);
        parameters.setJpegQuality(95);
        parameters.setRotation(0); // Keep sensor orientation; rotate the crop ourselves.
        if (!camera.enableShutterSound(false)) throw new Exception("This phone does not allow silent full-resolution photos");
        if (parameters.getSupportedFocusModes().contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE))
            parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE);
        camera.setParameters(parameters);
        surface = new SurfaceTexture(0);
        camera.setPreviewTexture(surface);
        camera.setErrorCallback((error, ignored) -> stopCapture("Camera error " + error + ". Reopen the app and tap Start."));
        camera.startPreview();
    }

    /** Schedule timed captures without overlapping unfinished uploads. */
    private void pushTick(int run) {
        if (!running || generation != run) return;
        if (!busy) capture(null, run);
        handler.postDelayed(() -> pushTick(run), intervalMillis);
    }

    /** Keep a command wait open to the Pi, independently of the screen lifecycle. */
    private void waitForCommand(int run) {
        if (!running || generation != run) return;
        PiClient connection = client;
        worker.execute(() -> {
            try {
                JSONObject command = connection.waitForCommand();
                handler.post(() -> {
                    if (!running || generation != run) return;
                    if (command == null) waitForCommand(run);
                    else if ("capture".equals(command.optString("command")) && command.optString("request_id").matches("[0-9a-f]{32}"))
                        capture(command.optString("request_id"), run);
                    else retry(run, "Unexpected Pi command");
                });
            } catch (Exception error) {
                handler.post(() -> { if (running && generation == run) retry(run, "Pull failed: " + error.getMessage()); });
            }
        });
    }

    /** Delay reconnects after a command-channel failure. */
    private void retry(int run, String message) {
        status = message + ". Retrying…";
        notifyScreen();
        handler.postDelayed(() -> { if (running && generation == run && pull) waitForCommand(run); }, 2000);
    }

    /** Take a full-resolution photo, then restart preview for the next request. */
    private void capture(String requestId, int run) {
        if (!running || generation != run || busy) return;
        updateCameraLight();
        busy = awaitingFrame = true;
        int capture = ++captureSequence;
        try {
            camera.takePicture(null, null, (data, source) -> {
                if (!running || generation != run || captureSequence != capture || !awaitingFrame) return;
                awaitingFrame = false;
                try { source.startPreview(); }
                catch (RuntimeException error) { stopCapture("Camera preview restart failed: " + error.getMessage()); return; }
                SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", Locale.US);
                formatter.setTimeZone(TimeZone.getTimeZone("UTC"));
                String capturedAt = formatter.format(new Date());
                PiClient connection = client;
                final int frameRotation = rotation;
                final float cropSize = size, cropX = x, cropY = y;
                worker.execute(() -> {
                    byte[] jpeg = null;
                    boolean success = false;
                    String result;
                    try {
                        jpeg = encodePhoto(data, frameRotation, cropSize, cropX, cropY);
                        connection.upload(jpeg, capturedAt, requestId);
                        success = true;
                        BitmapFactory.Options dimensions = new BitmapFactory.Options();
                        dimensions.inJustDecodeBounds = true;
                        BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length, dimensions);
                        result = "Photo uploaded: " + dimensions.outWidth + " × " + dimensions.outHeight;
                    } catch (Exception error) { result = "Capture/upload failed: " + error.getMessage(); }
                    final byte[] image = jpeg;
                    final boolean uploadedOK = success;
                    final String message = result;
                    handler.post(() -> {
                        if (!running || generation != run) return;
                        busy = false;
                        if (image != null) lastImage = image;
                        if (uploadedOK) uploaded++;
                        status = message + " · " + uploaded + " uploads";
                        notifyScreen();
                        if (pull) waitForCommand(run);
                    });
                });
            });
        } catch (RuntimeException error) {
            busy = awaitingFrame = false;
            if (pull) retry(run, "Camera capture failed: " + error.getMessage());
            else { status = "Camera capture failed: " + error.getMessage(); notifyScreen(); }
            return;
        }
        handler.postDelayed(() -> {
            if (running && generation == run && captureSequence == capture && awaitingFrame) {
                // A stalled camera requires user attention; do not keep claiming successful capture.
                stopCapture("No fresh camera frames. Reopen the app and tap Start.");
            }
        }, 10000);
    }

    /** Returns:
     *     byte[]: An upright photo cropped at sensor resolution, up to 2560 pixels on its long side.
     */
    private static byte[] encodePhoto(byte[] data, int rotation, float size, float x, float y) throws Exception {
        if (data == null) throw new Exception("Camera returned no JPEG");
        BitmapRegionDecoder decoder = BitmapRegionDecoder.newInstance(data, 0, data.length, false);
        Bitmap cropped;
        try {
            // Map the portrait crop back into sensor coordinates, decoding only that region.
            // This avoids allocating two full 12-megapixel bitmaps on the old phone.
            Matrix toPortrait = new Matrix(); toPortrait.postRotate(rotation);
            RectF bounds = new RectF(0, 0, decoder.getWidth(), decoder.getHeight());
            toPortrait.mapRect(bounds);
            toPortrait.postTranslate(-bounds.left, -bounds.top);
            float left = x * bounds.width() * (1 - size), top = y * bounds.height() * (1 - size);
            RectF region = new RectF(left, top, left + size * bounds.width(), top + size * bounds.height());
            Matrix toSensor = new Matrix();
            if (!toPortrait.invert(toSensor)) throw new Exception("Invalid photo orientation");
            toSensor.mapRect(region);
            Rect pixels = new Rect(); region.roundOut(pixels);
            pixels.intersect(0, 0, decoder.getWidth(), decoder.getHeight());
            BitmapFactory.Options options = new BitmapFactory.Options();
            options.inSampleSize = 1;
            while (Math.max(pixels.width(), pixels.height()) / (options.inSampleSize * 2) >= 2560)
                options.inSampleSize *= 2;
            cropped = decoder.decodeRegion(pixels, options);
        } finally { decoder.recycle(); }
        if (cropped == null) throw new Exception("Invalid camera JPEG");
        Matrix turn = new Matrix(); turn.postRotate(rotation);
        Bitmap upright = Bitmap.createBitmap(cropped, 0, 0, cropped.getWidth(), cropped.getHeight(), turn, true);
        if (upright != cropped) cropped.recycle();
        float scale = Math.min(1f, 2560f / Math.max(upright.getWidth(), upright.getHeight()));
        Bitmap resized = Bitmap.createScaledBitmap(upright, Math.max(1, Math.round(upright.getWidth() * scale)),
                Math.max(1, Math.round(upright.getHeight() * scale)), true);
        if (resized != upright) upright.recycle();
        try {
            ByteArrayOutputStream output = new ByteArrayOutputStream();
            for (int quality = 95; quality >= 75; quality -= 5) {
                output.reset();
                resized.compress(Bitmap.CompressFormat.JPEG, quality, output);
                if (output.size() <= 2 * 1024 * 1024) return output.toByteArray();
            }
            // Extremely detailed scenes can exceed the Pi's 2 MiB limit even at 75%.
            Bitmap smaller = Bitmap.createScaledBitmap(resized, Math.max(1, resized.getWidth() / 2),
                    Math.max(1, resized.getHeight() / 2), true);
            output.reset(); smaller.compress(Bitmap.CompressFormat.JPEG, 95, output); smaller.recycle();
            return output.toByteArray();
        } finally { resized.recycle(); }
    }

    /** Check the light schedule even with the screen off and while awaiting Pull commands. */
    private void checkCameraLight() {
        if (!running) return;
        if (!awaitingFrame) updateCameraLight();
        notifyScreen();
        handler.postDelayed(this::checkCameraLight, 30000);
    }

    /** Apply continuous torch illumination from 18:00 inclusive until 08:00 exclusive. */
    private void updateCameraLight() {
        int hour = Calendar.getInstance().get(Calendar.HOUR_OF_DAY);
        boolean night = hour >= 18 || hour < 8;
        try {
            Camera.Parameters parameters = camera.getParameters();
            List<String> modes = parameters.getSupportedFlashModes();
            String desired = night ? Camera.Parameters.FLASH_MODE_TORCH : Camera.Parameters.FLASH_MODE_OFF;
            if (modes == null || !modes.contains(desired)) { light = "Camera light unavailable"; return; }
            if (!desired.equals(parameters.getFlashMode())) { parameters.setFlashMode(desired); camera.setParameters(parameters); }
            light = "Camera light: " + (night ? "ON" : "OFF") + " · 18:00–08:00, phone local time";
        } catch (RuntimeException error) { light = "Camera light failed: " + error.getMessage(); }
    }

    /** Publish state only to an attached, visible screen. */
    private void notifyScreen() { if (listener != null) listener.changed(running, status, light, lastImage); }
}
