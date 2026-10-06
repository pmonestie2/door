package com.pierre.catdoorcamera;

import android.Manifest;
import android.app.Activity;
import android.content.*;
import android.content.pm.PackageManager;
import android.graphics.*;
import android.hardware.Camera;
import android.net.Uri;
import android.os.*;
import android.provider.Settings;
import android.text.InputType;
import android.view.TextureView;
import android.view.View;
import android.widget.*;
import java.net.URI;

/** Configuration and preview screen; the service owns each active capture run. */
@SuppressWarnings("deprecation")
public final class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private SharedPreferences preferences;
    private EditText address;
    private Switch mode;
    private Spinner interval;
    private static final int[] INTERVALS = {1, 2, 5, 10, 30, 60};
    private SeekBar cropSize, cropX, cropY;
    private TextView status, lightStatus;
    private Button start;
    private TextureView preview;
    private FrameLayout previewBox;
    private CropOverlay overlay;
    private ImageView lastImage;
    private Camera camera;
    private CameraService service;
    private boolean foreground, running, bound;
    private float size = 1, x = .5f, y = .5f;
    private Bitmap thumbnail;
    private byte[] displayedImage;
    private final ServiceConnection connection = new ServiceConnection() {
        @Override public void onServiceConnected(ComponentName name, IBinder binder) {
            service = ((CameraService.LocalBinder) binder).getService();
            if (foreground) service.setListener(MainActivity.this::renderState);
        }
        @Override public void onServiceDisconnected(ComponentName name) {
            service = null; running = false; start.setEnabled(false);
            status.setText("Camera service disconnected. Reopen the app.");
        }
    };

    /** Create controls and restore the last address, crop, and mode. */
    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        preferences = getSharedPreferences("camera", MODE_PRIVATE);
        size = preferences.getFloat("size", 1);
        x = preferences.getFloat("x", .5f);
        y = preferences.getFloat("y", .5f);
        ScrollView scroll = new ScrollView(this);
        scroll.setFitsSystemWindows(true);
        LinearLayout layout = new LinearLayout(this);
        layout.setOrientation(LinearLayout.VERTICAL);
        layout.setPadding(dp(16), dp(16), dp(16), dp(16));
        scroll.addView(layout);
        setContentView(scroll);
        label(layout, "Cat door camera").setTextSize(26);
        address = new EditText(this);
        address.setSingleLine();
        address.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        String savedAddress = preferences.getString("address", "http://pi3:8080");
        if ("http://pi3.local:8080".equals(savedAddress)) savedAddress = "http://pi3:8080";
        address.setText(savedAddress);
        layout.addView(address);
        mode = new Switch(this);
        mode.setText("Pull: Pi requests (off = timed Push)");
        mode.setChecked(preferences.getBoolean("pull", false));
        layout.addView(mode);
        label(layout, "Push interval");
        interval = new Spinner(this);
        ArrayAdapter<String> intervals = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item,
                new String[]{"1 second", "2 seconds", "5 seconds", "10 seconds", "30 seconds", "60 seconds"});
        intervals.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        interval.setAdapter(intervals);
        int savedInterval = preferences.getInt("intervalSeconds", 10);
        interval.setSelection(3);
        for (int i = 0; i < INTERVALS.length; i++) if (INTERVALS[i] == savedInterval) interval.setSelection(i);
        layout.addView(interval);
        interval.setEnabled(!mode.isChecked());
        mode.setOnCheckedChangeListener((button, checked) -> interval.setEnabled(!running && !checked));
        start = new Button(this);
        start.setText("Start");
        start.setOnClickListener(view -> { if (running) stop(); else start(); });
        layout.addView(start);
        status = label(layout, "Connecting to camera service…");
        start.setEnabled(false);
        lightStatus = label(layout, "Camera light: automatic 18:00–08:00 (phone local time)");
        previewBox = new FrameLayout(this);
        preview = new TextureView(this);
        preview.setSurfaceTextureListener(this);
        previewBox.addView(preview, new FrameLayout.LayoutParams(-1, -1));
        overlay = new CropOverlay(this);
        previewBox.addView(overlay, new FrameLayout.LayoutParams(-1, -1));
        layout.addView(previewBox, new LinearLayout.LayoutParams(-1, dp(360)));
        cropSize = slider(layout, "Crop size", Math.round((size - .2f) / .8f * 100));
        cropX = slider(layout, "Horizontal position", Math.round(x * 100));
        cropY = slider(layout, "Vertical position", Math.round(y * 100));
        Button battery = new Button(this);
        battery.setText("Allow screen-off operation");
        battery.setOnClickListener(view -> allowScreenOff());
        layout.addView(battery);
        label(layout, "Last captured crop (upload status shown above)");
        lastImage = new ImageView(this);
        lastImage.setAdjustViewBounds(true);
        layout.addView(lastImage);
        label(layout, "After Start, the screen may sleep and you can leave the app. Use Stop here or in the camera notification to end capture. Stop to change mode or crop. Photos go only to the Pi, never to Photos or cloud backup.");
    }

    /** Bind to the same service instance without starting a new capture run. */
    @Override public void onStart() {
        super.onStart();
        bound = bindService(new Intent(this, CameraService.class), connection, BIND_AUTO_CREATE);
    }

    /** Reattach the screen; request permissions only while visible. */
    @Override public void onResume() {
        super.onResume();
        foreground = true;
        if (Build.VERSION.SDK_INT >= 23 && checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED)
            requestPermissions(new String[]{Manifest.permission.CAMERA}, 1);
        if (service != null) service.setListener(this::renderState);
    }

    /** Release only the screen's idle preview; an active service continues uninterrupted. */
    @Override public void onPause() {
        foreground = false;
        saveSettings();
        if (service != null) service.setListener(null);
        closeCamera();
        super.onPause();
    }

    /** Detach the screen without stopping the started foreground service. */
    @Override public void onStop() {
        if (bound) { unbindService(connection); bound = false; }
        service = null;
        super.onStop();
    }

    /** Release the screen's in-memory thumbnail. */
    @Override public void onDestroy() {
        if (thumbnail != null) thumbnail.recycle();
        super.onDestroy();
    }

    /** Resume preview after camera permission has been granted. */
    @Override public void onRequestPermissionsResult(int code, String[] permissions, int[] results) {
        super.onRequestPermissionsResult(code, permissions, results);
        if (code != 1) return;
        if (results.length > 0 && results[0] == PackageManager.PERMISSION_GRANTED) {
            if (foreground && preview.isAvailable()) openCamera(preview.getSurfaceTexture());
        } else status.setText("Camera permission denied. Enable Camera in Android app settings.");
    }

    /** Start capture from the visible app, as required by newer Android camera permissions. */
    private void start() {
        if (service == null || camera == null) { status.setText("Wait for the camera preview."); return; }
        String base = address.getText().toString().trim();
        try {
            URI uri = new URI(base);
            if (!("http".equals(uri.getScheme()) || "https".equals(uri.getScheme())) || uri.getHost() == null
                    || uri.getUserInfo() != null || uri.getQuery() != null || uri.getFragment() != null
                    || !(uri.getPath().isEmpty() || "/".equals(uri.getPath()))) throw new Exception();
        } catch (Exception error) { status.setText("Enter http://PI-IP:8080 (no /camera suffix)."); return; }
        saveSettings();
        closeCamera();
        running = true;
        setControls(false);
        Intent intent = new Intent(this, CameraService.class).setAction(CameraService.START);
        try {
            if (Build.VERSION.SDK_INT >= 26) startForegroundService(intent); else startService(intent);
            if (Build.VERSION.SDK_INT >= 33 && checkSelfPermission(Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED)
                requestPermissions(new String[]{Manifest.permission.POST_NOTIFICATIONS}, 2);
        } catch (RuntimeException error) { renderState(false, "Could not start: " + error.getMessage(), "Camera light off", null); }
    }

    /** Stop the capture service explicitly; merely leaving the screen never calls this. */
    private void stop() {
        startService(new Intent(this, CameraService.class).setAction(CameraService.STOP));
    }

    /** Let the user exempt this dedicated camera from Doze's screen-off network suspension. */
    private void allowScreenOff() {
        if (Build.VERSION.SDK_INT < 23) { status.setText("Screen-off capture is supported."); return; }
        PowerManager power = (PowerManager) getSystemService(POWER_SERVICE);
        if (power.isIgnoringBatteryOptimizations(getPackageName())) {
            status.setText("Battery optimization is already disabled for this app."); return;
        }
        try {
            startActivity(new Intent(Settings.ACTION_REQUEST_IGNORE_BATTERY_OPTIMIZATIONS,
                    Uri.parse("package:" + getPackageName())));
        } catch (ActivityNotFoundException error) {
            startActivity(new Intent(Settings.ACTION_IGNORE_BATTERY_OPTIMIZATION_SETTINGS));
        }
    }

    /** Display current service state, including after reopening an already-running session. */
    private void renderState(boolean active, String message, String light, byte[] jpeg) {
        if (!foreground) return;
        running = active;
        setControls(!active);
        if (active) closeCamera();
        else if (preview.isAvailable()) openCamera(preview.getSurfaceTexture());
        status.setText(message);
        lightStatus.setText(light);
        if (jpeg != null && jpeg != displayedImage) {
            displayedImage = jpeg;
            Bitmap image = BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
            lastImage.setImageBitmap(image);
            if (thumbnail != null) thumbnail.recycle();
            thumbnail = image;
        }
    }

    /** Open the back camera at a modest preview resolution for old phones. */
    private void openCamera(SurfaceTexture texture) {
        if (!foreground || camera != null || service == null || running) return;
        if (Build.VERSION.SDK_INT >= 23 && checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) return;
        try {
            Camera.CameraInfo info = new Camera.CameraInfo();
            int selected = -1;
            for (int id = 0; id < Camera.getNumberOfCameras(); id++) {
                Camera.getCameraInfo(id, info);
                if (info.facing == Camera.CameraInfo.CAMERA_FACING_BACK) { selected = id; break; }
            }
            if (selected < 0) throw new Exception("No back camera");
            camera = Camera.open(selected);
            Camera.Parameters parameters = camera.getParameters();
            Camera.Size best = null;
            for (Camera.Size candidate : parameters.getSupportedPreviewSizes()) {
                if (best == null || Math.abs(candidate.width * candidate.height - 640 * 480)
                        < Math.abs(best.width * best.height - 640 * 480)) best = candidate;
            }
            parameters.setPreviewSize(best.width, best.height);
            if (parameters.getSupportedFocusModes().contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO))
                parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO);
            camera.setParameters(parameters);
            int rotation = (info.orientation - getWindowManager().getDefaultDisplay().getRotation() * 90 + 360) % 360;
            camera.setDisplayOrientation(rotation);
            int availableWidth = getResources().getDisplayMetrics().widthPixels - dp(32);
            boolean rotated = rotation == 90 || rotation == 270;
            previewBox.getLayoutParams().height = Math.round(availableWidth * (rotated ? (float) best.width / best.height : (float) best.height / best.width));
            previewBox.requestLayout();
            camera.setPreviewTexture(texture);
            camera.startPreview();
            status.setText("Camera ready. Adjust the crop, then tap Start.");
        } catch (Exception error) { closeCamera(); status.setText("Camera failed: " + error.getMessage()); }
    }

    /** Release the idle preview camera; never touch the service's camera. */
    private void closeCamera() {
        if (camera != null) { camera.release(); camera = null; }
    }

    /** Open the idle preview when its display surface is available. */
    @Override public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) { openCamera(surface); }
    @Override public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) { }
    @Override public void onSurfaceTextureUpdated(SurfaceTexture surface) { }
    /** Returns:
     *     boolean: True because the idle preview no longer needs the destroyed surface.
     */
    @Override public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) { closeCamera(); return true; }

    /** Persist only settings; image data is never written to phone storage. */
    private void saveSettings() {
        preferences.edit().putString("address", address.getText().toString().trim()).putBoolean("pull", mode.isChecked())
                .putFloat("size", size).putFloat("x", x).putFloat("y", y)
                .putInt("intervalSeconds", INTERVALS[interval.getSelectedItemPosition()]).apply();
    }

    /** Lock configuration while a run is active. */
    private void setControls(boolean editable) {
        address.setEnabled(editable); mode.setEnabled(editable);
        interval.setEnabled(editable && !mode.isChecked());
        cropSize.setEnabled(editable); cropX.setEnabled(editable); cropY.setEnabled(editable);
        start.setText(editable ? "Start" : "Stop");
        start.setEnabled(service != null);
        previewBox.setVisibility(editable ? View.VISIBLE : View.GONE);
    }

    /** Returns:
     *     SeekBar: A crop slider that refreshes the yellow preview rectangle.
     */
    private SeekBar slider(LinearLayout parent, String text, int value) {
        label(parent, text);
        SeekBar bar = new SeekBar(this); bar.setMax(100); bar.setProgress(value); parent.addView(bar);
        bar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            public void onProgressChanged(SeekBar view, int progress, boolean fromUser) {
                if (!fromUser) return;
                if (view == cropSize) size = .2f + .8f * progress / 100;
                if (view == cropX) x = progress / 100f;
                if (view == cropY) y = progress / 100f;
                overlay.invalidate();
            }
            public void onStartTrackingTouch(SeekBar view) { }
            public void onStopTrackingTouch(SeekBar view) { saveSettings(); }
        });
        return bar;
    }

    /** Returns:
     *     TextView: A new text label in the supplied layout.
     */
    private TextView label(LinearLayout parent, String text) {
        TextView label = new TextView(this); label.setText(text); label.setTextSize(16); parent.addView(label); return label;
    }

    /** Returns:
     *     int: Screen pixels corresponding to the supplied density-independent size.
     */
    private int dp(int value) { return Math.round(value * getResources().getDisplayMetrics().density); }

    private final class CropOverlay extends View {
        private final Paint paint = new Paint();
        CropOverlay(Context context) { super(context); paint.setColor(Color.YELLOW); paint.setStrokeWidth(dp(2)); paint.setStyle(Paint.Style.STROKE); }
        @Override protected void onDraw(Canvas canvas) {
            float left = getWidth() * x * (1 - size), top = getHeight() * y * (1 - size);
            canvas.drawRect(left, top, left + getWidth() * size, top + getHeight() * size, paint);
        }
    }
}
