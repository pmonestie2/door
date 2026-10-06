package com.pierre.catdoorcamera;

import org.json.JSONObject;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;

/** Local HTTP transport. Each run gets its own cancellable client; nothing is cached. */
final class PiClient {
    private final String base;
    private volatile boolean cancelled;
    private volatile HttpURLConnection connection;

    PiClient(String base) { this.base = base; }

    /** Cancel the current upload or command wait when Stop is pressed. */
    void cancel() {
        cancelled = true;
        HttpURLConnection current = connection;
        if (current != null) current.disconnect();
    }

    /** Returns:
     *     JSONObject: The capture command, or null after an idle command wait.
     */
    JSONObject waitForCommand() throws Exception {
        return exchange("/camera/command", null, null, null);
    }

    /** Upload the fresh JPEG and include its Pull request ID when present. */
    void upload(byte[] jpeg, String capturedAt, String requestId) throws Exception {
        exchange("/camera", jpeg, capturedAt, requestId);
    }

    /** Returns:
     *     JSONObject: Decoded server response, or null for HTTP 204.
     */
    private JSONObject exchange(String path, byte[] jpeg, String capturedAt, String requestId) throws Exception {
        if (cancelled) throw new IOException("Stopped");
        HttpURLConnection current = (HttpURLConnection) new URL(base + path).openConnection();
        connection = current;
        try {
            if (cancelled) throw new IOException("Stopped");
            current.setConnectTimeout(5000);
            current.setReadTimeout(jpeg == null ? 25000 : 10000);
            current.setUseCaches(false);
            current.setInstanceFollowRedirects(false);
            if (jpeg != null) {
                current.setRequestMethod("POST");
                current.setDoOutput(true);
                current.setFixedLengthStreamingMode(jpeg.length);
                current.setRequestProperty("Content-Type", "image/jpeg");
                current.setRequestProperty("X-Captured-At", capturedAt);
                if (requestId != null) current.setRequestProperty("X-Capture-Request-ID", requestId);
                try (OutputStream output = current.getOutputStream()) { output.write(jpeg); }
            }
            int status = current.getResponseCode();
            if (status == 204 && jpeg == null) return null;
            if (status != (jpeg == null ? 200 : 201)) throw new IOException("Pi returned HTTP " + status);
            try (InputStream input = current.getInputStream(); ByteArrayOutputStream output = new ByteArrayOutputStream()) {
                byte[] buffer = new byte[4096];
                int count;
                while ((count = input.read(buffer)) != -1) {
                    if (output.size() + count > 65536) throw new IOException("Server response too large");
                    output.write(buffer, 0, count);
                }
                return new JSONObject(output.toString("UTF-8"));
            }
        } finally {
            current.disconnect();
            connection = null;
        }
    }
}
