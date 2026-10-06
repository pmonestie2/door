"""Minimal local-network controls; uses only the Python standard library."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import secrets
import select
import socket
import threading
from urllib.parse import parse_qs, urlsplit
from datetime import date
from door_analytics import DEFAULT_END, render_report
from door_camera import CameraImages, CameraCommands, MAX_IMAGE_BYTES


def start_server(door, host="0.0.0.0", port=8080):
    """Start the door-control website in a background thread.

    Returns:
        ThreadingHTTPServer: The running server, which the caller can shut down.
    """
    token = secrets.token_urlsafe(32)
    camera_images = CameraImages()
    camera_commands = CameraCommands()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            """Serve the status page and controls, or reject unknown paths."""
            url = urlsplit(self.path)
            if url.path == "/camera/command":
                command = camera_commands.wait_for_command(disconnected=self.phone_disconnected)
                self.send_json(command, 200 if command else 204)
                return
            if url.path == "/camera/request":
                request_id = parse_qs(url.query).get("id", [""])[0]
                status = camera_commands.status(request_id)
                self.send_json(status or {"error": "Unknown request"}, 200 if status else 404)
                return
            if url.path in ("/camera/latest.jpg", "/camera/latest"):
                self.show_camera(url.path)
                return
            if url.path == "/analytics":
                self.show_analytics(url.query)
                return
            if url.path != "/":
                self.send_error(404)
                return
            state = "Moving / settling" if door.lock else (
                "Open" if door.open else "Closed")
            page = f"""<!doctype html>
<html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Cat door</title>
<style>body{{font:20px sans-serif;max-width:24em;margin:3em auto;padding:1em}}
button{{font:inherit;padding:.7em 1.4em;margin-right:.5em}}</style>
<h1>Cat door</h1><p>{state}</p>
<p>{"Kept closed by website override" if door.website_closed else "Automatic control"}</p>
<p>Daily keep-open: {door.schedule_description()} (Pi local time).</p>
<form method="post">
<input type="hidden" name="token" value="{token}">
<button formaction="/open">Open</button>
<button formaction="/close">Close</button>
</form><p>Close keeps the door closed, overriding the schedule and sensors,
until the next schedule start/end, Open is clicked, or the controller restarts.</p>
<p><button type="button" id="take-photo">Take photo (Pull mode)</button></p>
<p id="camera-result"></p>
<p><button type="button" id="run-inference">Run inference</button></p>
<p>Uses a fresh phone photo in Pull mode. Runs the trained obstruction model.</p>
<p id="inference-result" role="status"></p>
<script>
document.getElementById('run-inference').onclick = async function() {{
  const button = this, result = document.getElementById('inference-result');
  button.disabled = true;
  result.textContent = 'Waiting for a fresh phone photo, then running inference…';
  try {{
    const response = await fetch('/camera/inference', {{method:'POST',
      headers:{{'Content-Type':'application/json'}}, body:'{{}}'}});
    const inference = await response.json();
    if (!response.ok) throw new Error(inference.error || 'Inference failed');
    result.textContent = inference.clear_to_close
      ? 'Clear to close. Door unchanged.'
      : 'Obstructed: keep open. Door unchanged.';
  }} catch (error) {{ result.textContent = error.message; }}
  finally {{ button.disabled = false; }}
}};
document.getElementById('take-photo').onclick = async function() {{
  const button = this, result = document.getElementById('camera-result');
  button.disabled = true;
  try {{
    const response = await fetch('/camera/capture', {{method:'POST',
      headers:{{'Content-Type':'application/json'}}, body:'{{}}'}});
    const request = await response.json();
    if (!response.ok) throw new Error(request.error || 'Request failed');
    result.textContent = 'Waiting for the phone…';
    for (let attempt = 0; attempt < 35; attempt++) {{
      await new Promise(resolve => setTimeout(resolve, 1000));
      const check = await fetch('/camera/request?id=' + request.request_id);
      if (!check.ok) throw new Error('Request no longer available');
      const state = await check.json();
      if (state.status === 'complete') {{
        result.textContent = 'Photo received. Open Latest photo to view it.';
        return;
      }}
      if (state.status === 'timed_out') throw new Error('Timed out. Keep the phone app open and started in Pull mode.');
    }}
    throw new Error('No photo received.');
  }} catch (error) {{ result.textContent = error.message; }}
  finally {{ button.disabled = false; }}
}};
</script>
<p><a href="/">Refresh status</a> · <a href="/analytics">Activity graph</a> · <a href="/camera/latest.jpg">Latest photo</a></p></html>""".encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.end_headers()
            self.wfile.write(page)

        def show_analytics(self, query):
            """Compute the requested six-month report only when its page is opened."""
            if door.analytics is None:
                self.send_error(503, "Analytics unavailable")
                return
            fields = parse_qs(query)
            dataset = fields.get("dataset", ["archive"])[0]
            default_end = date.today().isoformat() if dataset == "live" else DEFAULT_END
            try:
                report = door.analytics.report(
                    end=fields.get("end", [default_end])[0], dataset=dataset,
                    bucket=fields.get("bucket", ["day"])[0])
            except (ValueError, OverflowError):
                self.send_error(400, "Invalid date, dataset, or bucket")
                return
            page = render_report(report).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(page)

        def phone_disconnected(self):
            """Returns:
                bool: Whether the phone cancelled its command wait or closed the connection.
            """
            try:
                readable, _, _ = select.select([self.connection], [], [], 0)
                return bool(readable) and self.connection.recv(1, socket.MSG_PEEK) == b""
            except OSError:
                return True

        def send_json(self, value, status=200):
            """Send a noncached JSON response, or an empty command-wait response."""
            body = b"" if status == 204 else json.dumps(value).encode()
            try:
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass  # The phone may cancel its wait when changing mode or leaving the app.

        def show_camera(self, path):
            """Serve the latest JPEG or its capture and receipt timestamps."""
            try:
                metadata, image = camera_images.latest()
            except FileNotFoundError:
                self.send_error(404, "No camera image received yet")
                return
            except (OSError, ValueError):
                self.send_error(503, "Camera storage unavailable")
                return
            is_image = path.endswith(".jpg")
            body = image if is_image else json.dumps(metadata).encode()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg" if is_image else "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def receive_camera(self):
            """Store a bounded JPEG upload; never operate the door from a snapshot."""
            if self.headers.get_content_type() != "image/jpeg":
                self.send_error(415, "Expected image/jpeg")
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 4 <= length <= MAX_IMAGE_BYTES:
                    raise ValueError("Invalid image size")
            except ValueError:
                self.send_error(400, "Expected a JPEG no larger than 2 MiB")
                return
            self.connection.settimeout(15)
            try:
                image = self.rfile.read(length)
                if len(image) != length:
                    raise ValueError("Incomplete image")
                request_id = self.headers.get("X-Capture-Request-ID")
                metadata = camera_images.save(image, self.headers.get("X-Captured-At", ""), request_id)
                if request_id:
                    camera_commands.complete(request_id, metadata)
            except (ValueError, OverflowError) as error:
                self.send_error(400, str(error))
                return
            except OSError:
                self.send_error(503, "Upload timed out or camera storage unavailable")
                return
            body = json.dumps(metadata).encode()
            self.send_response(201)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            """Apply a JSON API command or a token-protected website form command."""
            if self.path == "/camera":
                self.receive_camera()
                return
            if self.path not in ("/open", "/close", "/camera/capture", "/camera/inference"):
                self.send_error(404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                self.send_error(400)
                return
            if not 0 < length <= 1024:
                self.send_error(400)
                return
            body = self.rfile.read(length).decode("utf-8", "replace")
            is_json = self.headers.get_content_type() == "application/json"
            if is_json:
                try:
                    if not isinstance(json.loads(body), dict):
                        raise ValueError("Expected a JSON object")
                except ValueError:
                    self.send_error(400, "Expected a JSON object")
                    return
            else:
                fields = parse_qs(body)
                if fields.get("token") != [token]:
                    self.send_error(403)
                    return
            if self.path == "/camera/inference":
                try:
                    clear_to_close = door.close_check() is True
                except Exception as error:
                    self.send_json({"error": "Photo/inference check failed: %s" % error}, 503)
                    return
                self.send_json({"clear_to_close": clear_to_close, "mock": False})
                return
            if self.path == "/camera/capture":
                try:
                    request = camera_commands.request_capture()
                except RuntimeError as error:
                    self.send_json({"error": str(error)}, 409)
                    return
                self.send_json(request, 202)
                return
            if self.path == "/open":
                door.open_door(source="website")
            else:
                door.close_door(source="website")
            if is_json:
                response = json.dumps({"open": bool(door.open), "busy": door.lock,
                                       "keep_closed": door.website_closed}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(response)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(response)
            else:
                self.send_response(303)
                self.send_header("Location", "/")
                self.send_header("Content-Length", "0")
                self.end_headers()

    server = ThreadingHTTPServer((host, port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
