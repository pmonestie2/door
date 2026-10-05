"""Minimal local-network controls; uses only the Python standard library."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import secrets
import threading
from urllib.parse import parse_qs


def start_server(door, host="0.0.0.0", port=8080):
    token = secrets.token_urlsafe(32)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != "/":
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
<form method="post">
<input type="hidden" name="token" value="{token}">
<button formaction="/open">Open</button>
<button formaction="/close">Close</button>
</form><p>Close acts immediately. Automatic control stays active.</p>
<p><a href="/">Refresh status</a></p></html>""".encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.end_headers()
            self.wfile.write(page)

        def do_POST(self):
            if self.path not in ("/open", "/close"):
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
            fields = parse_qs(self.rfile.read(length).decode("utf-8", "replace"))
            if fields.get("token") != [token]:
                self.send_error(403)
                return
            if self.path == "/open":
                door.open_door(source="web")
            else:
                door.close_door()
            self.send_response(303)
            self.send_header("Location", "/")
            self.send_header("Content-Length", "0")
            self.end_headers()

    server = ThreadingHTTPServer((host, port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
