#!/usr/bin/env python3
"""Request a fresh phone photo through the running cat-door server."""

import argparse
import json
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen


def request_json(url, payload=None):
    """Returns:
        dict: The server's decoded JSON response to a GET or JSON POST.
    """
    data = None if payload is None else json.dumps(payload).encode()
    request = Request(url, data=data, headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=5) as response:
        return json.load(response)


def pull_photo(server="http://127.0.0.1:8080"):
    """Returns:
        dict: Metadata for the uploaded photo matching this capture request.

    Raises:
        TimeoutError: If the phone fails to return a photo within thirty-five seconds.
        HTTPError: If the server rejects the request, including another pending capture.
        URLError: If the server cannot be reached.
    """
    server = server.rstrip("/")
    capture = request_json(server + "/camera/capture", {})
    status_url = server + "/camera/request?" + urlencode({"id": capture["request_id"]})
    deadline = time.monotonic() + 35
    while time.monotonic() < deadline:
        status = request_json(status_url)
        if status["status"] == "complete":
            return status["image"]
        if status["status"] == "timed_out":
            break
        time.sleep(0.5)
    raise TimeoutError("No photo received. Keep the phone app open and started in Pull mode.")


def main():
    """Returns:
        int: Zero after printing photo metadata, or one after reporting a failure.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="http://127.0.0.1:8080",
                        help="Door server URL (default: http://127.0.0.1:8080)")
    args = parser.parse_args()
    try:
        metadata = pull_photo(args.server)
    except HTTPError as error:
        if error.code == 409:
            message = "Another capture is pending. Wait for it to finish or time out."
        elif error.code == 404:
            message = "Capture endpoint not found. Restart the updated door controller."
        else:
            message = "Server returned HTTP %s." % error.code
        print(message, file=sys.stderr)
        return 1
    except (URLError, OSError, ValueError, KeyError) as error:
        print("Photo request failed: %s" % error, file=sys.stderr)
        return 1
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
