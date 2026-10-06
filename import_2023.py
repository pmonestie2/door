"""One-time conversion of historical logs into ordinary analytics events."""

import argparse
from datetime import datetime
import re
import sqlite3

from door_analytics import Analytics, DEFAULT_DATABASE


def import_logs(analytics, paths):
    """Import 2023 completed cycles; each magnet opening counts as one inferred transit."""
    lines = set()
    for path in paths:
        with open(path, errors="replace") as stream:
            for line in stream:
                match = re.match(r"\[(.{24})\] (.*)", line)
                if match:
                    try:
                        when = datetime.strptime(match[1], "%a %b %d %H:%M:%S %Y")
                    except ValueError:
                        continue
                    if when.year == 2023:
                        lines.add((when, match[2]))
    events = []
    for when, message in sorted(lines):
        if message.startswith("door opened") or message == "door closed":
            kind = "OPEN" if message.startswith("door opened") else "CLOSE"
            source = message.split("source=", 1)[1].strip() if "source=" in message else "unknown"
            timestamp = when.isoformat(timespec="microseconds")
            events.append((timestamp, kind, source, "archive"))
            if kind == "OPEN" and source == "magnet":
                events.append((timestamp, "TRANSIT", "magnet", "archive"))
    with sqlite3.connect(analytics.path) as db:
        db.executemany("INSERT OR IGNORE INTO events VALUES (?, ?, ?, ?)", events)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Import 2023 door logs for website analytics")
    parser.add_argument("logs", nargs="+")
    parser.add_argument("--database", default=str(DEFAULT_DATABASE))
    args = parser.parse_args()
    import_logs(Analytics(args.database), args.logs)
    print("Imported 2023 events (duplicates ignored).")
