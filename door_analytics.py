"""Persistent magnet activity and on-demand six-month reports (standard library only)."""

import calendar
from datetime import date, datetime, timedelta
from html import escape
from pathlib import Path
import sqlite3
import threading

DEFAULT_DATABASE = Path(__file__).with_name("door_events.sqlite3")
DEFAULT_END = "2023-12-31"
QUIET_SECONDS = 8


class Analytics:
    def __init__(self, path=DEFAULT_DATABASE):
        """Create the event database and initialize magnet burst tracking."""
        self.path = str(path)
        self.last_detection = float("-inf")
        self.lock = threading.Lock()
        with sqlite3.connect(self.path) as db:
            db.execute("""CREATE TABLE IF NOT EXISTS events (
                time TEXT NOT NULL, kind TEXT NOT NULL, source TEXT NOT NULL,
                dataset TEXT NOT NULL,
                UNIQUE(time, kind, source, dataset))""")
            db.execute("CREATE INDEX IF NOT EXISTS event_dates ON events(dataset, time)")

    def record(self, timestamp, kind, source, dataset="live"):
        """Persist an event using the Pi's local wall time; ignore duplicate imports."""
        when = datetime.fromtimestamp(timestamp).isoformat(timespec="microseconds")
        with sqlite3.connect(self.path) as db:
            db.execute("INSERT OR IGNORE INTO events VALUES (?, ?, ?, ?)",
                       (when, kind, source, dataset))

    def magnet(self, timestamp, blocked=False):
        """Record one inferred TRANSIT per magnet burst, including while scheduled open."""
        with self.lock:
            fresh = timestamp - self.last_detection >= QUIET_SECONDS
            self.last_detection = timestamp
            if fresh and not blocked:
                self.record(timestamp, "TRANSIT", "magnet")

    def movement_finished(self, timestamp, kind, source):
        """Record completed motor movement and require quiet before another transit."""
        with self.lock:
            self.last_detection = timestamp
            self.record(timestamp, kind, source)
            if kind == "OPEN" and source == "magnet":
                self.record(timestamp, "TRANSIT", "magnet")

    def report(self, end=DEFAULT_END, dataset="archive", bucket="day"):
        """Returns:
            dict: Six calendar months of counts, peak buckets, and recorded date coverage.

        Raises:
            ValueError: If the date, dataset, or bucket is invalid.
        """
        end_date = date.fromisoformat(end)
        if dataset not in ("archive", "live") or bucket not in ("day", "hour"):
            raise ValueError("Invalid dataset or bucket")
        stop = end_date + timedelta(days=1)
        month_index = stop.year * 12 + stop.month - 1 - 6
        year, month = divmod(month_index, 12)
        start = date(year, month + 1, min(stop.day, calendar.monthrange(year, month + 1)[1]))
        with sqlite3.connect(self.path) as db:
            rows = db.execute("""SELECT time, kind FROM events
                WHERE dataset=? AND time>=? AND time<? ORDER BY time""",
                (dataset, start.isoformat(), stop.isoformat())).fetchall()
        days = (stop - start).days
        labels = ([(start + timedelta(days=n)).isoformat() for n in range(days)]
                  if bucket == "day" else [f"{hour:02d}:00" for hour in range(24)])
        counts = dict.fromkeys(labels, 0)
        for when, kind in rows:
            if kind == "TRANSIT":
                counts[when[:10] if bucket == "day" else when[11:13] + ":00"] += 1
        peak = max(counts.values(), default=0)
        total = sum(counts.values())
        return dict(start=start.isoformat(), end=end, dataset=dataset, bucket=bucket,
                    counts=counts, total=total, average=total / days, peak=peak,
                    busiest=[key for key, value in counts.items() if value == peak] if peak else [],
                    coverage=(rows[0][0][:10], rows[-1][0][:10]) if rows else None,
                    opens=sum(kind == "OPEN" for _, kind in rows),
                    closes=sum(kind == "CLOSE" for _, kind in rows))


def render_report(report):
    """Returns:
        str: A small HTML report with a responsive SVG bar graph.
    """
    counts = report["counts"]
    width = 900 / len(counts)
    peak = report["peak"]
    bars = []
    for index, (label, count) in enumerate(counts.items()):
        height = 180 * count / max(1, peak)
        bars.append(f'<rect x="{index * width:.2f}" y="{200-height:.2f}" '
                    f'width="{max(.5, width-.5):.2f}" height="{height:.2f}" fill="#267baf">'
                    f'<title>{label}: {count} transits</title></rect>')
    busiest = ", ".join(report["busiest"][:8]) or "None"
    if len(report["busiest"]) > 8:
        busiest += " (more ties)"
    coverage = " to ".join(report["coverage"]) if report["coverage"] else "No records"
    dataset_options = "".join(f'<option value="{value}" {"selected" if report["dataset"] == value else ""}>{label}</option>'
                              for value, label in (("archive", "2023 logs"), ("live", "Live events")))
    bucket_options = "".join(f'<option value="{value}" {"selected" if report["bucket"] == value else ""}>{label}</option>'
                             for value, label in (("day", "Daily"), ("hour", "Hour of day")))
    return f'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Cat activity</title>
<style>body{{font:18px sans-serif;max-width:55em;margin:2em auto;padding:1em}}select,input,button{{font:inherit}}svg{{width:100%}}</style>
<a href="/">Door controls</a><h1>Cat activity</h1>
<form><label>Data <select name="dataset">{dataset_options}</select></label>
<label>Six months ending <input type="date" name="end" value="{escape(report['end'])}" required></label>
<label>Buckets <select name="bucket">{bucket_options}</select></label><button>Show</button></form>
<p>{report['start']} – {report['end']} · Pi local time</p>
<p><b>{report['total']}</b> inferred transits · {report['average']:.2f} per calendar day<br>
Highest bucket: {peak} · {busiest}</p>
<svg viewBox="0 0 900 230" role="img" aria-label="Transit counts, maximum {peak}">
<text x="0" y="15">{peak}</text>{''.join(bars)}
<text x="0" y="225">{next(iter(counts))}</text>
<text x="900" y="225" text-anchor="end">{list(counts)[-1]}</text></svg>
<p>{'Each bar is one date.' if report['bucket'] == 'day' else 'Each bar sums that hour across the entire six months.'} Hover a bar for its count.</p>
<p>Motor events: {report['opens']} opens / {report['closes']} closes.<br>Recorded event dates: {coverage}.</p>
<p>TRANSIT means a magnet visit, not a confirmed passage. Direction (in/out) is unknown.
Visits separated by less than 8 seconds are grouped; movement and settling are excluded.
2023 counts use completed magnet-triggered openings only; their closes do not add transits. Historical logs may be incomplete. Empty buckets mean no recorded events,
not proof of inactivity; the average includes every day in the selected window.</p></html>'''
