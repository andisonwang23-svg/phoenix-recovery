#!/usr/bin/env python3
"""Export a PHOENIX v3 binary log into review and replay artifacts.

Usage:
    python tools/export_drop_log.py flight_3.log --output evidence/run_003

The raw input is copied unchanged. The exporter never claims aerodynamic
validation; it only makes recorded controller evidence inspectable.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import struct
import zlib
from pathlib import Path


HEADER = struct.Struct("<4sBBBBIIIIH2x")
ENTRY = struct.Struct("<IBBH")
TELEMETRY = struct.Struct("<12BH5I4d28f4I")
CONFIG = struct.Struct("<4BI2d8f")

TELEMETRY_FIELDS = [
    "schema_version", "drop_test_state", "guided_drop_status",
    "guided_authority", "flight_state", "guidance_mode", "failure_code",
    "validity_bits", "control_flags", "satellites", "reset_reason",
    "reserved", "drop_test_id", "sample_timestamp_ms",
    "configuration_version", "gps_age_ms", "baro_age_ms", "imu_age_ms",
    "latitude", "longitude", "target_latitude", "target_longitude",
    "gps_altitude_m", "hdop", "altitude_agl_m", "barometric_altitude_m",
    "barometric_pressure_hpa", "barometric_temperature_c",
    "vertical_speed_mps", "vertical_accel_mps2", "ground_speed_mps",
    "gps_course_deg", "target_bearing_deg", "heading_error_deg",
    "distance_to_target_m", "roll_deg", "pitch_deg", "yaw_deg",
    "angular_rate_dps", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "requested_servo1", "requested_servo2", "actual_servo1",
    "actual_servo2", "actual_servo1_us", "actual_servo2_us",
    "battery_voltage_v", "servo_rail_voltage_v", "drop_release_ms",
    "canopy_signature_ms", "stable_descent_ms", "landing_confirm_ms",
]

CONFIG_FIELDS = [
    "schema_version", "guided_authority", "flare_enabled",
    "servo1_controls_left_brake", "configuration_version",
    "target_latitude", "target_longitude", "servo1_neutral_us",
    "servo1_min_us", "servo1_max_us", "servo1_max_brake_us",
    "servo2_neutral_us", "servo2_min_us", "servo2_max_us",
    "servo2_max_brake_us",
]


def read_log(path: Path) -> tuple[dict, list[dict], list[dict], list[dict]]:
    data = path.read_bytes()
    if len(data) < HEADER.size:
        raise ValueError("file is shorter than a PHOENIX header")
    values = HEADER.unpack_from(data, 0)
    header = dict(
        magic=values[0].decode("ascii", "replace"),
        version=values[1],
        flight_number=values[2],
        start_time_ms=values[5],
        end_time_ms=values[6],
        entry_count=values[7],
        data_size=values[8],
        header_crc16=values[9],
        file_size=len(data),
        crc32=f"{zlib.crc32(data) & 0xFFFFFFFF:08X}",
        sha256=hashlib.sha256(data).hexdigest(),
    )
    if header["magic"] != "PRFL":
        raise ValueError("not a PHOENIX PRFL log")
    if header["version"] != 3:
        raise ValueError(
            f"log version {header['version']} is not the host-decodable v3 schema"
        )

    telemetry: list[dict] = []
    events: list[dict] = []
    configs: list[dict] = []
    offset = HEADER.size
    parsed = 0
    while offset + ENTRY.size <= len(data) and parsed < header["entry_count"]:
        timestamp_ms, entry_type, sequence, length = ENTRY.unpack_from(data, offset)
        offset += ENTRY.size
        if length > 256 or offset + length > len(data):
            raise ValueError(f"truncated or invalid entry {parsed} at byte {offset}")
        payload = data[offset : offset + length]
        offset += length
        parsed += 1
        if entry_type == 1:
            if length != TELEMETRY.size:
                raise ValueError(
                    f"telemetry entry {parsed} is {length} bytes, expected {TELEMETRY.size}"
                )
            row = dict(zip(TELEMETRY_FIELDS, TELEMETRY.unpack(payload)))
            row["entry_timestamp_ms"] = timestamp_ms
            row["entry_sequence"] = sequence
            telemetry.append(row)
        elif entry_type in (5, 6):
            events.append(
                dict(
                    timestamp_ms=timestamp_ms,
                    sequence=sequence,
                    severity="error" if entry_type == 6 else "event",
                    message=payload.decode("utf-8", "replace"),
                )
            )
        elif entry_type == 8:
            if length != CONFIG.size:
                raise ValueError(
                    f"configuration entry is {length} bytes, expected {CONFIG.size}"
                )
            configs.append(dict(zip(CONFIG_FIELDS, CONFIG.unpack(payload))))

    header["parsed_entries"] = parsed
    header["log_closed_successfully"] = (
        header["end_time_ms"] != 0 and parsed == header["entry_count"]
    )
    return header, telemetry, events, configs


def write_charts(rows: list[dict], output: Path) -> None:
    """Write dependency-free SVG charts that open in any modern browser."""
    if not rows:
        return
    width, height = 1200, 1040
    left, right, top, panel_height, gap = 80, 25, 45, 205, 45
    plot_width = width - left - right
    t0 = rows[0]["sample_timestamp_ms"]
    times = [(r["sample_timestamp_ms"] - t0) / 1000.0 for r in rows]
    time_max = max(max(times), 0.001)
    panels = [
        ("Descent", [("Altitude AGL", "altitude_agl_m", 1.0, "#2563eb"),
                      ("Vertical speed", "vertical_speed_mps", 1.0, "#dc2626")]),
        ("Navigation", [("GPS course", "gps_course_deg", 1.0, "#2563eb"),
                         ("Target bearing", "target_bearing_deg", 1.0, "#16a34a"),
                         ("Heading error", "heading_error_deg", 1.0, "#dc2626")]),
        ("Servo command percent", [("Requested Servo 1", "requested_servo1", 100.0, "#60a5fa"),
                                    ("Requested Servo 2", "requested_servo2", 100.0, "#f87171"),
                                    ("Actual Servo 1", "actual_servo1", 100.0, "#1d4ed8"),
                                    ("Actual Servo 2", "actual_servo2", 100.0, "#b91c1c")]),
        ("Stability / target", [("Angular rate", "angular_rate_dps", 1.0, "#7c3aed"),
                                 ("Distance to Home", "distance_to_target_m", 1.0, "#ea580c")]),
    ]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:system-ui,sans-serif;fill:#111827}.title{font-size:18px;font-weight:600}.axis{font-size:12px;fill:#4b5563}.legend{font-size:12px}</style>',
    ]
    for panel_index, (title, series) in enumerate(panels):
        y0 = top + panel_index * (panel_height + gap)
        values = [r[field] * scale for _, field, scale, _ in series for r in rows
                  if math.isfinite(r[field] * scale)]
        value_min, value_max = (min(values), max(values)) if values else (0.0, 1.0)
        if value_max - value_min < 1e-6:
            value_min -= 0.5
            value_max += 0.5
        margin = (value_max - value_min) * 0.08
        value_min -= margin
        value_max += margin
        svg.extend([
            f'<text x="{left}" y="{y0 - 14}" class="title">{title}</text>',
            f'<rect x="{left}" y="{y0}" width="{plot_width}" height="{panel_height}" fill="#f9fafb" stroke="#d1d5db"/>',
            f'<text x="{left - 8}" y="{y0 + 12}" text-anchor="end" class="axis">{value_max:.1f}</text>',
            f'<text x="{left - 8}" y="{y0 + panel_height}" text-anchor="end" class="axis">{value_min:.1f}</text>',
        ])
        for series_index, (label, field, scale, color) in enumerate(series):
            points = []
            for t, row in zip(times, rows):
                value = row[field] * scale
                if not math.isfinite(value):
                    continue
                x = left + plot_width * t / time_max
                y = y0 + panel_height * (value_max - value) / (value_max - value_min)
                points.append(f"{x:.1f},{y:.1f}")
            if points:
                svg.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="2"/>')
            legend_x = left + 12 + series_index * 205
            svg.extend([
                f'<line x1="{legend_x}" y1="{y0 + 18}" x2="{legend_x + 25}" y2="{y0 + 18}" stroke="{color}" stroke-width="3"/>',
                f'<text x="{legend_x + 32}" y="{y0 + 22}" class="legend">{label}</text>',
            ])
        if panel_index == len(panels) - 1:
            svg.append(f'<text x="{left + plot_width / 2}" y="{y0 + panel_height + 28}" text-anchor="middle" class="axis">payload time (seconds), 0 to {time_max:.1f}</text>')
    svg.append("</svg>")
    output.write_text("\n".join(svg) + "\n")


def summarize(header: dict, rows: list[dict], events: list[dict], configs: list[dict]) -> dict:
    commands = [abs(r["actual_servo1"]) for r in rows] + [abs(r["actual_servo2"]) for r in rows]
    signs: list[int] = []
    for row in rows:
        command = row["actual_servo2"] - row["actual_servo1"]
        sign = 1 if command > 0.01 else -1 if command < -0.01 else 0
        if sign and (not signs or sign != signs[-1]):
            signs.append(sign)
    return {
        "log_closed_successfully": header["log_closed_successfully"],
        "telemetry_rows": len(rows),
        "event_count": len(events),
        "maximum_actual_servo_command": max(commands, default=0.0),
        "turn_reversals": max(0, len(signs) - 1),
        "final_distance_to_home_m": rows[-1]["distance_to_target_m"] if rows else None,
        "failsafe_observed": any(r["failure_code"] != 0 for r in rows),
        "gps_suspect_observed": any(r["control_flags"] & 2 for r in rows),
        "course_hold_observed": any(r["control_flags"] & 4 for r in rows),
        "oscillation_damping_observed": any(r["control_flags"] & 8 for r in rows),
        "flare_disabled_in_config": bool(configs) and all(
            config["flare_enabled"] == 0 for config in configs
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    header, rows, events, configs = read_log(args.log)
    shutil.copy2(args.log, args.output / args.log.name)
    with (args.output / "telemetry.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=TELEMETRY_FIELDS + ["entry_timestamp_ms", "entry_sequence"])
        writer.writeheader(); writer.writerows(rows)
    (args.output / "events.json").write_text(json.dumps(events, indent=2) + "\n")
    (args.output / "configuration.json").write_text(json.dumps(configs, indent=2) + "\n")
    summary = summarize(header, rows, events, configs)
    (args.output / "summary.json").write_text(
        json.dumps({"header": header, "summary": summary}, indent=2) + "\n"
    )
    (args.output / "replay.json").write_text(json.dumps(rows, separators=(",", ":")) + "\n")
    write_charts(rows, args.output / "summary_charts.svg")
    print(json.dumps({"output": str(args.output), "summary": summary}, indent=2))


if __name__ == "__main__":
    main()
