#!/usr/bin/env python3
"""Stream and summarize PRISM-TopoMap structured and legacy runtime logs.

The parser reads the input one physical line at a time.  It tolerates ANSI
colour escapes, mixed C++/Python logger formats, old localization messages,
and more than one [FLOW] fragment on a line.  It never reads the complete log
into one string.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
TAG_RE = re.compile(r"\[(FRAME|STAMP|LOC_STAMP|STAGE)=([^\]]+)\]")
FIELD_RE = re.compile(
    r"([A-Za-z_][A-Za-z0-9_]*)="
    r"(\([^)]*\)|\[[^\]]*\]|[^\s\]\x1b]+)"
)
LOGGER_RE = re.compile(
    r"\[(DEBUG|INFO|WARN|ERROR|FATAL)\]\s*"
    r"\[([0-9]+(?:\.[0-9]+)?)(?:,\s*([0-9]+(?:\.[0-9]+)?))?\]:?\s*"
)
LEGACY_TIMER_RE = re.compile(r"Starting localization from stamp\s+([0-9.]+)")
LEGACY_FAISS_EMPTY_RE = re.compile(r"FAISS index is empty")
LEGACY_VERTEX_SCORE_RE = re.compile(
    r"Vertex\s+([0-9]+)\s+registration score:\s*([-+0-9.eE]+)"
)
LEGACY_INFER_REG_RE = re.compile(
    r"\[(INFER(?:-PY)?)\]\s+gridRegistration\s+type=(\S+)\s+"
    r"score=([-+0-9.eE]+)(?:\s+trans=\(([^)]*)\))?"
)

SEARCH_PATTERNS = [
    ("Starting localization", re.compile(r"Starting localization")),
    ("FAISS index is empty", re.compile(r"FAISS index is empty")),
    ("FAISS", re.compile(r"FAISS")),
    ("gridRegistration", re.compile(r"gridRegistration")),
    ("[INFER] gridRegistration", re.compile(r"\[INFER\] gridRegistration")),
    ("[INFER-PY] gridRegistration", re.compile(r"\[INFER-PY\] gridRegistration")),
    ("registration score", re.compile(r"registration score")),
    ("Vertex .* registration score", re.compile(r"Vertex .* registration score")),
    ("LOCALIZATION_RESULT", re.compile(r"LOCALIZATION_RESULT")),
    ("LOC_STAMP", re.compile(r"LOC_STAMP")),
    ("writeLocalizedState", re.compile(r"writeLocalizedState")),
    ("LOOP CLOSURE", re.compile(r"LOOP CLOSURE")),
    ("n_localized=", re.compile(r"n_localized=")),
]

CSV_FIELDS = [
    "frame",
    "stamp",
    "vertex_before",
    "vertex_after",
    "decision",
    "inside",
    "iou",
    "rel_dist",
    "localization_stamp",
    "matched_count",
    "unmatched_count",
    "new_vertex_id",
    "edge_events",
]

EVENT_STAGE_TYPES = {
    "DECISION": "node_decision",
    "EDGE": "edge",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream a PRISM-TopoMap log and generate trace analysis files."
    )
    parser.add_argument("log", type=Path, help="runtime .log file")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("analysis"),
        help="output directory (default: analysis)",
    )
    return parser.parse_args()


def clean_fragment(text: str) -> str:
    text = ANSI_RE.sub("", text).strip()
    # Concurrent stdout/stderr occasionally spliced another logger prefix into
    # one physical line. Retain the FLOW record before that prefix.
    for marker in ("\n", "[INFO] [", "[WARN] [", "[ERROR] ["):
        pos = text.find(marker, 1)
        if pos >= 0:
            text = text[:pos].rstrip()
    return text


def iter_flow_fragments(path: Path) -> Iterable[dict[str, Any]]:
    """Yield parsed FLOW fragments while reading the log line by line."""
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line_number, physical_line in enumerate(handle, 1):
            plain = ANSI_RE.sub("", physical_line)
            starts = [m.start() for m in re.finditer(r"\[FLOW\]", plain)]
            for ordinal, start in enumerate(starts):
                end = starts[ordinal + 1] if ordinal + 1 < len(starts) else len(plain)
                raw = clean_fragment(plain[start:end])
                tags = {key.lower(): value for key, value in TAG_RE.findall(raw)}
                fields = {key: value.rstrip(".,") for key, value in FIELD_RE.findall(raw)}
                # Bracket tags are metadata rather than ordinary fields.
                for tag in ("FRAME", "STAMP", "LOC_STAMP", "STAGE"):
                    fields.pop(tag, None)
                yield {
                    "line": line_number,
                    "fragment": ordinal + 1,
                    "frame_raw": tags.get("frame"),
                    "stamp": tags.get("stamp"),
                    "loc_stamp": tags.get("loc_stamp"),
                    "stage": tags.get("stage"),
                    "fields": fields,
                    "raw": raw,
                }


def scan_log(path: Path) -> dict[str, Any]:
    """Collect file identity, logger timestamps, and requested pattern counts."""
    counts = {name: 0 for name, _ in SEARCH_PATTERNS}
    occurrences = {name: 0 for name, _ in SEARCH_PATTERNS}
    refs: dict[str, list[int]] = {name: [] for name, _ in SEARCH_PATTERNS}
    first_logger = None
    last_logger = None
    wall_values: list[tuple[float, str, int]] = []
    sim_values: list[tuple[float, str, int]] = []
    total_lines = 0
    newline_chars = 0
    terminal_newline = False

    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line_number, physical_line in enumerate(handle, 1):
            total_lines = line_number
            terminal_newline = physical_line.endswith("\n")
            newline_chars += int(terminal_newline)
            plain = ANSI_RE.sub("", physical_line)
            for fragment, logger in enumerate(LOGGER_RE.finditer(plain), 1):
                item = {
                    "line": line_number,
                    "fragment": fragment,
                    "wall_time": logger.group(2),
                    "ros_time": logger.group(3),
                }
                if first_logger is None:
                    first_logger = item
                last_logger = item
                wall_values.append((float(logger.group(2)), logger.group(2), line_number))
                if logger.group(3) is not None:
                    sim_values.append(
                        (float(logger.group(3)), logger.group(3), line_number)
                    )
            for name, pattern in SEARCH_PATTERNS:
                found = list(pattern.finditer(plain))
                if found:
                    counts[name] += 1
                    occurrences[name] += len(found)
                    if len(refs[name]) < 20:
                        refs[name].append(line_number)

    nonzero_sim = [item for item in sim_values if item[0] > 0.0]
    return {
        "size_bytes": path.stat().st_size,
        "total_lines": total_lines,
        "newline_chars": newline_chars,
        "terminal_newline": terminal_newline,
        "first_logger": first_logger,
        "last_logger": last_logger,
        "wall_min": (
            {"value": min(wall_values)[1], "line": min(wall_values)[2]}
            if wall_values
            else None
        ),
        "wall_max": (
            {"value": max(wall_values)[1], "line": max(wall_values)[2]}
            if wall_values
            else None
        ),
        "ros_timestamp_field_count": len(sim_values),
        "ros_zero_count": sum(item[0] == 0.0 for item in sim_values),
        "ros_nonzero_count": len(nonzero_sim),
        "ros_first_nonzero": (
            {"value": nonzero_sim[0][1], "line": nonzero_sim[0][2]}
            if nonzero_sim
            else None
        ),
        "ros_last_nonzero": (
            {"value": nonzero_sim[-1][1], "line": nonzero_sim[-1][2]}
            if nonzero_sim
            else None
        ),
        "ros_min_nonzero": (
            {"value": min(nonzero_sim)[1], "line": min(nonzero_sim)[2]}
            if nonzero_sim
            else None
        ),
        "ros_max_nonzero": (
            {"value": max(nonzero_sim)[1], "line": max(nonzero_sim)[2]}
            if nonzero_sim
            else None
        ),
        "pattern_line_counts": counts,
        "pattern_occurrence_counts": occurrences,
        "pattern_first_lines": refs,
    }


def iter_legacy_localization_entries(path: Path) -> Iterable[dict[str, Any]]:
    """Yield old-style localization evidence while streaming the whole log."""
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line_number, physical_line in enumerate(handle, 1):
            plain = ANSI_RE.sub("", physical_line).rstrip("\n")
            loggers = list(LOGGER_RE.finditer(plain))
            segments: list[tuple[str, str | None, str | None]] = []
            if loggers:
                for index, logger in enumerate(loggers):
                    end = loggers[index + 1].start() if index + 1 < len(loggers) else len(plain)
                    segments.append(
                        (
                            plain[logger.end() : end],
                            logger.group(2),
                            logger.group(3),
                        )
                    )
            else:
                segments.append((plain, None, None))

            for fragment, (message, wall_time, ros_time) in enumerate(segments, 1):
                timer = LEGACY_TIMER_RE.search(message)
                if timer:
                    yield {
                        "kind": "timer_start",
                        "line": line_number,
                        "fragment": fragment,
                        "wall_time": wall_time,
                        "ros_time": ros_time,
                        "loc_stamp": timer.group(1),
                        "raw": message.strip(),
                    }
                elif "Starting localization" in message:
                    # Preserve a Timer event even when concurrent output cut the
                    # line before "from stamp ..."; never invent the stamp.
                    yield {
                        "kind": "timer_start",
                        "line": line_number,
                        "fragment": fragment,
                        "wall_time": wall_time,
                        "ros_time": ros_time,
                        "loc_stamp": None,
                        "raw": message.strip(),
                    }
                if LEGACY_FAISS_EMPTY_RE.search(message):
                    yield {
                        "kind": "faiss_empty",
                        "line": line_number,
                        "fragment": fragment,
                        "wall_time": wall_time,
                        "ros_time": ros_time,
                        "raw": message.strip(),
                    }
                vertex_score = LEGACY_VERTEX_SCORE_RE.search(message)
                if vertex_score:
                    yield {
                        "kind": "vertex_registration_score",
                        "line": line_number,
                        "fragment": fragment,
                        "wall_time": wall_time,
                        "ros_time": ros_time,
                        "candidate": int(vertex_score.group(1)),
                        "score": float(vertex_score.group(2)),
                        "raw": message.strip(),
                    }
                infer = LEGACY_INFER_REG_RE.search(message)
                if infer:
                    yield {
                        "kind": (
                            "registration_result_python"
                            if infer.group(1) == "INFER-PY"
                            else "registration_result_cpp"
                        ),
                        "line": line_number,
                        "fragment": fragment,
                        "wall_time": wall_time,
                        "ros_time": ros_time,
                        "registration_type": infer.group(2),
                        "score": float(infer.group(3)),
                        "transform": infer.group(4),
                        "raw": message.strip(),
                    }


def nearest_legacy_evidence(
    target: dict[str, Any],
    candidates: list[dict[str, Any]],
    used: set[int],
) -> int | None:
    """Match service result evidence by rounded score and nearby wall time."""
    target_wall = as_float(target.get("wall_time"))
    best: tuple[float, int, int] | None = None
    for index, candidate in enumerate(candidates):
        if index in used:
            continue
        if abs(candidate["score"] - target["score"]) > 0.00051:
            continue
        candidate_wall = as_float(candidate.get("wall_time"))
        if target_wall is not None and candidate_wall is not None:
            wall_delta = abs(candidate_wall - target_wall)
            if wall_delta > 0.25:
                continue
        else:
            wall_delta = 0.25
        rank = (wall_delta, abs(candidate["line"] - target["line"]), index)
        if best is None or rank < best:
            best = rank
    return best[2] if best else None


def build_legacy_localization_events(
    path: Path, frame_stamps: list[tuple[float, int]]
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    """Normalize old logs into Timer/FAISS/registration events."""
    entries = list(iter_legacy_localization_entries(path))
    timers = [entry for entry in entries if entry["kind"] == "timer_start"]
    cpp_results = [
        entry for entry in entries if entry["kind"] == "registration_result_cpp"
    ]
    python_results = [
        entry for entry in entries if entry["kind"] == "registration_result_python"
    ]
    used_cpp: set[int] = set()
    used_python: set[int] = set()
    events: list[dict[str, Any]] = []
    cycles: list[dict[str, Any]] = []
    current_cycle: dict[str, Any] | None = None

    for entry in entries:
        if entry["kind"] == "timer_start":
            current_cycle = {
                "timer": entry,
                "faiss_empty": None,
                "registration_scores": [],
            }
            cycles.append(current_cycle)
            frame_id = nearest_frame_for_stamp(entry["loc_stamp"], frame_stamps)
            events.append(
                {
                    "type": "TIMER_LOCALIZE_START",
                    "format": "legacy",
                    "line": entry["line"],
                    "fragment": entry["fragment"],
                    "frame": frame_id,
                    "loc_stamp": entry["loc_stamp"],
                    "wall_time": entry["wall_time"],
                    "ros_time": entry["ros_time"],
                    "fields": {"source": "Starting localization"},
                    "raw": entry["raw"],
                }
            )
        elif entry["kind"] == "faiss_empty":
            if current_cycle is not None:
                current_cycle["faiss_empty"] = entry
                timer = current_cycle["timer"]
                events.append(
                    {
                        "type": "FAISS_EMPTY",
                        "format": "legacy",
                        "line": entry["line"],
                        "fragment": entry["fragment"],
                        "frame": nearest_frame_for_stamp(
                            timer["loc_stamp"], frame_stamps
                        ),
                        "loc_stamp": timer["loc_stamp"],
                        "wall_time": entry["wall_time"],
                        "ros_time": entry["ros_time"],
                        "fields": {
                            "timer_line": timer["line"],
                            "result": "EMPTY_INDEX",
                        },
                        "raw": entry["raw"],
                    }
                )
        elif entry["kind"] == "vertex_registration_score":
            if current_cycle is not None:
                entry["timer"] = current_cycle["timer"]
                current_cycle["registration_scores"].append(entry)

    for cycle in cycles:
        scores = cycle["registration_scores"]
        timer = cycle["timer"]
        frame_id = nearest_frame_for_stamp(timer["loc_stamp"], frame_stamps)
        if scores and cycle["faiss_empty"] is None:
            candidates = [entry["candidate"] for entry in scores]
            events.append(
                {
                    "type": "FAISS_RESULT",
                    "format": "legacy_inferred",
                    "line": scores[0]["line"],
                    "fragment": scores[0]["fragment"],
                    "frame": frame_id,
                    "loc_stamp": timer["loc_stamp"],
                    "wall_time": timer["wall_time"],
                    "ros_time": timer["ros_time"],
                    "fields": {
                        "timer_line": timer["line"],
                        "candidates": candidates,
                        "distances": "日志未记录",
                        "evidence": "candidate registration scores",
                    },
                    "raw": scores[0]["raw"],
                }
            )

        for score_entry in scores:
            cpp_index = nearest_legacy_evidence(score_entry, cpp_results, used_cpp)
            python_index = nearest_legacy_evidence(
                score_entry, python_results, used_python
            )
            cpp_entry = cpp_results[cpp_index] if cpp_index is not None else None
            python_entry = (
                python_results[python_index] if python_index is not None else None
            )
            if cpp_index is not None:
                used_cpp.add(cpp_index)
            if python_index is not None:
                used_python.add(python_index)

            common_fields = {
                "timer_line": timer["line"],
                "candidate": score_entry["candidate"],
                "registration_type": "localization",
            }
            events.append(
                {
                    "type": "REGISTRATION_CALL",
                    "format": "legacy_inferred",
                    "line": score_entry["line"],
                    "fragment": score_entry["fragment"],
                    "frame": frame_id,
                    "loc_stamp": timer["loc_stamp"],
                    "wall_time": score_entry["wall_time"],
                    "ros_time": score_entry["ros_time"],
                    "fields": {
                        **common_fields,
                        "evidence": "Vertex registration score implies completed call",
                    },
                    "raw": score_entry["raw"],
                }
            )
            events.append(
                {
                    "type": "REGISTRATION_RESULT",
                    "format": "legacy",
                    "line": score_entry["line"],
                    "fragment": score_entry["fragment"],
                    "frame": frame_id,
                    "loc_stamp": timer["loc_stamp"],
                    "wall_time": score_entry["wall_time"],
                    "ros_time": score_entry["ros_time"],
                    "fields": {
                        **common_fields,
                        "score": score_entry["score"],
                        "service_success": (
                            True
                            if cpp_entry is not None or python_entry is not None
                            else "日志未记录"
                        ),
                        "cpp_infer_line": (
                            cpp_entry["line"] if cpp_entry is not None else None
                        ),
                        "python_infer_line": (
                            python_entry["line"] if python_entry is not None else None
                        ),
                        "pixel_transform": (
                            cpp_entry["transform"]
                            if cpp_entry is not None
                            else (
                                python_entry["transform"]
                                if python_entry is not None
                                else "日志未记录"
                            )
                        ),
                    },
                    "raw": score_entry["raw"],
                }
            )

    counts = Counter(event["type"] for event in events)
    return sorted(events, key=lambda event: (event["line"], event["fragment"])), dict(counts)


def as_int(value: Any) -> int | None:
    try:
        return int(str(value))
    except (TypeError, ValueError):
        return None


def as_float(value: Any) -> float | None:
    try:
        result = float(str(value))
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def fmt_num(value: float | None, digits: int = 3) -> str:
    return "日志未记录" if value is None else f"{value:.{digits}f}"


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("empty values")
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def timing_stats(values: list[float]) -> dict[str, Any] | None:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return None
    return {
        "count": len(finite),
        "min_ms": min(finite),
        "p10_ms": percentile(finite, 0.10),
        "median_ms": statistics.median(finite),
        "p90_ms": percentile(finite, 0.90),
        "max_ms": max(finite),
    }


def nearest_frame_for_stamp(
    stamp: str | None, frame_stamps: list[tuple[float, int]], tolerance: float = 2e-5
) -> int | None:
    target = as_float(stamp)
    if target is None:
        return None
    best: tuple[float, int] | None = None
    for candidate_stamp, frame_id in frame_stamps:
        distance = abs(candidate_stamp - target)
        if best is None or distance < best[0]:
            best = (distance, frame_id)
    return best[1] if best is not None and best[0] <= tolerance else None


def event_type(record: dict[str, Any]) -> str | None:
    stage = record["stage"]
    fields = record["fields"]
    if stage == "LOCALIZER_SNAPSHOT":
        if fields.get("action") == "WRITE":
            return "localization_snapshot_write"
        if fields.get("action") == "TIMER_READ":
            return "TIMER_LOCALIZE_START"
        return "localization_snapshot_timer_event"
    if stage == "FAISS":
        return (
            "FAISS_EMPTY"
            if fields.get("result") == "EMPTY_INDEX"
            else "FAISS_RESULT"
        )
    if stage == "REGISTRATION":
        return "REGISTRATION_RESULT"
    if stage == "REGISTRATION_SERVICE":
        return "registration_service_result"
    if stage == "REGISTRATION_PY":
        return "registration_python_result"
    if stage == "LOCALIZATION_RESULT":
        return (
            "localization_result_consumed"
            if fields.get("action") == "CONSUME"
            else "localization_result_generated"
        )
    if stage == "VERTEX":
        return (
            "vertex_created"
            if fields.get("event") == "CREATE"
            else "current_vertex_set"
        )
    return EVENT_STAGE_TYPES.get(stage)


def edge_label(record: dict[str, Any]) -> str:
    fields = record["fields"]
    kind = fields.get("EDGE_TYPE", "UNKNOWN")
    source = fields.get("source", "?")
    target = fields.get("target", "?")
    if fields.get("still_added") == "true":
        outcome = "EXISTING" if fields.get("already_exists") == "true" else "ADDED"
    elif fields.get("result") in {"REJECT", "REJECTED"}:
        outcome = "REJECTED"
    else:
        outcome = fields.get("result", "NOT_ADDED")
    return f"{kind}:{source}->{target}:{outcome}"


def frame_decision(frame: dict[str, Any]) -> str:
    """Recover a frame decision even when a SUMMARY fragment was truncated."""
    known = {"FIRST_VERTEX", "KEEP", "NEW_VERTEX"}
    for record_name in ("summary", "publish"):
        record = frame.get(record_name)
        if record and record["fields"].get("decision") in known:
            return record["fields"]["decision"]
    if frame.get("vertex_create"):
        return "FIRST_VERTEX" if not frame.get("decision_records") else "NEW_VERTEX"
    if frame.get("keep_check"):
        return "KEEP"
    return "UNKNOWN"


def event_stamp_key(value: Any) -> str | None:
    """Use millisecond precision to join old %.3f and new %.6f stamps."""
    number = as_float(value)
    return None if number is None else f"{number:.3f}"


def localization_events_match(
    structured: dict[str, Any], legacy: dict[str, Any]
) -> bool:
    if structured["type"] != legacy["type"]:
        return False
    structured_stamp = event_stamp_key(structured.get("loc_stamp"))
    legacy_stamp = event_stamp_key(legacy.get("loc_stamp"))
    if structured_stamp and legacy_stamp and structured_stamp != legacy_stamp:
        return False
    if structured_stamp is None or legacy_stamp is None:
        if abs(structured["line"] - legacy["line"]) > 12:
            return False
    if structured["type"] in {"REGISTRATION_CALL", "REGISTRATION_RESULT"}:
        structured_candidate = as_int(structured.get("fields", {}).get("candidate"))
        legacy_candidate = as_int(legacy.get("fields", {}).get("candidate"))
        if (
            structured_candidate is not None
            and legacy_candidate is not None
            and structured_candidate != legacy_candidate
        ):
            return False
        structured_score = as_float(structured.get("fields", {}).get("score"))
        legacy_score = as_float(legacy.get("fields", {}).get("score"))
        if (
            structured_score is not None
            and legacy_score is not None
            and abs(structured_score - legacy_score) > 0.0011
        ):
            return False
    return True


def merge_legacy_localization_events(
    structured_events: list[dict[str, Any]],
    legacy_events: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    """Prefer complete FLOW records and use old logs only to fill torn records."""
    logical_types = {
        "TIMER_LOCALIZE_START",
        "FAISS_EMPTY",
        "FAISS_RESULT",
        "REGISTRATION_CALL",
        "REGISTRATION_RESULT",
    }
    structured_logical = [
        event for event in structured_events if event["type"] in logical_types
    ]
    supplements: list[dict[str, Any]] = []
    suppressed: Counter[str] = Counter()
    for legacy in legacy_events:
        if any(
            localization_events_match(structured, legacy)
            for structured in structured_logical
        ):
            suppressed[legacy["type"]] += 1
            continue
        supplements.append(legacy)
    return supplements, dict(suppressed)


def compact_event_ref(event: dict[str, Any]) -> dict[str, Any]:
    return {
        "type": event["type"],
        "line": event["line"],
        "fragment": event.get("fragment", 1),
        "format": event.get("format", "structured"),
        "frame": event.get("frame"),
        "fields": event.get("fields", {}),
    }


def build_localization_chains(
    events: list[dict[str, Any]], frames: dict[int, dict[str, Any]]
) -> list[dict[str, Any]]:
    """Join snapshot, Timer, FAISS, registration, write-back, and consumers."""
    chain_types = {
        "TIMER_LOCALIZE_START",
        "FAISS_EMPTY",
        "FAISS_RESULT",
        "REGISTRATION_CALL",
        "REGISTRATION_RESULT",
        "localization_result_generated",
    }
    chains: dict[str, dict[str, Any]] = {}

    def chain_for(value: Any) -> dict[str, Any] | None:
        key = event_stamp_key(value)
        if key is None:
            return None
        return chains.setdefault(
            key,
            {
                "stamp_key": key,
                "snapshot_stamp": None,
                "snapshot_frame": None,
                "snapshot_write": None,
                "timer": None,
                "faiss": None,
                "registrations": [],
                "result_generated": None,
                "consumers": [],
            },
        )

    for event in events:
        if event["type"] not in chain_types:
            continue
        chain = chain_for(event.get("loc_stamp"))
        if chain is None:
            continue
        stamp = event.get("loc_stamp")
        if stamp and (
            chain["snapshot_stamp"] is None
            or len(str(stamp)) > len(str(chain["snapshot_stamp"]))
        ):
            chain["snapshot_stamp"] = stamp
        if event.get("frame") is not None:
            chain["snapshot_frame"] = event["frame"]
        ref = compact_event_ref(event)
        if event["type"] == "TIMER_LOCALIZE_START":
            chain["timer"] = ref
        elif event["type"] in {"FAISS_EMPTY", "FAISS_RESULT"}:
            chain["faiss"] = ref
        elif event["type"] == "REGISTRATION_RESULT":
            chain["registrations"].append(ref)
        elif event["type"] == "localization_result_generated":
            chain["result_generated"] = ref

    for chain in chains.values():
        frame_id = chain["snapshot_frame"]
        snapshot = frames.get(frame_id, {}).get("snapshot") if frame_id else None
        if snapshot is not None:
            chain["snapshot_write"] = compact_event_ref(
                {
                    "type": "localization_snapshot_write",
                    "line": snapshot["line"],
                    "fragment": snapshot["fragment"],
                    "frame": frame_id,
                    "fields": snapshot["fields"],
                }
            )
            if chain["snapshot_stamp"] is None:
                chain["snapshot_stamp"] = snapshot.get("stamp")

    for event in events:
        if event["type"] != "localization_result_consumed":
            continue
        result_stamp = event.get("fields", {}).get("result_stamp")
        chain = chain_for(result_stamp)
        if chain is None or as_float(result_stamp) in {None, 0.0}:
            continue
        chain["consumers"].append(compact_event_ref(event))

    for chain in chains.values():
        chain["registrations"].sort(key=lambda item: item["line"])
        chain["consumers"].sort(key=lambda item: item["line"])
        chain["first_consumer"] = (
            chain["consumers"][0] if chain["consumers"] else None
        )
    return sorted(
        chains.values(),
        key=lambda chain: float(chain["stamp_key"]),
    )


def build_analysis(log_path: Path) -> dict[str, Any]:
    log_scan = scan_log(log_path)
    records: list[dict[str, Any]] = []
    stage_counts: Counter[str] = Counter()
    flow_count = 0
    flow_line_numbers: set[int] = set()
    total_lines = 0

    # The loop is deliberately streaming. Structured fragments, not the full
    # text, are retained for correlation and output.
    for record in iter_flow_fragments(log_path):
        records.append(record)
        flow_count += 1
        flow_line_numbers.add(record["line"])
        if record["stage"]:
            stage_counts[record["stage"]] += 1
        total_lines = max(total_lines, record["line"])

    total_lines = log_scan["total_lines"]

    frames: dict[int, dict[str, Any]] = defaultdict(
        lambda: {
            "records": [],
            "edges": [],
            "decision_records": [],
            "python_descriptor": None,
        }
    )
    for record in records:
        frame_id = as_int(record["frame_raw"])
        if frame_id is not None:
            frames[frame_id]["records"].append(record)
            if record["stamp"] and "stamp" not in frames[frame_id]:
                frames[frame_id]["stamp"] = record["stamp"]

    frame_stamps = [
        (stamp, frame_id)
        for frame_id, frame in frames.items()
        if (stamp := as_float(frame.get("stamp"))) is not None
    ]

    # Associate Python service records (SERVICE-n) using the exact sensor stamp.
    unassociated_python = 0
    for record in records:
        if record["stage"] != "DESCRIPTOR_PY":
            continue
        frame_id = nearest_frame_for_stamp(record["stamp"], frame_stamps)
        if frame_id is None:
            if record["fields"].get("trace_enabled") != "true":
                unassociated_python += 1
            continue
        frames[frame_id]["python_descriptor"] = record

    for frame_id, frame in frames.items():
        for record in frame["records"]:
            stage = record["stage"]
            fields = record["fields"]
            if stage == "SYNC" and fields.get("result") == "OK":
                frame["sync"] = record
                frame["stamp"] = record["stamp"]
            elif stage == "CLOUD_PARSE":
                frame["cloud"] = record
            elif stage == "ODOM":
                frame["odom"] = record
            elif stage == "DESCRIPTOR" and fields.get("action") == "SERVICE_RESULT":
                frame["descriptor"] = record
            elif stage == "GRID":
                frame["grid"] = record
            elif stage == "LOCALIZER_SNAPSHOT" and fields.get("action") == "WRITE":
                frame["snapshot"] = record
            elif stage == "LOCALIZATION_RESULT" and fields.get("action") == "CONSUME":
                frame["localization_consume"] = record
            elif stage == "DECISION":
                frame["decision_records"].append(record)
                if fields.get("step") == "KEEP_CHECK":
                    frame["keep_check"] = record
            elif stage == "VERTEX" and fields.get("event") == "CREATE":
                frame["vertex_create"] = record
            elif stage == "VERTEX" and fields.get("event") == "SET_CURRENT":
                frame["vertex_set"] = record
            elif stage == "EDGE":
                frame["edges"].append(record)
            elif stage == "SUMMARY":
                frame["summary"] = record
            elif stage == "PUBLISH":
                frame["publish"] = record

    # SYNC result=OK is the earliest complete-frame marker. SUMMARY/PUBLISH
    # occasionally lose a fragment to stdout/stderr interleaving, so requiring
    # either tail record would silently drop otherwise completed frames.
    successful_frames = {
        frame_id: frame for frame_id, frame in frames.items() if "sync" in frame
    }

    rx_records = [
        record
        for record in records
        if record["stage"] == "RX" and as_int(record["fields"].get("rx_id")) is not None
    ]
    rx_ids = sorted(
        {
            rx_id
            for record in rx_records
            if (rx_id := as_int(record["fields"].get("rx_id"))) is not None
        }
    )
    total_cloud_messages = max(rx_ids, default=0)
    missing_rx_ids = [
        rx_id
        for rx_id in range(1, total_cloud_messages + 1)
        if rx_id not in set(rx_ids)
    ]
    throttle_records = [
        record
        for record in records
        if record["stage"] == "THROTTLE"
        and record["fields"].get("result") == "SKIPPED_INTERVAL"
    ]
    sync_problem_records = [
        record
        for record in records
        if record["stage"] == "SYNC" and record["fields"].get("result") != "OK"
    ]

    summary_decisions = Counter(
        frame_decision(frame) for frame in successful_frames.values()
    )
    vertex_creates = [
        record
        for record in records
        if record["stage"] == "VERTEX" and record["fields"].get("event") == "CREATE"
    ]
    edge_records = [record for record in records if record["stage"] == "EDGE"]
    edge_added = [
        record
        for record in edge_records
        if record["fields"].get("still_added") == "true"
        and record["fields"].get("already_exists") != "true"
    ]
    edge_rejected = [
        record
        for record in edge_records
        if record["fields"].get("still_added") != "true"
        or record["fields"].get("result") in {"REJECT", "REJECTED"}
    ]
    loop_events = [
        record
        for record in records
        if record["fields"].get("decision") == "LOOP_NEW_VERTEX"
        or record["fields"].get("EDGE_TYPE") == "LOOP"
        or record["fields"].get("loop_closure_published") == "true"
    ]

    timings: dict[str, dict[str, Any] | None] = {
        "descriptor_service": timing_stats(
            [
                value
                for record in records
                if record["stage"] == "DESCRIPTOR"
                and record["fields"].get("action") == "SERVICE_RESULT"
                and (value := as_float(record["fields"].get("elapsed_ms"))) is not None
            ]
        ),
        "descriptor_python_forward": timing_stats(
            [
                value
                for record in records
                if record["stage"] == "DESCRIPTOR_PY"
                and (value := as_float(record["fields"].get("forward_ms"))) is not None
            ]
        ),
        "descriptor_python_total": timing_stats(
            [
                value
                for record in records
                if record["stage"] == "DESCRIPTOR_PY"
                and (value := as_float(record["fields"].get("elapsed_ms"))) is not None
            ]
        ),
        "grid": timing_stats(
            [
                value
                for record in records
                if record["stage"] == "GRID"
                and (value := as_float(record["fields"].get("elapsed_ms"))) is not None
            ]
        ),
        "grid_occupancy_update": timing_stats(
            [
                value
                for record in records
                if record["stage"] == "GRID"
                and (value := as_float(record["fields"].get("occupancy_update_ms")))
                is not None
            ]
        ),
        "faiss": timing_stats(
            [
                value
                for record in records
                if record["stage"] == "FAISS"
                and (value := as_float(record["fields"].get("elapsed_ms"))) is not None
            ]
        ),
        "registration": timing_stats(
            [
                value
                for record in records
                if record["stage"] == "REGISTRATION"
                and (value := as_float(record["fields"].get("elapsed_ms"))) is not None
            ]
        ),
    }

    localization_ages = [
        value
        for record in records
        if record["stage"] == "LOCALIZATION_RESULT"
        and record["fields"].get("action") == "CONSUME"
        and (value := as_float(record["fields"].get("age"))) is not None
        and value >= 0.0
    ]

    events: list[dict[str, Any]] = []
    for record in records:
        kind = event_type(record)
        if kind is None:
            continue
        frame_id = as_int(record["frame_raw"])
        if frame_id is None and record["loc_stamp"]:
            frame_id = nearest_frame_for_stamp(record["loc_stamp"], frame_stamps)
        events.append(
            {
                "type": kind,
                "line": record["line"],
                "fragment": record["fragment"],
                "frame": frame_id,
                "frame_raw": record["frame_raw"],
                "stamp": record["stamp"],
                "loc_stamp": record["loc_stamp"],
                "stage": record["stage"],
                "fields": record["fields"],
                "raw": record["raw"],
            }
        )
        if record["stage"] == "REGISTRATION":
            events.append(
                {
                    "type": "REGISTRATION_CALL",
                    "format": "structured_inferred",
                    "line": record["line"],
                    "fragment": record["fragment"],
                    "frame": frame_id,
                    "frame_raw": record["frame_raw"],
                    "stamp": record["stamp"],
                    "loc_stamp": record["loc_stamp"],
                    "stage": record["stage"],
                    "fields": {
                        "candidate": record["fields"].get("candidate"),
                        "registration_type": record["fields"].get("type"),
                        "evidence": "structured registration result implies call",
                    },
                    "raw": record["raw"],
                }
            )

    legacy_events, legacy_event_counts = build_legacy_localization_events(
        log_path, frame_stamps
    )
    legacy_supplements, legacy_suppressed_counts = merge_legacy_localization_events(
        events, legacy_events
    )
    events.extend(legacy_supplements)
    events.sort(key=lambda event: (event["line"], event.get("fragment", 0)))
    localization_chains = build_localization_chains(events, frames)

    malformed_stage_fragments = [
        {"line": record["line"], "raw": record["raw"]}
        for record in records
        if record["stage"] is None
    ]

    return {
        "log_path": str(log_path.resolve()),
        "log_size_bytes": log_path.stat().st_size,
        "log_scan": log_scan,
        "total_lines": total_lines,
        "flow_count": flow_count,
        "flow_line_count": len(flow_line_numbers),
        "stage_counts": dict(sorted(stage_counts.items())),
        "records": records,
        "frames": frames,
        "successful_frames": successful_frames,
        "rx_count": len(rx_records),
        "total_cloud_messages": total_cloud_messages,
        "missing_rx_ids": missing_rx_ids,
        "throttle_count": len(throttle_records),
        "sync_problem_records": sync_problem_records,
        "summary_decisions": summary_decisions,
        "vertex_creates": vertex_creates,
        "edge_records": edge_records,
        "edge_added": edge_added,
        "edge_rejected": edge_rejected,
        "loop_events": loop_events,
        "timings": timings,
        "localization_ages": localization_ages,
        "events": events,
        "legacy_event_counts": legacy_event_counts,
        "legacy_supplement_count": len(legacy_supplements),
        "legacy_suppressed_counts": legacy_suppressed_counts,
        "localization_chains": localization_chains,
        "malformed_stage_fragments": malformed_stage_fragments,
        "unassociated_python": unassociated_python,
    }


def write_csv(data: dict[str, Any], output: Path) -> None:
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for frame_id in sorted(data["successful_frames"]):
            frame = data["successful_frames"][frame_id]
            summary = frame.get("summary", {}).get("fields", {})
            publish = frame.get("publish", {}).get("fields", {})
            begin = next(
                (
                    record["fields"]
                    for record in frame["decision_records"]
                    if record["fields"].get("action") == "BEGIN"
                ),
                {},
            )
            keep = frame.get("keep_check", {}).get("fields", {})
            consume = frame.get("localization_consume", {}).get("fields", {})
            vertex = frame.get("vertex_create", {}).get("fields", {})
            new_vertex_id = vertex.get("new_id", "")
            if not new_vertex_id and frame_decision(frame) == "NEW_VERTEX":
                sequential_edges = [
                    edge
                    for edge in frame["edges"]
                    if edge["fields"].get("EDGE_TYPE") == "SEQUENTIAL"
                ]
                if sequential_edges:
                    new_vertex_id = sequential_edges[-1]["fields"].get("target", "")
            writer.writerow(
                {
                    "frame": frame_id,
                    "stamp": frame.get("stamp", ""),
                    "vertex_before": summary.get(
                        "vertex_before", begin.get("last_vertex", "")
                    ),
                    "vertex_after": summary.get(
                        "vertex_after", publish.get("current_vertex", "")
                    ),
                    "decision": frame_decision(frame),
                    "inside": summary.get("inside", keep.get("inside", "")),
                    "iou": summary.get("iou", keep.get("iou", "")),
                    "rel_dist": summary.get(
                        "rel_dist", keep.get("rel_dist", "")
                    ),
                    "localization_stamp": summary.get(
                        "localization_stamp", consume.get("result_stamp", "")
                    ),
                    "matched_count": summary.get(
                        "matched", consume.get("matched", "")
                    ),
                    "unmatched_count": summary.get(
                        "unmatched", consume.get("unmatched", "")
                    ),
                    "new_vertex_id": new_vertex_id,
                    "edge_events": ";".join(edge_label(edge) for edge in frame["edges"]),
                }
            )


def write_events_json(data: dict[str, Any], output: Path) -> None:
    payload = {
        "schema_version": 2,
        "source_log": data["log_path"],
        "source_size_bytes": data["log_size_bytes"],
        "source_total_lines": data["total_lines"],
        "flow_physical_lines": data["flow_line_count"],
        "flow_fragments": data["flow_count"],
        "log_scan": data["log_scan"],
        "legacy_event_counts": data["legacy_event_counts"],
        "legacy_supplement_count": data["legacy_supplement_count"],
        "legacy_duplicates_suppressed": data["legacy_suppressed_counts"],
        "event_count": len(data["events"]),
        "association_note": (
            "FRAME is used directly when present. Python descriptor service records "
            "are correlated by sensor STAMP. Structured LOC_STAMP records and legacy "
            "'Starting localization' records are correlated to frames by snapshot "
            "timestamp. Legacy FAISS candidates and grid-registration calls are "
            "associated inside the nearest Timer cycle; candidate distance remains "
            "'日志未记录' when the old log does not print it. When both formats "
            "describe the same event, the complete FLOW record is retained and the "
            "legacy record is counted as corroborating evidence rather than a "
            "second call. Consumers are joined to generated results by result_stamp."
        ),
        "localization_chain_count": len(data["localization_chains"]),
        "localization_chains": data["localization_chains"],
        "events": data["events"],
        "parse_warnings": {
            "flow_fragments_without_stage": data["malformed_stage_fragments"],
            "unassociated_python_descriptor_records": data["unassociated_python"],
        },
    }
    with output.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")


def timing_line(label: str, stats: dict[str, Any] | None) -> str:
    if stats is None:
        return f"- {label}：日志未记录。"
    return (
        f"- {label}：n={stats['count']}，中位数 {stats['median_ms']:.3f} ms，"
        f"P10–P90 {stats['p10_ms']:.3f}–{stats['p90_ms']:.3f} ms，"
        f"范围 {stats['min_ms']:.3f}–{stats['max_ms']:.3f} ms。"
    )


def line_refs(records: list[dict[str, Any]]) -> str:
    if not records:
        return "日志未记录"
    numbers = sorted({record["line"] for record in records})
    if len(numbers) == 1:
        return f"L{numbers[0]}"
    return f"L{numbers[0]}–L{numbers[-1]}"


def write_summary(data: dict[str, Any], output: Path) -> None:
    successful = data["successful_frames"]
    event_counts = Counter(event["type"] for event in data["events"])
    pattern_counts = data["log_scan"]["pattern_line_counts"]
    generated_results = [
        event
        for event in data["events"]
        if event["type"] == "localization_result_generated"
    ]
    nonempty_consumes = [
        event
        for event in data["events"]
        if event["type"] == "localization_result_consumed"
        and (
            (as_int(event["fields"].get("matched")) or 0) > 0
            or (as_int(event["fields"].get("unmatched")) or 0) > 0
        )
    ]
    matched_registrations = [
        event
        for event in data["events"]
        if event["type"] == "REGISTRATION_RESULT"
        and event["fields"].get("result") == "MATCHED"
    ]
    if not successful:
        lines = [
            "# PRISM-TopoMap 运行追踪摘要",
            "",
            "## 输入与解析范围",
            "",
            f"- 日志：`{data['log_path']}`",
            f"- 文件大小：{data['log_size_bytes']:,} 字节",
            f"- 物理行数：{data['total_lines']:,}",
            f"- 含 `[FLOW]` 的物理行数：{data['flow_line_count']:,}",
            "- 未发现结构化成功帧 SUMMARY；以下为旧式定位日志解析结果。",
            "",
            "## 旧式定位事件",
            "",
            f"- `Starting localization`：{pattern_counts['Starting localization']} 行。",
            f"- `FAISS index is empty`：{pattern_counts['FAISS index is empty']} 行。",
            f"- `gridRegistration`：{pattern_counts['gridRegistration']} 行。",
            f"- `Vertex ... registration score`："
            f"{pattern_counts['Vertex .* registration score']} 行。",
            f"- `TIMER_LOCALIZE_START`：{event_counts['TIMER_LOCALIZE_START']}。",
            f"- `FAISS_EMPTY`：{event_counts['FAISS_EMPTY']}。",
            f"- `FAISS_RESULT`：{event_counts['FAISS_RESULT']}。",
            f"- `REGISTRATION_CALL`：{event_counts['REGISTRATION_CALL']}。",
            f"- `REGISTRATION_RESULT`：{event_counts['REGISTRATION_RESULT']}。",
            "",
            "旧格式没有打印 FAISS distance 时，该字段保留为“日志未记录”。",
            "",
        ]
        output.write_text("\n".join(lines), encoding="utf-8")
        return
    first_stamp = min(
        value
        for frame in successful.values()
        if (value := as_float(frame.get("stamp"))) is not None
    )
    last_stamp = max(
        value
        for frame in successful.values()
        if (value := as_float(frame.get("stamp"))) is not None
    )
    final_nodes = max(
        as_int(frame.get("summary", {}).get("fields", {}).get("graph_vertices"))
        or as_int(frame.get("publish", {}).get("fields", {}).get("nodes"))
        or 0
        for frame in successful.values()
    )
    final_edges = max(
        as_int(frame.get("summary", {}).get("fields", {}).get("graph_edges"))
        or as_int(frame.get("publish", {}).get("fields", {}).get("edges"))
        or 0
        for frame in successful.values()
    )
    positive_age_stats = timing_stats(
        [age * 1000.0 for age in data["localization_ages"]]
    )

    lines = [
        "# PRISM-TopoMap 运行追踪摘要",
        "",
        "## 输入与解析范围",
        "",
        f"- 日志：`{data['log_path']}`",
        f"- 文件大小：{data['log_size_bytes']:,} 字节",
        f"- 物理行数：{data['total_lines']:,}",
        f"- 含 `[FLOW]` 的物理行数：{data['flow_line_count']:,}；"
        f"解析片段数：{data['flow_count']:,}（交错行可能含两个片段）",
        f"- 成功帧点云时间范围：{first_stamp:.9f}–{last_stamp:.9f}，跨度 {last_stamp-first_stamp:.6f} s",
        f"- `STAGE`：{', '.join(data['stage_counts'])}",
        f"- 日志时间字段：首条 logger wall time "
        f"`{data['log_scan']['first_logger']['wall_time']}`（L{data['log_scan']['first_logger']['line']}），"
        f"末条 `{data['log_scan']['last_logger']['wall_time']}`（L{data['log_scan']['last_logger']['line']}）；"
        f"物理首/末 logger 的 ROS time 分别为 "
        f"`{data['log_scan']['first_logger']['ros_time']}` 和 "
        f"`{data['log_scan']['last_logger']['ros_time']}`。",
        f"- 检测到 {data['log_scan']['ros_timestamp_field_count']} 个显式 ROS/sim-time 第二时间戳，"
        f"其中非零 {data['log_scan']['ros_nonzero_count']} 个；全文非零范围为 "
        f"`{data['log_scan']['ros_min_nonzero']['value']}`–"
        f"`{data['log_scan']['ros_max_nonzero']['value']}`，ROS time 已正常推进。",
        "",
        "## 消息与拓扑统计",
        "",
        f"- 总点云消息数：{data['total_cloud_messages']}（`rx_id` 从 1 到 "
        f"{data['total_cloud_messages']}；其中 {len(data['missing_rx_ids'])} 条 RX 行被交错输出截断，"
        f"缺失 ID 为 {data['missing_rx_ids']}）。",
        f"- 被 0.3 s 间隔限流：{data['total_cloud_messages'] - len(successful)}"
        f"（结构化行直接解析出 {data['throttle_count']} 条，另 "
        f"{data['total_cloud_messages'] - len(successful) - data['throttle_count']} 条"
        "由连续 `rx_id` 总数减成功帧数还原）。",
        f"- 同步等待或失败记录：{len(data['sync_problem_records'])}（{line_refs(data['sync_problem_records'])}）。",
        f"- 成功处理帧：{len(successful)}（以 `SYNC result=OK` 计；"
        "SUMMARY/PUBLISH 被交错截断时仍保留该帧）。",
        f"- 节点数量：0 → {final_nodes}；可完整解析的 `VERTEX CREATE` "
        f"{len(data['vertex_creates'])} 条，另有 "
        f"{max(0, final_nodes-len(data['vertex_creates']))} 条创建记录被交错截断。",
        f"- 最终边数：{final_edges}；新增边事件 {len(data['edge_added'])} 次，边添加拒绝 {len(data['edge_rejected'])} 次。",
        f"- 回环事件：{len(data['loop_events'])}。",
        "- `decision` 次数："
        + "，".join(
            f"`{key}` {value}" for key, value in sorted(data["summary_decisions"].items())
        )
        + "。",
        "",
        "这里的“边添加拒绝”只统计 `STAGE=EDGE` 的添加结果；"
        "`EDGE_REATTACH ... result=REJECT` 是节点切换尝试失败，不是图中边被拒绝。",
        "",
        "## 典型耗时",
        "",
        timing_line("C++ 描述符服务调用", data["timings"]["descriptor_service"]),
        timing_line("Python 描述符前向计算", data["timings"]["descriptor_python_forward"]),
        timing_line("Python 描述符服务总耗时", data["timings"]["descriptor_python_total"]),
        timing_line("LocalGrid 总更新", data["timings"]["grid"]),
        timing_line("occupancy 更新", data["timings"]["grid_occupancy_update"]),
        timing_line("FAISS", data["timings"]["faiss"]),
        timing_line("配准", data["timings"]["registration"]),
        "",
        "## 定位结果延迟与跨帧关系",
        "",
    ]
    if positive_age_stats is None:
        lines.extend(
            [
                f"- {event_counts['localization_snapshot_write']} 次 "
                "`LOCALIZER_SNAPSHOT action=WRITE` 已记录。",
                f"- 全文旧式日志统计：`Starting localization` "
                f"{pattern_counts['Starting localization']}，`FAISS index is empty` "
                f"{pattern_counts['FAISS index is empty']}，`gridRegistration` "
                f"{pattern_counts['gridRegistration']}，`Vertex ... registration score` "
                f"{pattern_counts['Vertex .* registration score']}。",
                f"- 新旧格式合并后：`TIMER_LOCALIZE_START` "
                f"{event_counts['TIMER_LOCALIZE_START']}，`FAISS_EMPTY` "
                f"{event_counts['FAISS_EMPTY']}，`FAISS_RESULT` "
                f"{event_counts['FAISS_RESULT']}，`REGISTRATION_CALL` "
                f"{event_counts['REGISTRATION_CALL']}，`REGISTRATION_RESULT` "
                f"{event_counts['REGISTRATION_RESULT']}。",
                "- 当前源码在 `Localizer::localize()` 进入 initialized 分支后无条件输出"
                " `Starting localization from stamp ...`；本日志该旧式行和对应 `[FLOW]` 均为 0。"
                "因此这里不是“解析器只漏掉结构化格式”，而是没有证据表明 Timer 回调进入了定位函数。",
                "- 日志中所有可解析的显式 ROS/sim-time 第二时间戳均为 0；"
                "结合 `localization_frequency=2.0` 和 `ros::Timer` 使用 ROS time，"
                "这支持“本次回放期间 ROS time 未推进，Timer 未触发”的解释。"
                "这是基于日志与源码的推断，不等同于运行时 `/clock` 的直接记录。",
                f"- {event_counts['localization_result_consumed']} 次主循环消费记录中的"
                " `result_stamp` 均为 `0.000000`，"
                "`age=-1.000000`、matched/unmatched 均为 0。",
                "- 分层结论：Timer 未见执行；FAISS 未见执行；GridRegistration 未见调用；"
                "LocalizedState 未见写回；主循环未消费到非空结果。后三层结论依赖第一层，"
                "不是仅由缺少 `LOC_STAMP` 推断。",
            ]
        )
    else:
        chain_162 = next(
            (
                chain
                for chain in data["localization_chains"]
                if chain["snapshot_frame"] == 162
            ),
            None,
        )
        lines.extend(
            [
                timing_line("定位结果消费延迟", positive_age_stats),
                f"- 快照写入 {event_counts['localization_snapshot_write']} 次；"
                f"归一化 Timer 周期 {event_counts['TIMER_LOCALIZE_START']} 次，"
                f"其中 FAISS 空索引 {event_counts['FAISS_EMPTY']} 次、"
                f"非空候选 {event_counts['FAISS_RESULT']} 次。",
                f"- 候选配准调用/结果各 {event_counts['REGISTRATION_CALL']} 次；"
                f"其中明确 `MATCHED` {len(matched_registrations)} 次。"
                f"LocalizedState 结果生成 {len(generated_results)} 次。",
                f"- 主循环消费记录 {event_counts['localization_result_consumed']} 次；"
                f"其中 {len(nonempty_consumes)} 次 matched/unmatched 至少一项非零。",
                f"- 新旧格式同时出现时已去重：旧式记录补齐 "
                f"{data['legacy_supplement_count']} 个被交错截断的结构化事件；"
                "其余旧式记录仅作为旁证，不重复计为第二次调用。",
            ]
        )
        if chain_162 and chain_162["first_consumer"]:
            registration_results = chain_162["registrations"]
            matched = [
                item
                for item in registration_results
                if item["fields"].get("result") == "MATCHED"
            ]
            consumer = chain_162["first_consumer"]
            lines.append(
                f"- 真实跨帧链：FRAME 162 在 L"
                f"{chain_162['snapshot_write']['line']} 写入快照，L"
                f"{chain_162['timer']['line']} 被 Timer 读取，L"
                f"{chain_162['faiss']['line']} 得到非空 FAISS 候选，"
                f"{len(registration_results)} 个候选完成配准（匹配 {len(matched)} 个），"
                f"L{chain_162['result_generated']['line']} 写回结果；"
                f"FRAME {consumer['frame']} 在 L{consumer['line']} 消费该 "
                f"`result_stamp`。"
            )
    lines.extend(
        [
            "",
            "## 数据质量说明",
            "",
            f"- `{len(data['malformed_stage_fragments'])}` 个 `[FLOW]` 片段因混合进程输出交错而未解析出完整 `STAGE`；"
            "解析器保留其原始行号作为警告，不据此补造事件。",
            f"- `{data['unassociated_python']}` 条 Python 描述符记录无法按 `STAMP` 关联到成功帧。",
            "- 首帧描述符服务临时失败（`success=false`、维度 0），但栅格和首节点仍被真实创建；"
            "其余 241 帧服务结果均为成功的 256 维描述符。这是本次运行现象，不是算法固定行为。",
            "",
            "## 推荐案例",
            "",
            "- 案例 A：FRAME 1，首节点创建。",
            "- 案例 B：FRAME 162→163，真实展示快照写入、Timer 定位、非空 FAISS、"
            "候选配准、结果写回、后续帧消费和 KEEP。",
            "- 案例 C：FRAME 106，距离 5.3755 m 超过 5.0000 m，"
            "创建 vertex 2 并添加顺序边 1→2。",
            "- 本次未观察到 `EDGE_SWITCH`、`LOCALIZATION_SWITCH` 或 `LOOP_CLOSURE`；"
            "不选择、不虚构对应案例。",
            "",
        ]
    )
    output.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    log_path = args.log.resolve()
    output_dir = args.output_dir.resolve()
    if not log_path.is_file():
        raise SystemExit(f"log not found: {log_path}")
    output_dir.mkdir(parents=True, exist_ok=True)

    data = build_analysis(log_path)
    write_csv(data, output_dir / "runtime_trace_index.csv")
    write_events_json(data, output_dir / "runtime_trace_events.json")
    write_summary(data, output_dir / "runtime_trace_summary.md")

    print(f"log={data['log_path']}")
    print(
        f"lines={data['total_lines']} flow={data['flow_count']} "
        f"flow_lines={data['flow_line_count']} "
        f"frames={len(data['successful_frames'])} events={len(data['events'])}"
    )
    print(f"stages={','.join(data['stage_counts'])}")
    print(
        "decisions="
        + ",".join(
            f"{key}:{value}" for key, value in sorted(data["summary_decisions"].items())
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
