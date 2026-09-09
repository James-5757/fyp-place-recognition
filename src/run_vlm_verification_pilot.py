import argparse
import base64
import csv
import json
import os
import re
import time
from pathlib import Path

from openai import OpenAI


TASK_PROMPT = """You are a visual place-recognition verifier for a mobile robot.
The image contains three rows:
QUERY
CANDIDATE A
CANDIDATE B
Each row contains a short temporal sequence from the same camera.
Determine whether Candidate A or Candidate B is more likely to show the
SAME PHYSICAL PLACE as the Query.
The camera may observe the location from substantially different viewpoints.
Focus on persistent physical evidence such as:
- building facade geometry
- window and doorway arrangement
- relative positions of buildings
- road curvature
- intersections and junction geometry
- sidewalks and curbs
- walls and fences
- traffic signs
- poles and fixed infrastructure
- distinctive vegetation layout
Do NOT rely strongly on:
- parked or moving cars
- pedestrians
- shadows
- lighting
- temporary objects
Do not choose a candidate simply because both scenes are generic residential
streets.
Look for MULTIPLE spatially consistent persistent landmarks.
If there is not enough evidence, return UNCERTAIN.
If neither candidate appears to depict the same physical place, return
NEITHER.
Return JSON only:
{
  "choice": "A" | "B" | "UNCERTAIN" | "NEITHER",
  "confidence": integer from 0 to 100,
  "matched_landmarks": [
    "short description"
  ],
  "contradictions_A": [
    "short description"
  ],
  "contradictions_B": [
    "short description"
  ],
  "viewpoint_difference": "small" | "moderate" | "large" | "unknown",
  "reason": "brief explanation"
}"""


def parse_args():
    parser = argparse.ArgumentParser(description="Run VLM verification pilot")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("outputs/vlm_verification_pilot/manifest.csv"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/vlm_verification_pilot"),
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--max-retries", type=int, default=3)
    parser.add_argument("--retry-base-seconds", type=float, default=2.0)
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    return parser.parse_args()


def normalize_base_url(value):
    base = value.strip().rstrip("/")
    if not re.search(r"/v1$", base):
        base += "/v1"
    return base


def image_to_data_url(path):
    with path.open("rb") as file:
        encoded = base64.b64encode(file.read()).decode("ascii")
    return f"data:image/png;base64,{encoded}"


def request_key(row):
    return (
        int(row["query_frame"]),
        int(row["challenger_frame"]),
        str(row["order"]),
    )


def parse_json_response(raw):
    text = raw.strip()
    try:
        return json.loads(text), True
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", text, flags=re.DOTALL)
        if match:
            try:
                return json.loads(match.group(0)), True
            except json.JSONDecodeError:
                pass
    return {}, False


def normalize_decision(parsed):
    choice = str(parsed.get("choice", "")).strip().upper()
    if choice not in {"A", "B", "UNCERTAIN", "NEITHER"}:
        choice = "UNCERTAIN"
    try:
        confidence = int(parsed.get("confidence", 0))
    except (TypeError, ValueError):
        confidence = 0
    confidence = max(0, min(100, confidence))
    viewpoint = str(parsed.get("viewpoint_difference", "unknown")).strip().lower()
    if viewpoint not in {"small", "moderate", "large", "unknown"}:
        viewpoint = "unknown"
    def list_value(key):
        value = parsed.get(key, [])
        if isinstance(value, list):
            return [str(item) for item in value]
        return [str(value)] if value else []
    return {
        "choice": choice,
        "confidence": confidence,
        "matched_landmarks": list_value("matched_landmarks"),
        "contradictions_A": list_value("contradictions_A"),
        "contradictions_B": list_value("contradictions_B"),
        "viewpoint_difference": viewpoint,
        "reason": str(parsed.get("reason", "")),
    }


def append_jsonl(path, record):
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(record, ensure_ascii=False) + "\n")
        file.flush()
        os.fsync(file.fileno())


def write_decisions_csv(path, records):
    fieldnames = [
        "query_frame", "sc_rank1_frame", "challenger_frame", "order",
        "candidate_A_frame", "candidate_B_frame", "choice", "chosen_frame",
        "confidence", "viewpoint_difference", "matched_landmarks",
        "contradictions_A", "contradictions_B", "reason", "parse_success",
        "raw_response", "error", "request_success",
    ]
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            row = {key: record.get(key, "") for key in fieldnames}
            for key in [
                "matched_landmarks", "contradictions_A", "contradictions_B"
            ]:
                if isinstance(row[key], list):
                    row[key] = json.dumps(row[key], ensure_ascii=False)
            writer.writerow(row)
    os.replace(temporary, path)


def main():
    args = parse_args()
    if args.max_retries < 1:
        raise ValueError("--max-retries must be positive")
    if not args.manifest.is_file():
        raise FileNotFoundError(args.manifest)
    key = os.environ.get("TEST_API_KEY")
    base = os.environ.get("TEST_API_BASE")
    if not key or not base:
        raise RuntimeError(
            "TEST_API_KEY and TEST_API_BASE must be set in the runner environment; "
            "no API request was made"
        )
    model = os.environ.get("VLM_MODEL") or "gpt-5.6-sol"
    client = OpenAI(
        api_key=key,
        base_url=normalize_base_url(base),
        timeout=args.timeout_seconds,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.output_dir / "raw_responses.jsonl"
    decisions_path = args.output_dir / "vlm_decisions.csv"
    manifest = list(csv.DictReader(args.manifest.open(encoding="utf-8")))

    completed = {}
    if raw_path.is_file() and not args.force:
        with raw_path.open(encoding="utf-8") as file:
            for line in file:
                try:
                    record = json.loads(line)
                    completed[tuple(record["request_key"])] = record
                except (json.JSONDecodeError, KeyError, TypeError):
                    continue
    records = list(completed.values())
    print(f"Manifest requests: {len(manifest)}")
    print(f"Existing completed records: {len(records)}")

    for index, row in enumerate(manifest, start=1):
        key_tuple = request_key(row)
        if key_tuple in completed and not args.force:
            continue
        collage_path = Path(row["collage_path"])
        if not collage_path.is_file():
            raise FileNotFoundError(collage_path)
        content = [
            {"type": "text", "text": TASK_PROMPT},
            {
                "type": "image_url",
                "image_url": {"url": image_to_data_url(collage_path)},
            },
        ]
        record = {
            "request_key": list(key_tuple),
            "query_frame": int(row["query_frame"]),
            "sc_rank1_frame": int(row["sc_rank1_frame"]),
            "challenger_frame": int(row["challenger_frame"]),
            "order": row["order"],
            "candidate_A_frame": int(row["candidate_A_frame"]),
            "candidate_B_frame": int(row["candidate_B_frame"]),
        }
        last_error = ""
        for attempt in range(args.max_retries):
            try:
                response = client.chat.completions.create(
                    model=model,
                    messages=[{"role": "user", "content": content}],
                    temperature=0,
                )
                raw = response.choices[0].message.content or ""
                parsed, success = parse_json_response(raw)
                decision = normalize_decision(parsed)
                chosen_frame = ""
                if decision["choice"] == "A":
                    chosen_frame = record["candidate_A_frame"]
                elif decision["choice"] == "B":
                    chosen_frame = record["candidate_B_frame"]
                record.update(decision)
                record.update(
                    {
                        "chosen_frame": chosen_frame,
                        "parse_success": success,
                        "raw_response": raw,
                        "error": "",
                        "request_success": True,
                    }
                )
                break
            except Exception as error:
                last_error = f"{type(error).__name__}: {error}"
                if attempt + 1 < args.max_retries:
                    time.sleep(args.retry_base_seconds * (2 ** attempt))
                else:
                    record.update(
                        {
                            "choice": "UNCERTAIN",
                            "chosen_frame": "",
                            "confidence": 0,
                            "viewpoint_difference": "unknown",
                            "matched_landmarks": [],
                            "contradictions_A": [],
                            "contradictions_B": [],
                            "reason": "API request failed",
                            "parse_success": False,
                            "raw_response": "",
                            "error": last_error,
                            "request_success": False,
                        }
                    )
        append_jsonl(raw_path, record)
        completed[key_tuple] = record
        records = list(completed.values())
        write_decisions_csv(decisions_path, records)
        print(f"Processed {index}/{len(manifest)}")

    write_decisions_csv(decisions_path, records)
    print(f"Saved raw responses: {raw_path}")
    print(f"Saved decisions: {decisions_path}")


if __name__ == "__main__":
    main()
