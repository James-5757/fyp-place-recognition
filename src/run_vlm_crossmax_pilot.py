import argparse
import csv
import json
import os
import time
from pathlib import Path

from openai import OpenAI

from run_vlm_verification_pilot import (
    TASK_PROMPT,
    image_to_data_url,
    normalize_base_url,
    normalize_decision,
    parse_json_response,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Run Cross-Max VLM pilot")
    parser.add_argument(
        "--manifest", type=Path, default=Path("outputs/vlm_crossmax_pilot/manifest.csv")
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("outputs/vlm_crossmax_pilot")
    )
    parser.add_argument("--max-retries", type=int, default=3)
    return parser.parse_args()


def request_key(row):
    return (int(row["query_frame"]), int(row["challenger_frame"]), str(row["order"]))


def append_jsonl(path, record):
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(record, ensure_ascii=False) + "\n")
        file.flush()
        os.fsync(file.fileno())


def write_csv(path, records):
    fields = [
        "query_frame", "sc_rank1_frame", "challenger_frame", "order",
        "candidate_A_frame", "candidate_B_frame", "choice", "chosen_frame",
        "confidence", "viewpoint_difference", "matched_landmarks",
        "contradictions_A", "contradictions_B", "reason", "parse_success",
        "raw_response", "error", "request_success",
    ]
    temp = path.with_suffix(".tmp")
    with temp.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for record in records:
            row = {field: record.get(field, "") for field in fields}
            for field in ["matched_landmarks", "contradictions_A", "contradictions_B"]:
                if isinstance(row[field], list):
                    row[field] = json.dumps(row[field], ensure_ascii=False)
            writer.writerow(row)
    os.replace(temp, path)


def main():
    args = parse_args()
    if not args.manifest.is_file():
        raise FileNotFoundError(args.manifest)
    api_key = os.environ.get("TEST_API_KEY")
    api_base = os.environ.get("TEST_API_BASE")
    if not api_key or not api_base:
        raise RuntimeError("TEST_API_KEY and TEST_API_BASE must be nonempty")
    client = OpenAI(
        api_key=api_key,
        base_url=normalize_base_url(api_base),
        timeout=120.0,
    )
    model = os.environ.get("VLM_MODEL") or "gpt-5.6-sol"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.output_dir / "raw_responses.jsonl"
    decisions_path = args.output_dir / "vlm_decisions.csv"
    manifest = list(csv.DictReader(args.manifest.open(encoding="utf-8")))
    records_by_key = {}
    if raw_path.is_file():
        for line in raw_path.read_text(errors="ignore").splitlines():
            try:
                record = json.loads(line)
                key = tuple(record["request_key"])
                records_by_key[key] = record
            except (json.JSONDecodeError, KeyError, TypeError):
                pass
    completed = {
        key: record
        for key, record in records_by_key.items()
        if bool(record.get("request_success"))
    }
    records = list(records_by_key.values())
    print("manifest_requests", len(manifest), "existing_records", len(records), "successful_records", len(completed))
    for index, row in enumerate(manifest, start=1):
        key = request_key(row)
        if key in completed:
            continue
        collage = Path(row["collage_path"])
        content = [
            {"type": "text", "text": TASK_PROMPT},
            {"type": "image_url", "image_url": {"url": image_to_data_url(collage)}},
        ]
        record = {
            "request_key": list(key),
            "query_frame": int(row["query_frame"]),
            "sc_rank1_frame": int(row["sc_rank1_frame"]),
            "challenger_frame": int(row["challenger_frame"]),
            "order": row["order"],
            "candidate_A_frame": int(row["candidate_A_frame"]),
            "candidate_B_frame": int(row["candidate_B_frame"]),
        }
        for attempt in range(args.max_retries):
            try:
                response = client.chat.completions.create(
                    model=model,
                    messages=[{"role": "user", "content": content}],
                    temperature=0,
                )
                raw = response.choices[0].message.content or ""
                parsed, parse_success = parse_json_response(raw)
                decision = normalize_decision(parsed)
                chosen = ""
                if decision["choice"] == "A":
                    chosen = record["candidate_A_frame"]
                elif decision["choice"] == "B":
                    chosen = record["candidate_B_frame"]
                record.update(decision)
                record.update({
                    "chosen_frame": chosen,
                    "parse_success": parse_success,
                    "raw_response": raw,
                    "error": "",
                    "request_success": True,
                })
                break
            except Exception as error:
                if attempt + 1 < args.max_retries:
                    time.sleep(2.0 ** attempt)
                else:
                    record.update({
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
                        "error": f"{type(error).__name__}: {error}",
                        "request_success": False,
                    })
        append_jsonl(raw_path, record)
        completed[key] = record
        records_by_key[key] = record
        records = list(records_by_key.values())
        write_csv(decisions_path, records)
        print("processed", index, "/", len(manifest))
    write_csv(decisions_path, records)
    print("saved_records", len(records))


if __name__ == "__main__":
    main()
