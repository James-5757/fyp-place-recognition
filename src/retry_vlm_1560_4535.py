import base64
import csv
import json
import os
import re
import tempfile
from pathlib import Path

from openai import OpenAI

from run_vlm_verification_pilot import (
    TASK_PROMPT,
    image_to_data_url,
    normalize_base_url,
    normalize_decision,
    parse_json_response,
)


def main():
    query_frame = 1560
    challenger_frame = 4535
    order = "AB"
    output_dir = Path("outputs/vlm_verification_pilot")
    manifest = list(csv.DictReader((output_dir / "manifest.csv").open()))
    row = next(
        item
        for item in manifest
        if int(item["query_frame"]) == query_frame
        and int(item["challenger_frame"]) == challenger_frame
        and item["order"] == order
    )
    key = os.environ["TEST_API_KEY"]
    base = os.environ["TEST_API_BASE"]
    client = OpenAI(
        api_key=key,
        base_url=normalize_base_url(base),
        timeout=120.0,
    )
    content = [
        {"type": "text", "text": TASK_PROMPT},
        {
            "type": "image_url",
            "image_url": {"url": image_to_data_url(Path(row["collage_path"]))},
        },
    ]
    response = client.chat.completions.create(
        model=os.environ.get("VLM_MODEL") or "gpt-5.6-sol",
        messages=[{"role": "user", "content": content}],
        temperature=0,
    )
    raw = response.choices[0].message.content or ""
    parsed, parse_success = parse_json_response(raw)
    decision = normalize_decision(parsed)
    chosen = ""
    if decision["choice"] == "A":
        chosen = int(row["candidate_A_frame"])
    elif decision["choice"] == "B":
        chosen = int(row["candidate_B_frame"])
    replacement = {
        "request_key": [query_frame, challenger_frame, order],
        "query_frame": query_frame,
        "sc_rank1_frame": int(row["sc_rank1_frame"]),
        "challenger_frame": challenger_frame,
        "order": order,
        "candidate_A_frame": int(row["candidate_A_frame"]),
        "candidate_B_frame": int(row["candidate_B_frame"]),
        **decision,
        "chosen_frame": chosen,
        "parse_success": parse_success,
        "raw_response": raw,
        "error": "",
        "request_success": True,
    }
    raw_path = output_dir / "raw_responses.jsonl"
    records = []
    for line in raw_path.read_text().splitlines():
        item = json.loads(line)
        if tuple(item.get("request_key", [])) == (query_frame, challenger_frame, order):
            item = replacement
        records.append(item)
    temporary = raw_path.with_suffix(".retry.tmp")
    with temporary.open("w", encoding="utf-8") as file:
        for item in records:
            file.write(json.dumps(item, ensure_ascii=False) + "\n")
    os.replace(temporary, raw_path)
    print("retry_updated", query_frame, challenger_frame, order)
    print("parse_success", parse_success)
    print("choice", decision["choice"])
    print("confidence", decision["confidence"])


if __name__ == "__main__":
    main()
