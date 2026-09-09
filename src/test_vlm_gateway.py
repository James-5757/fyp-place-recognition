import os
import base64
from pathlib import Path

from openai import OpenAI


API_KEY = os.environ["TEST_API_KEY"]
BASE_URL = os.environ["TEST_API_BASE"]

client = OpenAI(
    api_key=API_KEY,
    base_url=f"{BASE_URL}/v1",
)


def image_to_data_url(path):
    path = Path(path)

    with path.open("rb") as f:
        encoded = base64.b64encode(f.read()).decode("utf-8")

    return f"data:image/png;base64,{encoded}"


image_path = Path(
    "data/kitti/dataset/sequences/00/image_2/001550.png"
)

response = client.chat.completions.create(
    model="gpt-5.6-sol",
    messages=[
        {
            "role": "user",
            "content": [
                {
                    "type": "text",
                    "text": (
                        "Look at this road scene. "
                        "Describe three persistent physical landmarks "
                        "that could be useful for visual place recognition."
                    ),
                },
                {
                    "type": "image_url",
                    "image_url": {
                        "url": image_to_data_url(image_path)
                    },
                },
            ],
        }
    ],
    temperature=0,
)

print(response.choices[0].message.content)