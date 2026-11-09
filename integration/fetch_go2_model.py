"""Download the official Unitree Go2 model at a fixed unitree_mujoco commit."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import urllib.request

REVISION = "4134cb5"
REPOSITORY = "unitreerobotics/unitree_mujoco"


def fetch(destination):
    destination = Path(destination)
    api = f"https://api.github.com/repos/{REPOSITORY}/git/trees/{REVISION}?recursive=1"
    request = urllib.request.Request(api, headers={"User-Agent": "CAPO-torque-sim"})
    with urllib.request.urlopen(request, timeout=60) as response:
        tree = json.load(response)
    prefix = "unitree_robots/go2/"
    entries = [entry for entry in tree["tree"] if entry["type"] == "blob"
               and entry["path"].startswith(prefix)
               and (entry["path"].endswith(".obj")
                    or entry["path"].removeprefix(prefix) in ("go2.xml", "scene.xml"))]

    def download(entry):
        relative = entry["path"].removeprefix(prefix)
        target = destination / relative
        if not target.exists() or target.stat().st_size != entry["size"]:
            url = f"https://raw.githubusercontent.com/{REPOSITORY}/{tree['sha']}/{entry['path']}"
            with urllib.request.urlopen(url, timeout=120) as response:
                payload = response.read()
            blob = hashlib.sha1(f"blob {len(payload)}\0".encode() + payload).hexdigest()
            if blob != entry["sha"]:
                raise RuntimeError(f"Git blob checksum mismatch: {relative}")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(payload)
        return relative

    with ThreadPoolExecutor(max_workers=4) as pool:
        for relative in pool.map(download, entries):
            print(relative, flush=True)
    (destination / "MODEL_SOURCE.json").write_text(
        json.dumps({"repository": REPOSITORY, "commit": tree["sha"]}, indent=2) + "\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("destination", type=Path)
    fetch(parser.parse_args().destination)
