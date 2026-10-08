"""Copy only this display's Xauthority cookie, without printing secrets or xhost."""

import os
import re
import shutil
import subprocess
from pathlib import Path


def main():
    display = os.environ.get("DISPLAY", "")
    match = re.fullmatch(r"(?:unix)?:([0-9]+)(?:\.[0-9]+)?", display)
    if not match:
        raise SystemExit(
            "Please run simctl gui from a local desktop terminal (DISPLAY=:0 etc.)."
        )
    if not Path("/tmp/.X11-unix", "X" + match[1]).is_socket():
        raise SystemExit("Local X11/Xwayland display socket is missing.")
    if not shutil.which("xauth"):
        raise SystemExit("xauth is required to copy the local display credential.")
    result = subprocess.run(
        ["xauth", "nlist", display], capture_output=True, text=True, check=True
    )
    records = result.stdout.splitlines()
    if not records:
        raise SystemExit(
            "No Xauthority cookie for DISPLAY; run from the logged-in desktop session."
        )
    directory = Path(__file__).resolve().parents[1] / ".runtime/gui"
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    directory.chmod(0o700)
    path = directory / "Xauthority"
    # FamilyWild lets the cookie work inside the container's private hostname.
    data = "\n".join("ffff" + record[4:] for record in records) + "\n"
    with path.open("w"):
        pass
    path.chmod(0o600)
    subprocess.run(
        ["xauth", "-f", str(path), "nmerge", "-"], input=data, text=True, check=True
    )
    print("Prepared private Xauthority file for display " + display)


if __name__ == "__main__":
    main()
