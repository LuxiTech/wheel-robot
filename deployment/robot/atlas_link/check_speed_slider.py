#!/usr/bin/env python3
"""Exercise the live speed slider in a fresh browser without acquiring control."""

import argparse
import base64
import json
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

import websocket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="atlas-slider-browser-") as profile:
        with (args.output / "chrome.log").open("w") as log:
            browser = subprocess.Popen(
                [
                    "google-chrome",
                    "--headless=new",
                    "--no-sandbox",
                    "--disable-gpu",
                    "--no-proxy-server",
                    "--remote-debugging-port=0",
                    "--remote-allow-origins=*",
                    "--window-size=1440,1400",
                    "--user-data-dir=" + profile,
                    "about:blank",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            try:
                deadline = time.monotonic() + 20
                while not (Path(profile) / "DevToolsActivePort").exists():
                    if time.monotonic() > deadline:
                        raise RuntimeError("Chrome debugging endpoint unavailable")
                    time.sleep(0.1)
                port = (Path(profile) / "DevToolsActivePort").read_text().splitlines()[0]
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/json/list") as response:
                    pages = json.load(response)
                page = next(item for item in pages if item["type"] == "page")
                connection = websocket.create_connection(page["webSocketDebuggerUrl"], timeout=8)
                identifier = 0
                mutations = []

                def call(method, params=None):
                    nonlocal identifier
                    identifier += 1
                    connection.send(
                        json.dumps({"id": identifier, "method": method, "params": params or {}})
                    )
                    while True:
                        message = json.loads(connection.recv())
                        if message.get("method") == "Fetch.requestPaused":
                            connection.send(
                                json.dumps(
                                    {
                                        "method": "Fetch.failRequest",
                                        "params": {
                                            "requestId": message["params"]["requestId"],
                                            "errorReason": "BlockedByClient",
                                        },
                                        "id": 100000 + identifier,
                                    }
                                )
                            )
                        if message.get("method") == "Network.requestWillBeSent":
                            request = message["params"]["request"]
                            if request["method"] not in {"GET", "HEAD", "OPTIONS"}:
                                mutations.append(request)
                        if message.get("id") == identifier:
                            if "error" in message:
                                raise RuntimeError(message["error"])
                            return message.get("result", {})

                def evaluate(expression):
                    result = call(
                        "Runtime.evaluate", {"expression": expression, "returnByValue": True}
                    )
                    if "exceptionDetails" in result:
                        raise RuntimeError(result["exceptionDetails"])
                    return result.get("result", {}).get("value")

                call("Network.enable")
                call(
                    "Fetch.enable",
                    {"patterns": [{"urlPattern": "*/api/v1/control/*", "requestStage": "Request"}]},
                )
                call("Page.navigate", {"url": "http://192.168.4.118:8082/#/device"})
                while evaluate("document.querySelector('#manual-speed')?.max") != "0.5":
                    if time.monotonic() > deadline:
                        raise RuntimeError("Live slider did not receive SDK cap 0.50 m/s")
                    time.sleep(0.15)
                results = []
                for speed in [0.05, 0.1, 0.3, 0.5]:
                    evaluate(
                        f"document.querySelector('#manual-speed').value={speed}; document.querySelector('#manual-speed').dispatchEvent(new Event('input', {{bubbles:true}}))"
                    )
                    time.sleep(0.15)
                    display = evaluate(
                        "document.querySelector('label[for=manual-speed] strong').textContent.trim()"
                    )
                    assert display == f"{speed:.2f} m/s", display
                    results.append({"selected": speed, "display": display})
                evaluate(
                    "document.querySelector('#manual-speed').value=.3; document.querySelector('#manual-speed').dispatchEvent(new Event('input', {bubbles:true}))"
                )
                time.sleep(0.15)
                screenshot = call("Page.captureScreenshot", {"format": "png"})
                (args.output / "slider-after.png").write_bytes(base64.b64decode(screenshot["data"]))
                assert not mutations, mutations
                (args.output / "browser-check.json").write_text(
                    json.dumps(
                        {"range": [0.05, 0.5], "selections": results, "mutations": mutations},
                        indent=2,
                    )
                    + "\n"
                )
                connection.close()
                print("PASS: live slider selects 0.05/0.10/0.30/0.50 m/s; no control requests")
            finally:
                browser.terminate()
                browser.wait(timeout=10)


if __name__ == "__main__":
    main()
