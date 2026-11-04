#!/usr/bin/env python3
"""Command-line client for dLidar's phone protocol v1 over the local socket
(docs/phone-protocol.md). Standard library only.

  tools/phone_client.py [--socket PATH] [--wait S] CMD [JSON_ARGS]
  tools/phone_client.py [--socket PATH] [--wait S] --follow [CMD [JSON_ARGS]]

Prints the response to CMD (hello, status.get, info.get, recording.start, recording.stop, wifi.status, wifi.connect, log.get) as one JSON line.
--follow then keeps printing every event until Ctrl-C or the scanner closes
the connection. The socket is /run/dlidar/dlidar.sock (the service), else
$XDG_RUNTIME_DIR/dlidar.sock (dLidar run by hand). --wait retries the connect
for S seconds, for a dLidar that is just starting.
"""
import argparse
import json
import os
import socket
import sys
import time


def default_socket():
    candidates = ["/run/dlidar/dlidar.sock"]
    xdg = os.environ.get("XDG_RUNTIME_DIR")
    if xdg:
        candidates.append(os.path.join(xdg, "dlidar.sock"))
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[-1]


def connect(path, wait_s):
    deadline = time.monotonic() + wait_s
    while True:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(path)
            return s
        except OSError as e:
            s.close()
            if time.monotonic() >= deadline:
                sys.exit(f"phone_client: cannot connect to {path}: {e.strerror}")
            time.sleep(0.2)


def lines(sock):
    buf = b""
    while True:
        chunk = sock.recv(16384)
        if not chunk:
            return
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            yield line.decode("utf-8", "replace")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--socket", default=None, help="the socket path")
    p.add_argument("--wait", type=float, default=0, metavar="S", help="retry the connect for S seconds")
    p.add_argument("--follow", action="store_true", help="print events until Ctrl-C")
    p.add_argument("cmd", nargs="?", help="command, e.g. status.get")
    p.add_argument("args", nargs="?", help="arguments as a JSON object")
    a = p.parse_args()
    if not a.cmd and not a.follow:
        p.error("give a command, --follow, or both")

    sock = connect(a.socket or default_socket(), a.wait)
    request_id = None
    if a.cmd:
        request = {"v": 1, "id": 1, "cmd": a.cmd}
        if a.args:
            try:
                request["args"] = json.loads(a.args)
            except json.JSONDecodeError as e:
                sys.exit(f"phone_client: JSON_ARGS is not JSON: {e}")
        request_id = 1
        sock.sendall((json.dumps(request, separators=(",", ":")) + "\n").encode())

    status = 0
    try:
        for line in lines(sock):
            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                print(line)
                continue
            is_response = "event" not in message
            if is_response and message.get("id") == request_id:
                print(line, flush=True)
                if not message.get("ok"):
                    status = 1
                if not a.follow:
                    break
            elif "event" in message and a.follow:
                print(line, flush=True)
            elif is_response:
                print(line, flush=True)  # too_many_clients or another unsolicited error
                if message.get("error", {}).get("code") == "too_many_clients":
                    status = 1
                    break
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
