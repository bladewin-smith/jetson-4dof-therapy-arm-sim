#!/usr/bin/env python3
"""Tiny dependency-free development server for the simulator."""

from __future__ import annotations

import argparse
import http.server
import socketserver
from pathlib import Path


ROOT = Path(__file__).resolve().parent / "src"


class SimulatorHandler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8",
        ".json": "application/json; charset=utf-8",
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        super().end_headers()


class ThreadingServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the 4-DOF arm simulator")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    args = parser.parse_args()

    with ThreadingServer((args.host, args.port), SimulatorHandler) as server:
        print(f"Simulator: http://127.0.0.1:{args.port}")
        print("Press Ctrl+C to stop")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped")


if __name__ == "__main__":
    main()

