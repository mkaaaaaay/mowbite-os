#!/usr/bin/python3
"""mowbite-web: serves the static export of the MowBite App, read only.

The root is looked up for every request: the own installation the host chose (/opt/mowbite-app/active/app), else the
one the OS brings along (/opt/mowbite/current/app). systemd hands in the listening socket
(mowbite-web.socket, or systemd-socket-activate on a PC); --prefix puts all paths under a directory.
  mowbite_web.py [--prefix DIR]
"""
import argparse
import itertools
import os
import shutil
import socket
import sys
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ACTIVE = "/opt/mowbite-app/active/app"
BUNDLED = "/opt/mowbite/current/app"
# Hashed parts an open page of an older installation may still load; their names never change their content.
FALLBACK = ("/opt/mowbite-app/current/app", "/opt/mowbite-app/previous/app", BUNDLED,
            "/opt/mowbite/previous/app")
STATIC = "_next/static/"
TIMEOUT = 30
# Below TasksMax of the unit; further connections wait in the backlog.
MAX_CONNECTIONS = 24
CHUNK = 64 * 1024
TYPES = {
    "html": "text/html; charset=utf-8", "js": "text/javascript; charset=utf-8",
    "mjs": "text/javascript; charset=utf-8", "css": "text/css; charset=utf-8",
    "json": "application/json", "map": "application/json", "webmanifest": "application/manifest+json",
    "txt": "text/plain; charset=utf-8", "xml": "application/xml", "svg": "image/svg+xml", "png": "image/png",
    "jpg": "image/jpeg", "jpeg": "image/jpeg", "gif": "image/gif", "webp": "image/webp", "ico": "image/x-icon",
    "woff": "font/woff", "woff2": "font/woff2", "ttf": "font/ttf", "wasm": "application/wasm",
}


def regular_file(root, rel):
    """The real path of root/rel when it is a regular file inside root, else None."""
    path = os.path.join(root, rel)
    if not os.path.isfile(path):
        return None
    path = os.path.realpath(path)
    return path if path.startswith(root + os.sep) else None


def accepts_gzip(header):
    for part in (header or "").split(","):
        token, *params = [p.strip() for p in part.split(";")]
        if token.lower() in ("gzip", "x-gzip"):
            q = [p[2:] for p in params if p.lower().startswith("q=")]
            try:
                return not q or float(q[0]) > 0
            except ValueError:
                return False
    return False


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "mowbite-web"
    sys_version = ""
    timeout = TIMEOUT
    # Head and a small body leave in one packet; handle_one_request flushes after each request.
    wbufsize = CHUNK
    prefix = ""

    def log_message(self, format, *args):
        pass

    # Every other method, also unknown ones.
    def __getattr__(self, name):
        if name.startswith("do_"):
            return self.not_allowed
        raise AttributeError(name)

    def do_GET(self):
        self.serve()

    do_HEAD = do_GET

    def not_allowed(self):
        self.reply(405, "Only GET and HEAD.\n", {"Allow": "GET, HEAD"})

    def root(self):
        for path in (ACTIVE, BUNDLED):
            real = os.path.realpath(self.prefix + path)
            if os.path.isdir(real):
                return real
        return None

    def fallbacks(self, root):
        for path in FALLBACK:
            real = os.path.realpath(self.prefix + path)
            if real != root:
                yield real

    def serve(self):
        raw = urllib.parse.urlsplit(self.path)
        try:
            path = urllib.parse.unquote(raw.path, errors="strict") if raw.path.isascii() else ""
        except UnicodeDecodeError:
            path = ""
        parts = path.split("/")[1:]
        root = self.root()
        if (not path.startswith("/") or "\0" in path or any(p in (".", "..") for p in parts) or "" in parts[:-1]
                or parts[0] == "cgi-bin"):
            return self.not_found(root)
        if root is None:
            return self.reply(503, "The MowBite App is not installed.\n")
        rel = path[1:] + ("index.html" if path.endswith("/") else "")
        for r in itertools.chain([root], self.fallbacks(root) if rel.startswith(STATIC) else ()):
            found = regular_file(r, rel)
            if found:
                return self.send(r, rel, found, 200)
        if not path.endswith("/") and regular_file(root, rel + "/index.html"):
            return self.reply(301, "", {"Location": raw.path + "/" + ("?" + raw.query if raw.query else "")})
        self.not_found(root)

    def not_found(self, root):
        found = root and regular_file(root, "404.html")
        if found:
            return self.send(root, "404.html", found, 404)
        self.reply(404, "Not found.\n")

    def send(self, root, rel, path, status):
        headers = {"Content-Type": TYPES.get(rel.rpartition(".")[2].lower(), "application/octet-stream"),
                   "Cache-Control": "public, max-age=31536000, immutable" if rel.startswith(STATIC) else "no-cache"}
        gz = regular_file(root, rel + ".gz")
        if gz:
            headers["Vary"] = "Accept-Encoding"
            if accepts_gzip(self.headers.get("Accept-Encoding")):
                path = gz
                headers["Content-Encoding"] = "gzip"
        # The host may have moved active since the lookup.
        try:
            f = open(path, "rb")
        except OSError:
            return self.reply(404, "Not found.\n")
        with f:
            st = os.fstat(f.fileno())
            headers["ETag"] = '"%x-%x%s"' % (st.st_mtime_ns, st.st_size, "-gz" if path == gz else "")
            tags = (self.headers.get("If-None-Match") or "").split(",")
            if status == 200 and headers["ETag"] in {tag.strip().removeprefix("W/") for tag in tags}:
                del headers["Content-Type"]
                return self.head(304, headers)
            headers["Content-Length"] = str(st.st_size)
            self.head(status, headers)
            if self.command != "HEAD":
                shutil.copyfileobj(f, self.wfile, CHUNK)

    def reply(self, status, text, headers=None):
        data = text.encode()
        headers = {"Content-Type": "text/plain; charset=utf-8", "Cache-Control": "no-cache",
                   "Content-Length": str(len(data)), **(headers or {})}
        self.head(status, headers)
        if self.command != "HEAD":
            self.wfile.write(data)

    def head(self, status, headers):
        self.send_response(status)
        self.send_header("X-Content-Type-Options", "nosniff")
        for key, value in headers.items():
            self.send_header(key, value)
        self.end_headers()


class Server(ThreadingHTTPServer):
    request_queue_size = 64

    def __init__(self, address, handler, sock=None):
        self.slots = threading.BoundedSemaphore(MAX_CONNECTIONS)
        super().__init__(address, handler, bind_and_activate=sock is None)
        if sock is not None:
            self.socket.close()
            self.socket = sock

    def process_request(self, request, client_address):
        self.slots.acquire()
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


def main():
    parser = argparse.ArgumentParser(description="Serves the MowBite App.")
    parser.add_argument("--prefix", default="")
    Handler.prefix = parser.parse_args().prefix
    if os.environ.get("LISTEN_PID") != str(os.getpid()) or os.environ.get("LISTEN_FDS") != "1":
        sys.exit("no socket from systemd (LISTEN_FDS)")
    Server(None, Handler, socket.socket(fileno=3)).serve_forever()


if __name__ == "__main__":
    main()
