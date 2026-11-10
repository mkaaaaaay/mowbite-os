"""mowbite-web against a small export in a temporary prefix.

  python3 -I web/test/test_web.py
"""
import gzip
import http.client
import os
import shutil
import socket
import sys
import tempfile
import threading
import time
import unittest

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "host"))
import mowbite_web  # noqa: E402
from mowbite_host import paths  # noqa: E402

CHUNK = "_next/static/chunks/main-abc123.js"


class WebTest(unittest.TestCase):
    def setUp(self):
        self.prefix = tempfile.mkdtemp(prefix="mowbite-web-")
        self.addCleanup(shutil.rmtree, self.prefix)
        handler = type("H", (mowbite_web.Handler,), {"prefix": self.prefix})
        self.server = mowbite_web.Server(("127.0.0.1", 0), handler)
        self.handler = handler
        self.port = self.server.server_address[1]
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def write(self, path, data="x"):
        path = os.path.join(self.prefix, path.lstrip("/"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as f:
            f.write(data.encode() if isinstance(data, str) else data)
        return path

    # An export at <opt>/<revision>/app with current (and active for the app) on it.
    def export(self, opt, revision, label, link="current"):
        app = f"{opt}/{revision}/app"
        self.write(f"{app}/index.html", f"<p>{label} home</p>")
        self.write(f"{app}/map/index.html", f"<p>{label} map</p>")
        self.write(f"{app}/map/index.txt", f"{label} rsc")
        self.write(f"{app}/404.html", f"<p>{label} not found</p>")
        self.write(f"{app}/version.json", '{"version": "%s"}' % label)
        self.write(f"{app}/{CHUNK}", f"// {label}")
        self.write(f"{app}/{CHUNK}.gz", gzip.compress(f"// {label}".encode()))
        self.write(f"{app}/font.woff2", b"\0woff")
        self.link(opt, link, revision)
        return os.path.join(self.prefix, app.lstrip("/"))

    def link(self, opt, name, revision):
        path = os.path.join(self.prefix, opt.lstrip("/"), name)
        if os.path.lexists(path):
            os.unlink(path)
        os.symlink(revision, path)

    def request(self, method, path, headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
        self.addCleanup(conn.close)
        conn.request(method, path, headers=headers or {})
        r = conn.getresponse()
        return r.status, dict(r.getheaders()), r.read()

    def get(self, path, headers=None):
        return self.request("GET", path, headers)

    # mowbite-web keeps its own copy of the folders of the host; they have to stay the same.
    def test_folders_are_those_of_the_host(self):
        p = paths.Paths()
        app, os_ = p.places["app"], p.os
        self.assertEqual((mowbite_web.ACTIVE, mowbite_web.BUNDLED), (app.active + "/app", os_.current + "/app"))
        self.assertEqual(mowbite_web.FALLBACK, tuple(f + "/app" for f in (app.current, app.previous, os_.current,
                                                                          os_.previous)))

    def test_no_installation_is_503(self):
        status, headers, body = self.get("/")
        self.assertEqual(status, 503)
        self.assertIn(b"not installed", body)
        self.assertEqual(headers["X-Content-Type-Options"], "nosniff")

    def test_root_is_bundled_without_active(self):
        self.export("/opt/mowbite", "os1", "bundled")
        self.assertEqual(self.get("/")[2], b"<p>bundled home</p>")

    def test_active_wins_and_is_read_per_request(self):
        self.export("/opt/mowbite", "os1", "bundled")
        self.export("/opt/mowbite-app", "app1", "own", link="active")
        self.assertEqual(self.get("/")[2], b"<p>own home</p>")
        os.unlink(os.path.join(self.prefix, "opt/mowbite-app/active"))
        self.assertEqual(self.get("/")[2], b"<p>bundled home</p>")

    def test_routes(self):
        self.export("/opt/mowbite", "os1", "bundled")
        status, headers, body = self.get("/map/")
        self.assertEqual((status, body), (200, b"<p>bundled map</p>"))
        self.assertEqual(headers["Content-Type"], "text/html; charset=utf-8")
        status, headers, _ = self.get("/map?x=1")
        self.assertEqual((status, headers["Location"]), (301, "/map/?x=1"))
        status, _, body = self.get("/map/index.txt?_rsc=1")
        self.assertEqual((status, body), (200, b"bundled rsc"))
        self.assertEqual(self.get("/font.woff2")[1]["Content-Type"], "font/woff2")

    def test_unknown_is_404_with_the_page_of_the_export(self):
        self.export("/opt/mowbite", "os1", "bundled")
        for path in ("/nope", "/nope/", "/map/nope.js"):
            status, headers, body = self.get(path)
            self.assertEqual((status, body), (404, b"<p>bundled not found</p>"), path)
            self.assertEqual(headers["Cache-Control"], "no-cache")

    def test_cgi_bin_is_always_404(self):
        self.export("/opt/mowbite", "os1", "bundled")
        self.write("/opt/mowbite/os1/app/cgi-bin/version", "secret")
        for path in ("/cgi-bin/version", "/cgi-bin/", "/cgi-bin", "/cgi-bin/update?x"):
            self.assertEqual(self.get(path)[0], 404, path)
        self.assertEqual(self.request("POST", "/cgi-bin/update")[0], 405)

    def test_only_get_and_head(self):
        self.export("/opt/mowbite", "os1", "bundled")
        for method in ("POST", "PUT", "DELETE", "OPTIONS", "PATCH", "FOO"):
            status, headers, _ = self.request(method, "/")
            self.assertEqual((status, headers["Allow"]), (405, "GET, HEAD"), method)
        status, headers, body = self.request("HEAD", "/")
        self.assertEqual((status, body), (200, b""))
        self.assertEqual(headers["Content-Length"], str(len(b"<p>bundled home</p>")))

    def test_no_way_out_of_the_root(self):
        self.export("/opt/mowbite", "os1", "bundled")
        self.write("/opt/secret.txt", "secret")
        self.write("/opt/mowbite/os1/secret.txt", "secret")
        os.symlink(os.path.join(self.prefix, "opt/secret.txt"), os.path.join(self.prefix, "opt/mowbite/os1/app/out.txt"))
        os.symlink(os.path.join(self.prefix, "opt"), os.path.join(self.prefix, "opt/mowbite/os1/app/up"))
        for path in ("/../secret.txt", "/..%2fsecret.txt", "/%2e%2e/secret.txt", "/map/%2E%2E/%2e%2e/secret.txt",
                     "/out.txt", "/up/secret.txt", "/up/", "/up", "//opt/secret.txt", "/map/./index.html",
                     "/%00", "/%ff", "/map/../../secret.txt"):
            status, _, body = self.get(path)
            self.assertNotIn(b"secret", body, path)
            self.assertEqual(status, 404, path)
        # Absolute-form request line.
        self.assertNotIn(b"secret", self.get("http://x/../secret.txt")[2])

    def test_no_directory_listing(self):
        self.export("/opt/mowbite", "os1", "bundled")
        os.makedirs(os.path.join(self.prefix, "opt/mowbite/os1/app/empty"))
        self.assertEqual(self.get("/empty/")[0], 404)
        self.assertEqual(self.get("/empty")[0], 404)
        self.assertEqual(self.get("/_next/")[0], 404)

    def test_cache_headers_and_304(self):
        self.export("/opt/mowbite", "os1", "bundled")
        _, headers, _ = self.get("/" + CHUNK)
        self.assertEqual(headers["Cache-Control"], "public, max-age=31536000, immutable")
        status, headers, _ = self.get("/")
        self.assertEqual((status, headers["Cache-Control"]), (200, "no-cache"))
        tag = headers["ETag"]
        status, headers, body = self.get("/", {"If-None-Match": f'"other", {tag}'})
        self.assertEqual((status, body, headers["ETag"]), (304, b"", tag))
        self.assertNotIn("Content-Length", headers)
        self.assertEqual(self.get("/", {"If-None-Match": '"other"'})[0], 200)
        # A new installation gives a new tag.
        self.export("/opt/mowbite-app", "app1", "own", link="active")
        self.assertEqual(self.get("/", {"If-None-Match": tag})[0], 200)

    def test_gzip_copy(self):
        self.export("/opt/mowbite", "os1", "bundled")
        status, headers, body = self.get("/" + CHUNK, {"Accept-Encoding": "br, gzip;q=0.8"})
        self.assertEqual((status, headers["Content-Encoding"], headers["Vary"]), (200, "gzip", "Accept-Encoding"))
        self.assertEqual(gzip.decompress(body), b"// bundled")
        self.assertEqual(headers["Content-Type"], "text/javascript; charset=utf-8")
        plain_tag = self.get("/" + CHUNK)[1]["ETag"]
        self.assertNotEqual(headers["ETag"], plain_tag)
        for accept in (None, "identity", "gzip;q=0", "br"):
            status, headers, body = self.get("/" + CHUNK, {"Accept-Encoding": accept} if accept else {})
            self.assertEqual(body, b"// bundled", accept)
            self.assertNotIn("Content-Encoding", headers)

    def test_static_falls_back_to_older_installations(self):
        bundled = self.export("/opt/mowbite", "os1", "bundled")
        self.export("/opt/mowbite-app", "app1", "old")
        self.link("/opt/mowbite-app", "previous", "app1")
        self.export("/opt/mowbite-app", "app2", "new", link="active")
        self.write("/opt/mowbite/os1/app/_next/static/chunks/bundled-only.js", "// b")
        self.write("/opt/mowbite-app/app1/app/_next/static/chunks/old-only.js", "// o")
        self.write("/opt/mowbite-app/app1/app/old-page.html", "o")
        self.assertEqual(self.get("/_next/static/chunks/old-only.js")[2], b"// o")
        self.assertEqual(self.get("/_next/static/chunks/bundled-only.js")[2], b"// b")
        self.assertEqual(self.get("/" + CHUNK)[2], b"// new")
        # Only hashed parts: pages and other files come from the root alone.
        self.assertEqual(self.get("/old-page.html")[0], 404)
        self.assertTrue(os.path.isdir(bundled))

    def test_hanging_client_is_cut_and_others_go_on(self):
        self.handler.timeout = 0.5
        self.export("/opt/mowbite", "os1", "bundled")
        quiet = socket.create_connection(("127.0.0.1", self.port))
        self.addCleanup(quiet.close)
        quiet.sendall(b"GET / HTTP/1.1\r\n")
        self.assertEqual(self.get("/")[0], 200)
        quiet.settimeout(5)
        start = time.monotonic()
        self.assertEqual(quiet.recv(100), b"")
        self.assertLess(time.monotonic() - start, 3)

    def test_connections_beyond_the_limit_wait(self):
        self.handler.timeout = 1
        self.export("/opt/mowbite", "os1", "bundled")
        idle = [socket.create_connection(("127.0.0.1", self.port)) for _ in range(mowbite_web.MAX_CONNECTIONS)]
        for s in idle:
            self.addCleanup(s.close)
        deadline = time.monotonic() + 5
        while self.server.slots._value and time.monotonic() < deadline:
            time.sleep(0.01)
        start = time.monotonic()
        self.assertEqual(self.get("/")[0], 200)
        self.assertGreater(time.monotonic() - start, 0.5)


if __name__ == "__main__":
    unittest.main()
