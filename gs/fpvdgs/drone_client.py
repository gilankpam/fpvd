"""HTTP client to the drone fpvd. Used by the unified /config + /apply lanes
(the facade read-merge and the LinkCoordinator's drone push)."""

import json
import urllib.error
import urllib.request


class DroneUnreachable(Exception):
    pass


class DroneRejected(Exception):
    """The drone returned a 4xx — a validation/permission rejection, NOT a
    connectivity failure. Carries the status code and parsed error body."""
    def __init__(self, code: int, body):
        self.code = code
        self.body = body
        self.message = (body.get("message") if isinstance(body, dict) else None) \
            or f"drone rejected ({code})"
        super().__init__(f"{code}: {self.message}")


class DroneClient:
    def __init__(self, endpoint: str, timeout: float = 10.0):
        self.endpoint = endpoint.rstrip("/")
        self.timeout = timeout

    def _request(self, method: str, path: str, body: dict | None = None) -> tuple[int, bytes]:
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.endpoint + path, data=data, method=method)
        if data is not None:
            req.add_header("content-type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return resp.status, resp.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()
        except (urllib.error.URLError, OSError) as e:
            raise DroneUnreachable(str(e))

    def _ok_json(self, method: str, path: str, body: dict | None = None) -> dict:
        code, raw = self._request(method, path, body)
        if 400 <= code < 500:
            try:
                parsed = json.loads(raw or b"{}")
            except ValueError:
                parsed = {"raw": raw.decode("utf-8", "replace")}
            raise DroneRejected(code, parsed)
        if code >= 500:
            raise DroneUnreachable(f"drone {method} {path} -> {code}")
        return json.loads(raw or b"{}")

    def healthz(self) -> bool:
        try:
            code, _ = self._request("GET", "/healthz")
            return code == 200
        except DroneUnreachable:
            return False

    def get_config(self) -> dict:
        return self._ok_json("GET", "/config")

    def get_status(self) -> dict:
        return self._ok_json("GET", "/status")

    def patch_config(self, sparse: dict) -> dict:
        return self._ok_json("PATCH", "/config", sparse)

    def apply(self) -> dict:
        return self._ok_json("POST", "/apply")
