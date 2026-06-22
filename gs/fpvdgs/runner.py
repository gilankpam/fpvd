"""fpvd-runner: the wfb data plane, built on the wfb_ng library.

Spawned and supervised by the fpvd supervisor. WIFIBROADCAST_CFG must already
be in the environment (the supervisor sets it before spawn) so that wfb_ng.conf
parses our rendered cfg at import time.
"""

import os
import sys


def build_argv(profile: str, wlans: list[str]) -> list[str]:
    return ["--profiles", profile, "--wlans", *wlans]


def main(argv: list[str] | None = None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    if "WIFIBROADCAST_CFG" not in os.environ:
        sys.stderr.write("fpvd-runner: WIFIBROADCAST_CFG not set\n")
        raise SystemExit(2)
    from wfb_ng import server  # noqa: E402  (cfg parsed at import; env already set)

    sys.argv = ["fpvd-runner", *argv]
    server.main()


if __name__ == "__main__":
    main()
