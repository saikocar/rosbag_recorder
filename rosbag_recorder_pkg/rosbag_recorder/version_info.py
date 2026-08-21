"""自分がどの版で動いているかを名乗る(dw-notes「版を名乗る」方針 2026-08-22)。ptp_sync の version_info.py と同じ。"""
import os
import subprocess
from functools import lru_cache


@lru_cache(maxsize=1)
def get_version() -> str:
    here = os.path.dirname(os.path.realpath(__file__))
    d = here
    for _ in range(7):  # ããã±ã¼ã¸ç´ä¸ã«ç©ºã® .git ãå±ããã¨ãããã®ã§è¦ªã¸ããã®ã¼ã
        try:
            out = subprocess.run(['git', '-C', d, 'rev-parse', '--short', 'HEAD'],
                                 capture_output=True, text=True, timeout=2)
            if out.returncode == 0 and out.stdout.strip():
                dirty = subprocess.run(['git', '-C', d, 'status', '--porcelain', '--', here],
                                       capture_output=True, text=True, timeout=2).stdout.strip()
                return out.stdout.strip() + ('+dirty' if dirty else '')
        except Exception:  # noqa: BLE001
            break
        d = os.path.dirname(d)
    vf = os.path.join(here, 'VERSION')
    if os.path.isfile(vf):
        return open(vf).read().strip() or 'unknown'
    return 'unknown (copy-install, no git)'
