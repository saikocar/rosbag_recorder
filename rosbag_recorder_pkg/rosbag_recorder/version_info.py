"""èªåãã©ã®çã§åãã¦ããããåä¹ã(dw-notesãçãåä¹ããæ¹é 2026-08-22)ã
å½¢å¼: <branch>@<short hash>[+dirty]ãdetached ãªã HEAD@<hash>ãgit ãç¡ããã° unknownã
"""
import os
import subprocess
from functools import lru_cache


def _git(d, *args):
    out = subprocess.run(['git', '-C', d, *args], capture_output=True, text=True, timeout=2)
    return out.stdout.strip() if out.returncode == 0 else None


@lru_cache(maxsize=1)
def get_version() -> str:
    here = os.path.dirname(os.path.realpath(__file__))
    d = here
    for _ in range(7):  # ããã±ã¼ã¸ç´ä¸ã«ç©ºã® .git ãå±ããã¨ãããã®ã§è¦ªã¸ããã®ã¼ã
        try:
            h = _git(d, 'rev-parse', '--short', 'HEAD')
            if h:
                b = _git(d, 'rev-parse', '--abbrev-ref', 'HEAD') or 'HEAD'
                dirty = _git(d, 'status', '--porcelain', '--', here)
                return f'{b}@{h}' + ('+dirty' if dirty else '')
        except Exception:  # noqa: BLE001
            break
        d = os.path.dirname(d)
    vf = os.path.join(here, 'VERSION')
    if os.path.isfile(vf):
        return open(vf).read().strip() or 'unknown'
    return 'unknown (copy-install, no git)'
