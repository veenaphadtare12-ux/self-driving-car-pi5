"""
Non-blocking single-key reads from the terminal (works over SSH, no window needed).

    with KeyReader() as keys:
        k = keys.get()   # '' if no key pressed
"""
import sys


class KeyReader:
    def __init__(self):
        self._old = None
        self._win = sys.platform.startswith("win")
        self.enabled = sys.stdin is not None and sys.stdin.isatty()

    def __enter__(self):
        if self.enabled and not self._win:
            import termios
            import tty
            fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(fd)
            tty.setcbreak(fd)   # keeps Ctrl+C (ISIG) working
        return self

    def __exit__(self, *exc):
        if self._old is not None:
            import termios
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old)
        return False

    def get(self):
        if not self.enabled:
            return ""
        try:
            if self._win:
                import msvcrt
                return msvcrt.getwch() if msvcrt.kbhit() else ""
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        except Exception:
            pass
        return ""
