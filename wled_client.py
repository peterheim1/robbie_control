"""WLED LED segment bar controller via HTTP JSON API.

Robbie's 37-LED strip is split into 3 WLED segments (confirmed live via
/json/state — do not renumber without re-checking, WLED segment IDs are a
device-side config, not something this file controls):
  Segment 0 (2 LEDs)  — eyes:   idle / listening indicator
  Segment 1 (8 LEDs)  — mouth:  outside-in chase while speaking
  Segment 2 (27 LEDs) — status: battery voltage / fault indicator

Colors:
  eyes idle        → green   [0, 255, 0]
  eyes listening    → magenta [255, 0, 255]
  status normal     → blue    [0, 80, 255]   (charging, or voltage >= 12.8V)
  status low        → aqua    [0, 255, 255]  (12.0V <= voltage < 12.8V)
  status critical   → red     [255, 20, 0]   (voltage < 12.0V, solid)
  status problem     → red, blinking          (an active /diagnostics WARN/ERROR,
                                                takes priority over the voltage color)
"""

import asyncio
import contextlib
import json
import logging
import urllib.request

logger = logging.getLogger(__name__)

SEG_EYES = 0
SEG_TALK = 1
SEG_STATUS = 2
TALK_LEN = 8  # LEDs in the mouth segment

COLOR_GREEN   = [0, 255, 0]
COLOR_MAGENTA = [255, 0, 255]
COLOR_BLUE    = [0, 80, 255]
COLOR_AQUA    = [0, 255, 255]
COLOR_RED     = [255, 20, 0]
COLOR_TALK    = [255, 182, 193]
COLOR_OFF     = [0, 0, 0]

FX_SOLID = 0
FX_BLINK = 1

# Outside-in chase: each entry is the pair of lit LED indices (relative to
# the 8-LED mouth segment) for that animation frame — 0&7 first, closing in
# to 3&4, then repeating for as long as talking() is active.
_TALK_STEPS = [(0, 7), (1, 6), (2, 5), (3, 4)]
_TALK_FRAME_SECS = 0.13


class WLEDClient:
    """Controls Robbie's 3-segment WLED strip via its HTTP JSON API."""

    def __init__(self, host: str, timeout: float = 2.0):
        self._url = f"http://{host}/json/state"
        self._timeout = timeout

    # ------------------------------------------------------------------
    # Segment 0 — eyes
    # ------------------------------------------------------------------

    async def set_eyes_idle(self):
        """Green at 50% brightness — system ready, wake word listening."""
        await self._set_segment(SEG_EYES, COLOR_GREEN, brightness=128)

    async def set_eyes_listening(self):
        """Magenta — actively capturing a voice response."""
        await self._set_segment(SEG_EYES, COLOR_MAGENTA)

    # ------------------------------------------------------------------
    # Segment 1 — mouth / talking
    # ------------------------------------------------------------------

    async def set_mouth_off(self):
        """Blank the mouth bar — call once at startup so it doesn't inherit
        whatever effect/colour WLED last had on this segment before Robbie
        ever spoke."""
        await self._blank_talk_leds()

    @contextlib.asynccontextmanager
    async def talking(self):
        """Run the outside-in chase on the mouth bar for the duration of the block."""
        task = asyncio.create_task(self._talk_loop())
        try:
            yield
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
            await self._blank_talk_leds()

    async def _talk_loop(self):
        step = 0
        while True:
            lit = _TALK_STEPS[step % len(_TALK_STEPS)]
            await self._post({"seg": [{"id": SEG_TALK, "fx": FX_SOLID,
                                        "i": self._talk_frame(lit)}]})
            step += 1
            await asyncio.sleep(_TALK_FRAME_SECS)

    async def _blank_talk_leds(self):
        """Explicitly clear every individually-addressed LED in the mouth
        segment — a plain `col` update does NOT clear pixels previously set
        via the `i` (individual LED) array, they're two separate WLED
        code paths, so `col`-only left the last-lit pair stuck on."""
        await self._post({"seg": [{"id": SEG_TALK, "fx": FX_SOLID,
                                    "i": self._talk_frame(())}]})

    @staticmethod
    def _talk_frame(lit: tuple) -> list:
        frame = []
        for i in range(TALK_LEN):
            frame += [i, COLOR_TALK if i in lit else COLOR_OFF]
        return frame

    # ------------------------------------------------------------------
    # Segment 2 — main status
    # ------------------------------------------------------------------

    async def set_status(self, voltage: float | None, charging: bool, has_problem: bool):
        """Battery-voltage colour on the status segment.

        A currently-active fault (`has_problem`, from /diagnostics WARN/ERROR)
        overrides the voltage colour with a blinking red.
        """
        if has_problem:
            await self._set_segment(SEG_STATUS, COLOR_RED, fx=FX_BLINK, speed=150)
        elif charging or voltage is None or voltage >= 12.8:
            await self._set_segment(SEG_STATUS, COLOR_BLUE)
        elif voltage >= 12.0:
            await self._set_segment(SEG_STATUS, COLOR_AQUA)
        else:
            await self._set_segment(SEG_STATUS, COLOR_RED)

    # ------------------------------------------------------------------
    # Transport
    # ------------------------------------------------------------------

    async def _set_segment(self, seg_id: int, rgb: list, brightness: int = 255,
                            fx: int = FX_SOLID, speed: int = 128):
        payload = {"on": True,
                   "seg": [{"id": seg_id, "col": [rgb], "bri": brightness,
                            "fx": fx, "sx": speed}]}
        await self._post(payload)

    async def _post(self, payload: dict):
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._post_blocking, payload)

    def _post_blocking(self, payload: dict):
        """Blocking HTTP POST — run in executor."""
        try:
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                self._url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                logger.debug(f"WLED response: {resp.status}")
        except Exception as e:
            logger.warning(f"WLED request failed: {e}")
