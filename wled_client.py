"""WLED LED segment bar controller via HTTP JSON API.

Colors:
  idle      → green   [0, 255, 0]
  listening → magenta [255, 0, 255]
"""

import asyncio
import json
import logging
import urllib.request

logger = logging.getLogger(__name__)

COLOR_GREEN = [0, 255, 0]
COLOR_MAGENTA = [255, 0, 255]


class WLEDClient:
    """Controls a WLED instance via its HTTP JSON API."""

    def __init__(self, host: str, timeout: float = 2.0):
        self._url = f"http://{host}/json/state"
        self._timeout = timeout

    async def set_idle(self):
        """Green at 50% brightness — system ready, wake word listening."""
        await self._set_color(COLOR_GREEN, brightness=128)

    async def set_listening(self):
        """Magenta — actively capturing a voice response."""
        await self._set_color(COLOR_MAGENTA)

    async def _set_color(self, rgb: list, brightness: int = 200):
        payload = {"on": True, "bri": brightness, "seg": [{"col": [rgb]}]}
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._post, payload)

    def _post(self, payload: dict):
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
