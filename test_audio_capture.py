#!/usr/bin/env python3
"""Capture 10 seconds of audio from ESP32 TCP stream and save to WAV file.

Usage: python3 robbie_control/test_audio_capture.py
Then say something near the ReSpeaker to test audio quality.
"""

import asyncio
import struct
import wave
import signal

LISTEN_PORT = 8765
SAMPLE_RATE = 16000
DURATION_SECS = 10
MSG_AUDIO_MIC = 0x01
HEADER_SIZE = 5


async def main():
    audio_buf = bytearray()
    target_bytes = SAMPLE_RATE * 2 * DURATION_SECS

    async def handle_client(reader, writer):
        nonlocal audio_buf
        peer = writer.get_extra_info("peername")
        print(f"ESP32 connected from {peer}")
        print(f"Recording {DURATION_SECS}s of audio...")

        try:
            while len(audio_buf) < target_bytes:
                header = await reader.readexactly(HEADER_SIZE)
                msg_type = header[0]
                length = struct.unpack(">I", header[1:5])[0]
                payload = await reader.readexactly(length)

                if msg_type == MSG_AUDIO_MIC:
                    audio_buf.extend(payload)
                    secs = len(audio_buf) / (SAMPLE_RATE * 2)
                    if len(audio_buf) % (SAMPLE_RATE * 2) < length:
                        print(f"  {secs:.0f}s captured...")
        except asyncio.IncompleteReadError:
            print("ESP32 disconnected")
        finally:
            writer.close()

    server = await asyncio.start_server(handle_client, "0.0.0.0", LISTEN_PORT)
    print(f"Listening on port {LISTEN_PORT}, waiting for ESP32...")

    stop = asyncio.Event()
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    # Wait for enough audio or Ctrl+C
    while len(audio_buf) < target_bytes and not stop.is_set():
        await asyncio.sleep(0.5)

    server.close()
    await server.wait_closed()

    # Save to WAV
    out_path = "/tmp/respeaker_test.wav"
    with wave.open(out_path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(bytes(audio_buf))

    duration = len(audio_buf) / (SAMPLE_RATE * 2)
    print(f"\nSaved {duration:.1f}s of audio to {out_path}")
    print(f"Play it with: aplay {out_path}")


asyncio.run(main())
