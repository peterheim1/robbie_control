#!/usr/bin/env python3
"""Quick test: start TCP audio server, wait for ESP32 connection, print mic audio stats."""

import asyncio
import logging
import struct
import signal

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

MSG_AUDIO_MIC = 0x01
HEADER_SIZE = 5


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    peer = writer.get_extra_info("peername")
    print(f"\n*** ESP32 connected from {peer} ***\n")

    total_bytes = 0
    total_frames = 0

    try:
        while True:
            header = await reader.readexactly(HEADER_SIZE)
            msg_type = header[0]
            length = struct.unpack(">I", header[1:5])[0]
            payload = await reader.readexactly(length)

            if msg_type == MSG_AUDIO_MIC:
                total_bytes += length
                total_frames += 1
                duration_s = total_bytes / (16000 * 2)
                if total_frames % 50 == 0:
                    print(
                        f"  frames={total_frames}  bytes={total_bytes}  "
                        f"duration={duration_s:.1f}s  last_chunk={length}B"
                    )
            else:
                print(f"  msg_type=0x{msg_type:02X}  length={length}")

    except asyncio.IncompleteReadError:
        print(f"\nESP32 disconnected ({peer})")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        writer.close()


async def main():
    server = await asyncio.start_server(handle_client, "0.0.0.0", 8765)
    addr = server.sockets[0].getsockname()
    print(f"TCP audio test server listening on {addr[0]}:{addr[1]}")
    print("Waiting for ESP32 connection... (Ctrl+C to stop)\n")

    stop = asyncio.Event()
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    await stop.wait()
    server.close()
    await server.wait_closed()
    print("Server stopped.")


asyncio.run(main())
