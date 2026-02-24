# Voice Control - Project Status

**Date**: 2026-02-18
**Status**: Wake word + full pipeline working on desktop. Ready to move to robot.

---

## Quick Start

```bash
~/ai_projects/voice_control/start.sh
```

Say **"hey Jarvis"** then a command (e.g. "look right", "go forward", "stop").

---

## What Works (Desktop - AM4137 USB mic, GPU PC)

- **Wake word**: openwakeword `hey_jarvis` triggers reliably (scores 0.85–0.95)
- **STT**: Faster-Whisper (small, CUDA) transcribes speech correctly
- **Intent classifier**: "look right" → direction=right, pan=-1.2, tilt=0.0
- **ROS 2 dispatcher**: publishes to /voice/intent, /head/position, /cmd_vel etc
- **TTS**: Piper `en_GB-alan-medium` (British male voice, Jarvis-like)
- **Audio**: Captures at 16kHz directly via sounddevice (PipeWire resamples internally)
- **Config**: `audio_source: "local_mic"` in `config/voice_config.yaml`

---

## Audio Pipeline (current working approach)

```
AM4137 USB mic
  ↓ sounddevice (16kHz, 1024 block, int16)
  ↓ openwakeword hey_jarvis (threshold 0.85, int16 input)
  ↓ webrtcvad (VAD end-of-speech, ~1.5s silence)
  ↓ Faster-Whisper STT (CUDA)
  ↓ Intent classifier
  ↓ ROS 2 dispatcher
  ↓ Piper TTS (en_GB-alan-medium)
  ↓ sounddevice playback (mic paused during playback)
```

Key lessons learned:
- Capture at **16kHz natively** - let sounddevice/PipeWire resample internally
- Pass **raw int16** to openwakeword `predict()` - not float32
- Block size **1024** samples works well
- Threshold **0.85** gives good balance of sensitivity vs false positives

---

## Moving to the Robot

### Robot hardware
- i7 CPU, 8GB RAM, Ubuntu 24.04, ROS 2 Jazzy
- USB mic: AM4137 (or eventually ReSpeaker Lite when DFU issue resolved)
- **No GPU** → change STT device to `cpu` in config

### Setup on robot PC

```bash
# 1. Clone / copy repo
git clone <repo> ~/ai_projects   # or rsync from desktop

# 2. Create venv and install deps
cd ~/ai_projects/voice_control
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 3. Install openwakeword models
python -c "from openwakeword.model import Model; Model()"

# 4. Download Piper voice model
mkdir -p models/piper
cd models/piper
wget "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_GB/alan/medium/en_GB-alan-medium.onnx"
wget "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_GB/alan/medium/en_GB-alan-medium.onnx.json"

# 5. Install udev rule for AM4137 input events (if using mute button later)
sudo cp ~/ai_projects/50-am4137-input.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo usermod -aG input $USER   # log out/in after this

# 6. Change STT to CPU in config (robot has no GPU)
# Edit voice_control/config/voice_config.yaml:
#   stt:
#     device: "cpu"    ← change from "cuda"

# 7. Run
~/ai_projects/voice_control/start.sh
```

### Config changes needed on robot

Edit `config/voice_config.yaml`:
```yaml
stt:
  device: "cpu"          # robot has no GPU (was "cuda" on desktop)
  model: "tiny"          # use tiny model on CPU for speed (was "small")
```

### Transferring from desktop (alternative to git)

```bash
# From desktop - copy everything except large binaries and venv
rsync -av --exclude='.venv' --exclude='__pycache__' --exclude='*.bin' \
  ~/ai_projects/voice_control/ robot@<robot-ip>:~/ai_projects/voice_control/
```

---

## ReSpeaker Lite Status (deferred)

The ReSpeaker Lite (Seeed, USB VID:PID 2886:0019) is stuck in DFU boot loop.
- Hardware bcdDevice=1.10, tried firmware v2.0.7 (USB) and v1.1.0 (I2S) - both boot back to DFU
- Likely firmware version mismatch with this hardware revision
- **Not blocking** - AM4137 USB mic works well as replacement
- When resolved: change `audio_source: "tcp"` in config and flash ESP32 firmware

---

## Architecture

```
USB mic (AM4137)                          Robot PC (i7, no GPU)
┌─────────────────────────────────────────────────────────────┐
│ sounddevice 16kHz int16                                      │
│ → openwakeword hey_jarvis (thr 0.85)                        │
│ → Faster-Whisper STT (CPU, tiny model)                      │
│ → Intent classifier (regex + fuzzy)                         │
│ → Fast-path stop detector                                   │
│ → ROS 2 dispatcher (/voice/intent, /head/position, etc)     │
│ → Piper TTS (en_GB-alan-medium)                             │
│ → sounddevice playback                                      │
└─────────────────────────────────────────────────────────────┘
```

---

## Key Files

| File | Purpose |
|---|---|
| `start.sh` | Start the voice server (short command) |
| `robbie_voice_server.py` | Main orchestrator |
| `local_mic_client.py` | USB mic capture + wake word + VAD + TTS playback |
| `stt_engine.py` | Faster-Whisper wrapper |
| `intent_classifier.py` | Text → intent (regex + fuzzy) |
| `tts_engine.py` | Piper TTS subprocess wrapper |
| `ros2_dispatcher.py` | ROS 2 topic publisher |
| `tcp_audio_client.py` | TCP server for ReSpeaker Lite (when ready) |
| `config/voice_config.yaml` | Main config (audio source, models, thresholds) |
| `config/intents.yaml` | Intent patterns and response templates |
| `models/piper/en_GB-alan-medium.onnx` | British TTS voice (60MB) |

---

## Next Steps

1. **Set up on robot PC** (follow setup steps above)
2. **Test on robot** - same hey Jarvis commands, verify ROS 2 topics publish correctly
3. **Tune STT on CPU** - tiny model may be less accurate; test and adjust
4. **Fix ReSpeaker Lite DFU** - switch to TCP audio when resolved
5. **Test all intents** - drive, stop, dock, query_status, goto
6. **Pull Mistral**: `ollama pull mistral` for general Q&A support
