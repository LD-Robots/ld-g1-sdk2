# G1 Audio Control SDK (Local)

Minimal project for Unitree G1 audio control using `unitree_sdk2`.

## Requirements
- `UNITREE_SDK2` exported to the root of your `unitree_sdk2` checkout.
- CMake 3.10+ and a C++17 compiler.

## Build
```bash
export UNITREE_SDK2=/path/to/unitree_sdk2
mkdir build && cd build
cmake -S .. -B .
cmake --build . -j
```

## Run (TTS test)
```bash
./g1_audio_tts_test eth0
```

Notes:
- Pass the correct network interface name for your robot connection.
