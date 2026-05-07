
Bosch IMU: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi085-ds001.pdf

Bosch barometer: https://www.bosch-sensortec.com/media/boschsensortec/downloads/shuttle_board_flyer/application_board_3_1/bst-bmp581-sf000.pdf

TI Radar: https://www.ti.com/tool/IWR6843AOPEVM

Build and upload to teensy 4.1:
'pio run -e teensy41'
'pio run -e teensy41 --target upload'


Build and upload to teensy 4.0:
'pio run -e teensy40'
'pio run -e teensy40 --target upload'


Replay an SD-card log offline (desktop):
```
cmake -S tools/replay -B build/replay -DCMAKE_BUILD_TYPE=Release
cmake --build build/replay -j
./build/replay/rio_replay /path/to/LOG0001.CSV runs/baseline
python3 tools/replay/scripts/plot_runs.py runs/baseline
```
Edit `makeParams()` / `P0_diag` in `tools/replay/src/replay.cpp` to retune,
rebuild, write to a new output dir, and overlay runs:
```
python3 tools/replay/scripts/plot_runs.py runs/baseline runs/variant
```
