
Bosch IMU: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi085-ds001.pdf

Bosch barometer: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf

TI Radar: https://www.ti.com/tool/IWR6843AOPEVM

Build and upload to teensy 4.1:
```
pio run -e teensy41
pio run -e teensy41 --target upload
```

Build and upload to teensy 4.0:
```
pio run -e teensy40
pio run -e teensy40 --target upload
```


Offline replay. `RUN_DIR` is any folder name; the binary writes
`state.csv`, `cov_diag.csv`, `radar_innov.csv`, `baro_innov.csv`
into it.
```
cmake -S tools/replay -B build/replay -DCMAKE_BUILD_TYPE=Release
cmake --build build/replay -j
./build/replay/rio_replay PATH_TO_CSV RUN_DIR
python3 tools/replay/scripts/plot_runs.py RUN_DIR
```

Retune by editing `makeParams()` / `kRadarParams` / `kBaroParams` /
`P0_diag` in `tools/replay/src/replay.cpp`. Write each run into its
own dir to keep the originals for comparison:
```
cmake --build build/replay -j
./build/replay/rio_replay PATH_TO_CSV RUN_DIR_2
python3 tools/replay/scripts/plot_runs.py RUN_DIR RUN_DIR_2
```

Compare a replay against PX4 EKF2 from a paired `.ulg`:
```
PATH_TO_VENV/bin/python3 tools/replay/scripts/compare_with_ulog.py \
    RUN_DIR PATH_TO_ULG --include_live
```
Outputs in `RUN_DIR/cmp/`. `--include_live` overlays the live
`vehicle_visual_odometry` trace from the `.ulg`; if it doesn't match
replay, the firmware and replay binary are out of sync.
