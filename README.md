# graph_visualizer

Builds and visualizes 3D RRT Graph (Path) from the native RRT library and Python bindings
(static 3D plot and optional GIF animation).  
Assets (PNG images) are read from `vis_artifacts/`. Outputs (plot and GIF) are written to `output/`.

## Required repositories

This project depends on two sibling repositories that must be present and built:

- rrt_graph_builder — the RRT C/C++ library sources and core implementation.  
  Remote: https://github.com/shedlock987/rrt_graph_builder

- rrt_demo_app — CMake project that builds the RRT library and produces the Python extension `rrtDemo.so`.  
  Remote: https://github.com/shedlock987/rrt_demo_app

Note: `rrt_demo_app` invokes the rrt_graph_builder build, compiles the native code and produces Python bindings (typically via pybind11 or Boost.Python). The resulting `rrtDemo.so` must be built with the same Python ABI as the interpreter used to run `graph_visualizer.py` and placed in `rrt_demo_app/build/` so the visualizer can import it.

## Quick start

1. Clone required repos as siblings of this repo:
```bash
# from your projects directory
git clone https://github.com/shedlock987/graph_visualizer.git
git clone https://github.com/shedlock987/rrt_graph_builder.git
git clone https://github.com/shedlock987/rrt_demo_app.git
```

2. Build the demo app (produce `rrtDemo.so`):
```bash
cd rrt_demo_app
mkdir -p build && cd build
cmake ..
make -j$(sysctl -n hw.ncpu)   # or `make -j4`
# this should create rrtDemo.so in rrt_demo_app/build/
```

3. Create & activate a venv (from `graph_visualizer` repo root)
```bash
python3 -m venv .venv
source .venv/bin/activate
```

4. Install Python dependencies
```bash
pip install --upgrade pip
pip install numpy matplotlib plotly pillow imageio kaleido
```

5. Run the visualizer
```bash
python3 graph_visualizer.py
```

- Static plot is written to `output/rrt_plot.jpeg`.
- If you choose to generate the GIF (or set `render_gif = True`), the GIF is written to `output/rrt_animation.gif`.
- PNG assets are loaded from `vis_artifacts/` (e.g. `vis_artifacts/blue_man.png`, `vis_artifacts/yield.png`).

## Configuration

- Toggle image rendering in `graph_visualizer.py`:
  - `render_png = True|False`
  - `render_gif = True|False`
- Image files and sizes are handled in `prepare_images()` inside the script.
- The script computes absolute paths for `vis_artifacts/` and `output/` relative to the script location.

## Installation dependencies

### macOS (Homebrew)
```bash
brew install python ffmpeg imagemagick pkg-config freetype libpng
```

### Debian / Ubuntu (apt)
```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip build-essential pkg-config \
                    libfreetype6-dev libpng-dev libjpeg-dev ffmpeg imagemagick
```

After system packages:
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy matplotlib plotly pillow imageio kaleido
```

## Build / debug notes

- If `rrtDemo.so` fails to import or you get a segfault, rebuild `rrt_demo_app` with debug symbols and ensure it was compiled against the same Python used in `.venv`.
- To make `graph_visualizer` find the extension, ensure `rrt_demo_app/build/rrtDemo.so` exists — the script appends that build directory to `sys.path`.
- If you need to change the build toolchain or enable sanitizers, edit `rrt_demo_app/CMakeLists.txt` and rebuild.

## Troubleshooting

- Missing `rrtDemo` import: confirm `rrt_demo_app/build/rrtDemo.so` exists and matches your Python.
- Segfaults: try running with a non-GUI backend to isolate (e.g. `MPLBACKEND=agg python3 graph_visualizer.py`) or debug native extension with lldb.
- PNGs still visible when disabled: ensure `render_png` is set to `False` and all image plotting is gated by `if render_png:` blocks.
- Duplicate `vis_artifacts/` directories: search the repo for other references and ensure ARTIFACT_DIR is computed once from `__file__`.

## Project layout
- graph_visualizer.py — main script
- vis_artifacts/ — input PNG assets (blue_man.png, yield.png, ...)
- output/ — generated `rrt_plot.jpeg` and `rrt_animation.gif`
- rrt_demo_app/ — external/native RRT build (must be built separately)
- rrt_graph_builder/ — RRT C/C++ library sources

## License
See `LICENSE` (MIT).
