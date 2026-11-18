# Isaac Lab Install Paths, Visualizers and Livestream

Moved from SKILL.md. Verified against Isaac Lab 3.0.0 (tag `v3.0.0`) / Isaac Sim 6.2.0,
2026-11-18. Not executed: examples are checked against source, not run.
The recommended uv path and the verify steps stay in SKILL.md.

## Isaac Lab to Isaac Sim compatibility (from the Isaac Lab README)

- `v3.0.0`, `release/3.0.0` and `main` pair with 6.2. `v3.0.0-EA` and `-beta2` pair with 6.1
  and 6.0.
- `v2.3.x` pairs with 4.5/5.0/5.1, and `v2.0`-`v2.2` with 4.5 (2.2 also 5.0).
- Isaac Lab 3.0 rejects Isaac Sim 5.1 and older.

## First launch

- **Disk/network:** the first Isaac Sim launch downloads extensions and compiles shaders, which
  can take more than 10 minutes. Set `OMNI_KIT_ACCEPT_EULA=yes` for non-interactive runs.

## Other install paths

| Path | When | Key commands |
|------|------|--------------|
| `isaaclab.sh -i` (legacy) | You manage the venv/conda env yourself | `./isaaclab.sh -i` (core + mimic + teleop + RL); `-i 'isaacsim'`, `-i 'ov[ovrtx]'` |
| pip Isaac Sim in your env | Editable checkout plus full Isaac Sim | `uv pip install "isaacsim[all,extscache]==6.2.0.0" --extra-index-url https://pypi.nvidia.com --index-strategy unsafe-best-match --prerelease=allow`, then `./isaaclab.sh -i` |
| Isaac Lab wheel | Isaac Lab as a dependency of your project | `uv add --index-strategy unsafe-best-match --index nvidia=https://pypi.nvidia.com isaaclab==3.0.0`. Wheels exist only for major releases. The wheel has no `rl-games` and no robomimic |
| Binary Isaac Sim | Old GLIBC, or VS Code debugging inside Isaac Sim | Extract to `~/isaacsim`, `ln -s ~/isaacsim _isaac_sim`, `./isaaclab.sh -i`. Use the bundled `python.sh` only, never mixed with conda/uv |
| Docker | Clusters, CI, reproducibility | `./docker/container.py start [base\|ros2\|kitless]`, then `enter`, `copy`, `stop`. Prebuilt `nvcr.io/nvidia/isaac-lab:3.0.0` and `:3.0.0-kitless` |
| Cloud | No local RTX GPU (for example on a Mac) | Isaac Automator: `./run ./deploy-aws` (also `-gcp`, `-azure`, `-alicloud`) |

Docker gotchas: `container.py stop` deletes the named volumes, so run `copy` first. Images run
as uid/gid 1000, so `chown 1000:1000` bind-mounted cache dirs. Rendering needs
`--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all`.

## Headless, visualizers, livestream

In 3.0, no `--viz` means headless, and `--headless`/`--enable_cameras` are rejected (see
SKILL.md). Other options:

- `--viz kit,newton_gl` opens several visualizers. `--viz none` overrides any configured in code.
- Livestream: `--livestream 1` (public WebRTC) or `2` (LAN), or set the `LIVESTREAM` env var.
  This forces the Kit visualizer and binds TCP 49100. An `NVST_R_BUSY` error means a stale
  process still holds that port.
- `--viz viser` serves a web UI on port 8080. It is the easiest remote view without Kit.
- Isaac Sim alone (no Isaac Lab) runs headless with `SimulationApp({"headless": True})`. The
  binary package's WebRTC launcher is `isaac-sim.streaming.sh`.
