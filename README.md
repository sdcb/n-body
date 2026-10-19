# N·Body — 引力的交响 / A Gravitational Symphony

A real-time 3D gravity simulator for Windows, built on Direct3D 11 compute shaders, HDR bloom and a frosted-glass HUD.

一个基于 Direct3D 11 计算着色器、HDR 泛光和毛玻璃界面的实时 3D 引力模拟器。

![Galaxy collision](docs/galaxy-collision.jpg)

| | |
|---|---|
| ![Butterfly I](docs/butterfly-1.jpg) | ![Solar system](docs/solar-system.jpg) |
| ![Butterfly effect](docs/chaos-twins.jpg) | ![Protoplanetary disk](docs/protoplanetary-disk.jpg) |

## 特性 · Features

- **35 个场景，7 个类别 / 35 scenarios in 7 categories**
  - **星系 / Galaxies**: galaxy collision, Milky Way × Andromeda, a self-gravitating spiral. Up to 65k particles, computed with O(N²) direct summation on the GPU (tiled HLSL compute shader, leapfrog). Dark-matter halos and dynamical friction let the galaxies actually merge.
  - **三体周期解 / Periodic three-body**: Figure-8, the Šuvakov–Dmitrašinović family (butterflies, moths, dragonfly, yin-yang, bumblebee, yarn…), Broucke, Hénon, Lagrange.
  - **编舞与星环 / Choreographies**: rigidly rotating rings of 4, 6 and 12 bodies, and a moonlet ring.
  - **行星系统 / Planetary systems**: the real Solar System from JPL J2000 elements (G = 4π², AU, years), the inner Solar System, and a circumbinary "Tatooine" system.
  - **星团与盘 / Clusters & disks**: Plummer globular cluster, cold collapse, a protoplanetary disk with gap-carving planets.
  - **混沌 / Chaos**: the butterfly effect (two Pythagorean three-body systems whose initial conditions differ by 10⁻⁹) and the Pythagorean problem itself.
  - **沙盒 / Sandbox**: Shift + drag to launch new bodies; colliding bodies merge.
- **物理核心 / Physics core** (`Sdcb.NBody`, platform-independent, AOT-compatible): Structure-of-Arrays layout, SIMD plus parallel force summation, Plummer softening, and four integrators (Leapfrog, Yoshida 4, RK4, adaptive Dormand–Prince 5(4)). Energy, momentum and angular momentum are tracked live, with an energy-drift chart in the HUD.
- **视觉 / Visuals**: procedural starfield and nebula, HDR rendering, physically-inspired bloom, ACES tonemapping, fading GPU trails, stars with corona and diffraction spikes, lit planets with atmospheres, black holes with Doppler-beamed accretion disks.
- **体验 / Experience**: launches into a cinematic attract mode (auto camera, scenario carousel); bilingual zh/en UI switchable at runtime; procedurally synthesized ambient soundtrack (no audio assets); F12 screenshots; automatic quality tiers per GPU.
- **工程 / Engineering**: native Win32 window, no WinForms; Native AOT single-file exe (~3 MB); xUnit tests; GitHub Actions CI and releases.

## 操作 · Controls

| 输入 / Input | 功能 / Action |
|---|---|
| 左键拖动 / Left drag | 旋转视角 / Orbit camera |
| 右键或中键拖动 / Right or middle drag | 平移 / Pan |
| 滚轮 / Wheel | 缩放 / Zoom |
| 双击天体 / Double-click a body | 跟随 / Follow |
| `Space` | 暂停 / Pause |
| `←` `→` | 切换场景 / Previous or next scenario |
| `↑` `↓` | 加速 / 减速 (Faster / slower) |
| `G` | 场景画廊 / Scenario gallery |
| `I` | 切换积分器 / Cycle integrator |
| `T` | 轨迹开关 / Toggle trails |
| `R` | 重置 / Reset |
| `F` | 自动取景 / Re-frame camera |
| `Tab` | 隐藏界面 / Hide HUD |
| `L` | 中文 / English |
| `M` | 静音 / Mute |
| `F11` | 全屏 / Fullscreen |
| `F12` | 截图（保存到“图片\N-Body”）/ Screenshot (saved to Pictures\N-Body) |
| `H` | 帮助 / Help |
| `Esc` | 退出 / Quit |

## 构建 · Build

Requirements: Windows 10+, .NET 10 SDK, and a D3D11 GPU (feature level 11.0).

```powershell
dotnet run --project Sdcb.NBody.App                 # run (fullscreen)
dotnet run --project Sdcb.NBody.App -- --windowed   # run in a window
dotnet test Sdcb.NBody.Tests                        # unit tests
dotnet publish Sdcb.NBody.App -c Release -r win-x64 # Native AOT single exe
```

Command-line options:

```text
--windowed                start in a window
--scenario <id>           start directly in a scenario (e.g. galaxy-collision, figure-8)
--capture <file.png>      render to a PNG after --capture-at seconds, then exit
--check-shaders           compile every shader and report errors
```

Set the environment variable `NBODY_QUALITY` to `ultra`, `high`, `medium` or `low` to override the automatic quality tier.

## 项目结构 · Layout

```text
Sdcb.NBody/          physics core: BodySet, ForceModel, integrators, Simulation, Scenarios/
Sdcb.NBody.App/      renderer: Win32 window, D3D11 device, HLSL shaders, HUD, audio
Sdcb.NBody.Tests/    xUnit tests: conservation laws, periodic-orbit regression, scenario sanity
```

## License

MIT
