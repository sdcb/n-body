# N·Body — 引力的交响 [![QQ](https://img.shields.io/badge/QQ_Group-495782587-52B6EF?style=social&logo=tencent-qq&logoColor=000&logoWidth=20)](http://qm.qq.com/cgi-bin/qm/qr?_wv=1027&k=mma4msRKd372Z6dWpmBp4JZ9RL4Jrf8X&authKey=gccTx0h0RaH5b8B8jtuPJocU7MgFRUznqbV%2FLgsKdsK8RqZE%2BOhnETQ7nYVTp1W0&noverify=0&group_code=495782587)

**中文** | [English](README.en.md)

一个基于 Direct3D 11 计算着色器、HDR 泛光和毛玻璃界面的 Windows 实时 3D 引力模拟器。

![星系碰撞](docs/galaxy-collision.jpg)

| | |
|---|---|
| ![蝴蝶 I](docs/butterfly-1.jpg) | ![太阳系](docs/solar-system.jpg) |
| ![蝴蝶效应](docs/chaos-twins.jpg) | ![原行星盘](docs/protoplanetary-disk.jpg) |

## 特性

- **7 个类别，共 35 个场景**
  - **星系**：星系碰撞、银河系 × 仙女座星系、自引力旋涡星系。最多 65k 个粒子，在 GPU 上以 O(N²) 直接求和计算（分块 HLSL 计算着色器 + 蛙跳积分）。暗物质晕与动力学摩擦让星系能够真正合并。
  - **三体周期解**：8 字形解、Šuvakov–Dmitrašinović 族（蝴蝶、飞蛾、蜻蜓、阴阳、大黄蜂、毛线团……）、Broucke、Hénon、拉格朗日解。
  - **编舞与星环**：4、6、12 体刚性旋转环，以及小卫星环。
  - **行星系统**：基于 JPL J2000 轨道根数的真实太阳系（G = 4π²，单位为 AU、年）、内太阳系，以及环双星的“塔图因”系统。
  - **星团与盘**：Plummer 球状星团、冷坍缩、带有行星开辟空隙的原行星盘。
  - **混沌**：蝴蝶效应（两个初始条件仅相差 10⁻⁹ 的毕达哥拉斯三体系统）以及毕达哥拉斯三体问题本身。
  - **沙盒**：Shift + 拖动发射新天体；碰撞的天体会合并。
- **物理核心**（`Sdcb.NBody`，平台无关，兼容 AOT）：结构数组（SoA）布局，SIMD 加并行的引力求和，Plummer 软化，以及四种积分器（Leapfrog、Yoshida 4、RK4、自适应 Dormand–Prince 5(4)）。实时跟踪能量、动量和角动量，HUD 中带有能量漂移曲线。
- **视觉**：程序化生成的星空与星云、HDR 渲染、基于物理的泛光、ACES 色调映射、渐隐的 GPU 轨迹、带日冕和衍射星芒的恒星、带大气层的受光行星、带多普勒增亮吸积盘的黑洞。
- **体验**：启动后进入电影式演示模式（自动镜头、场景轮播）；运行时可切换的中英双语界面；程序化合成的环境配乐（无音频资源文件）；F12 截图；按 GPU 自动选择画质档位。
- **工程**：原生 Win32 窗口，不依赖 WinForms；Native AOT 单文件 exe（约 3 MB）；xUnit 测试；GitHub Actions 持续集成与发布。

## 操作

| 输入 | 功能 |
|---|---|
| 左键拖动 | 旋转视角 |
| 右键或中键拖动 | 平移 |
| 滚轮 | 缩放 |
| 双击天体 | 跟随 |
| `Space` | 暂停 |
| `←` `→` | 上一个 / 下一个场景 |
| `↑` `↓` | 加速 / 减速 |
| `G` | 场景画廊 |
| `I` | 切换积分器 |
| `T` | 轨迹开关 |
| `R` | 重置 |
| `F` | 自动取景 |
| `Tab` | 隐藏界面 |
| `L` | 中文 / English |
| `M` | 静音 |
| `F11` | 全屏 |
| `F12` | 截图（保存到“图片\N-Body”） |
| `H` | 帮助 |
| `Esc` | 退出 |

## 构建

环境要求：Windows 10 及以上、.NET 10 SDK，以及支持 D3D11（特性级别 11.0）的显卡。

```powershell
dotnet run --project Sdcb.NBody.App                 # 运行（全屏）
dotnet run --project Sdcb.NBody.App -- --windowed   # 窗口模式运行
dotnet test Sdcb.NBody.Tests                        # 单元测试
dotnet publish Sdcb.NBody.App -c Release -r win-x64 # Native AOT 单文件 exe
```

命令行参数：

```text
--windowed                以窗口模式启动
--scenario <id>           直接进入指定场景（如 galaxy-collision、figure-8）
--capture <file.png>      在 --capture-at 秒后渲染为 PNG，然后退出
--check-shaders           编译所有着色器并报告错误
```

设置环境变量 `NBODY_QUALITY` 为 `ultra`、`high`、`medium` 或 `low` 可覆盖自动画质档位。

## 项目结构

```text
Sdcb.NBody/          物理核心：BodySet、ForceModel、积分器、Simulation、Scenarios/
Sdcb.NBody.App/      渲染器：Win32 窗口、D3D11 设备、HLSL 着色器、HUD、音频
Sdcb.NBody.Tests/    xUnit 测试：守恒律、周期轨道回归、场景合理性检查
```

## 微信群

![](https://io.starworks.cc:88/cv-public/2026/ocr-wxg-qr.png?1022)

如果微信群二维码过期了，请加入 QQ 群 [.NET骚操作 495782587](http://qm.qq.com/cgi-bin/qm/qr?_wv=1027&k=mma4msRKd372Z6dWpmBp4JZ9RL4Jrf8X&authKey=gccTx0h0RaH5b8B8jtuPJocU7MgFRUznqbV%2FLgsKdsK8RqZE%2BOhnETQ7nYVTp1W0&noverify=0&group_code=495782587)。

## 许可证

[MIT](LICENSE.txt)
