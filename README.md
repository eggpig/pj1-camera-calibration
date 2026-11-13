# PJ1 单目相机标定

用同一台相机拍摄的三张棋盘照片，估计镜头焦距、主点和畸变系数。程序还会标出棋盘角点，并生成校正前后的对照图。本分支一次运行即可得到三种方法的对比结果。

| 想运行的实验 | 分支 |
| --- | --- |
| 张正友固定棋盘法 | [main](https://github.com/eggpig/pj1-camera-calibration/tree/main) |
| 固定棋盘法与 Strobl 2011 | [refine-board](https://github.com/eggpig/pj1-camera-calibration/tree/refine-board) |
| 上述两种方法与参考 Huang 2024 的单目实现 | [planar-constraint](https://github.com/eggpig/pj1-camera-calibration/tree/planar-constraint) |

## 运行代码

### 1. 下载项目

```bash
git clone --branch planar-constraint https://github.com/eggpig/pj1-camera-calibration.git
cd pj1-camera-calibration
```

使用离线 ZIP 时，解压后打开其中的 `planar-constraint/` 目录，直接进行下一步。

### 2. 放入三张照片

解压课程附件 `PJ1--待标定图片2.rar`，找到标定板 1 的 `11.jpg`、`12.jpg` 和 `18.jpg`。三张照片都是 1440 × 1080 像素。棋盘有 9 行、9 列内角点，相邻角点的标称间距为 50 mm。三个分支都使用这一组照片。

在 `calibrate.py` 旁边新建 `images/` 目录，把三张原图放进去，保留原来的文件名、尺寸和方向。

```text
pj1-camera-calibration/
├── calibrate.py
├── requirements.txt
└── images/
    ├── 11.jpg
    ├── 12.jpg
    └── 18.jpg
```

### 3. 安装依赖并运行

使用 Python 3.12。该分支使用NumPy 2.2.6、OpenCV 4.12.0 和 SciPy 1.15.3，安装版本由 `requirements.txt` 固定。程序在 CPU 上运行。本项目在 macOS Apple Silicon 上验证。

**macOS / Linux**

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python calibrate.py --output results
```

**Windows PowerShell**

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe calibrate.py --output results
```

如果照片已经放在别处，在运行命令末尾加上 `--images "图片目录"`。指定的目录应直接包含三张 JPG。运行结束后，终端会显示误差和输出目录。

## 查看输出

| `results/` 下的文件 | 内容 |
| --- | --- |
| `parameters.csv` | `fx` 和 `fy` 是以像素为单位的水平、垂直焦距。 `cx` 和 `cy` 是光轴落在照片上的位置。 `k1` 和 `k2` 描述像素偏移和它到主点的距离有什么关系。 |
| `reprojection_errors.csv` | 三种方法对每张图的拟合和留出误差。 `All` 是三张图的汇总，`rmse_px` 的单位是像素。 |
| `corners.png` | 三张照片上的棋盘角点。每张标出 81 个黑白格交点。 |
| `undistortion.png` | 以 `18.jpg` 为例，展示原图和三种方法校正后的图。 |

## 方法与结果

程序先找到照片里的棋盘交点，再用相机模型预测这些交点在照片中的位置。程序调整参数，让预测位置尽量贴近检测到的位置。

- **固定棋盘**把每个角点放在理想的方格上，估计镜头参数和棋盘在每张照片中的姿态。
- **Strobl 2011**还会调整角点在棋盘上的三维位置，用三个角点的部分坐标确定棋盘的原点、方向和尺度。
- **参考 Huang 2024 的单目实现**从理想网格出发，为每个角点估计一个三维偏移，用整块棋盘来确定原点、方向和尺度。

| 方法 | 全量拟合 RMSE（243 个观测） | 留出预测 RMSE（234 个预测） |
| --- | ---: | ---: |
| 固定棋盘 | 0.915 px | 0.637 px |
| Strobl 2011 | 0.095 px | 0.215 px |
| 参考 Huang 2024 | 0.095 px | 0.215 px |

表中数值来自三方法对比分支对同一组三张图的实测。本分支一次运行即可复算表中的全部结果。

全量拟合使用全部角点求参数，再检查模型预测的位置与检测位置相差多少。留出评估分四轮，每轮先从三张图中取出同一组角点，用剩下的点求参数。然后用其中两张图里的留出点推算三维位置，预测同一个点在第三张图的位置。三张图轮流作为预测目标，每张得到 78 个预测。Strobl 方法使用的三个坐标锚点在所有方法中始终参与拟合。

两种联合优化的留出误差都约为 0.215 像素。它们允许棋盘角点在三个方向移动，采用不同的坐标定义，最后对这三张照片中的角点给出了几乎相同的预测。

### 平面约束的实现

Huang 等人以理想平面为参照，让角点的偏移参与标定。出版社公开预览说明了偏移总和为零的条件。完整论文第 3 节的约束公式尚未核对，本分支以此思路实现单目版本。

程序先用固定棋盘法求出初始参数，再一起调整相机参数、每张图的棋盘姿态和每个角点的三维偏移。将参与拟合的理想角点相对棋盘中心的坐标记为 `qᵢ`，将其偏移记为 `dᵢ`。程序让 `Σdᵢ = 0`，并用 `Σ(qᵢ × dᵢ) = 0` 和 `Σ(qᵢ · dᵢ) = 0` 固定整体方向和尺度。最后用 SciPy 的非线性最小二乘求解。

## 参考

- Zhang, Z. *A Flexible New Technique for Camera Calibration*. IEEE TPAMI, 2000. [论文](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/)
- Strobl, K. H. & Hirzinger, G. *More Accurate Pinhole Camera Calibration with Imperfect Planar Target*. ICCV Workshops, 2011. [论文](https://elib.dlr.de/71888/)
- Huang, W. et al. *A planar constraint optimization method to improve camera calibration for imperfect planar targets*. Optics and Lasers in Engineering, 2024. [论文](https://doi.org/10.1016/j.optlaseng.2024.108273)
