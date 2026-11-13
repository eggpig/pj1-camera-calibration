# PJ1 单目相机标定

把同一台相机从不同角度拍摄的棋盘照片交给程序，估计镜头成像参数，并生成角点图、去畸变图和误差表。当前分支是**三种方法对比**。在同一次运行中比较固定棋盘、Strobl 2011 和参考 Huang 2024 的单目实现。

## 1. 准备照片

解压课程附件 `PJ1--待标定图片2.rar`。把标定板 1 的 `11.jpg`、`12.jpg`、`18.jpg` 复制到仓库根目录的 `images/` 文件夹。三张图来自同一台相机，分辨率均为 1440 × 1080。使用原图尺寸和方向。程序已按这块棋盘设置 9 × 9 个内角点，相邻角点间距为 50 mm。三个分支都用这三张图。

```text
pj1-camera-calibration/
├── calibrate.py
├── requirements.txt
└── images/
    ├── 11.jpg
    ├── 12.jpg
    └── 18.jpg
```

## 2. 安装和运行

使用 Python 3.12。依赖为 NumPy 2.2.6、OpenCV 4.12.0、SciPy 1.15.3，具体包版本见 `requirements.txt`。在 macOS Apple Silicon 上已验证。程序使用 CPU，可在终端直接运行。安装依赖时需要访问 Python 包索引。

从 GitHub 获取代码。使用离线 ZIP 的同学可直接打开 `planar-constraint/` 文件夹，从创建虚拟环境开始。

```bash
git clone https://github.com/eggpig/pj1-camera-calibration.git
cd pj1-camera-calibration
git switch planar-constraint
```

在含有 `calibrate.py` 的目录执行。以下是 macOS 或 Linux 的命令。

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python calibrate.py --output results
```

Windows PowerShell 使用以下命令。

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe calibrate.py --output results
```

如果照片放在别处，在最后一条命令加上 `--images "你的图片目录"`。这个目录应直接包含三张 JPG。程序运行完成后会打印误差和输出目录。

## 3. 查看结果

| 文件 | 打开后看什么 |
| --- | --- |
| `results/parameters.csv` | `fx, fy` 是以像素计的水平和垂直焦距。 `cx, cy` 是光轴落在图像上的像素坐标。 `k1, k2` 描述镜头弯曲图像的程度。 |
| `results/reprojection_errors.csv` | 三种方法的拟合误差与留出预测误差。 `All` 汇总所有图，`rmse_px` 的单位为像素。 |
| `results/corners.png` | 每张图上标出的 81 个内角点。内角点是棋盘内部四个黑白格的交点。 |
| `results/undistortion.png` | 以 `18.jpg` 为例展示原图和三种方法的校正结果。 |

## 4. 方法和实测

三种方法都先用 OpenCV 在照片中找出棋盘角点，再调整相机参数，使程序预测的角点位置尽量贴近实际检测位置。区别在于棋盘的三维坐标是否参与调整，以及怎样给棋盘定坐标。

| 方法 | 怎样处理棋盘 | 分支 |
| --- | --- | --- |
| 固定棋盘 | 把角点放在间距 50 mm 的理想平面上，估计相机参数及每张图的棋盘姿态。 | [main](https://github.com/eggpig/pj1-camera-calibration/tree/main) |
| Strobl 2011 | 同时估计角点的三维位置，用三个角点的部分坐标定原点、方向和尺度。 | [refine-board](https://github.com/eggpig/pj1-camera-calibration/tree/refine-board) |
| 参考 Huang 2024 | 同时估计每个角点相对理想网格的三维偏移，用整个网格定坐标和尺度。 | [planar-constraint](https://github.com/eggpig/pj1-camera-calibration/tree/planar-constraint) |

下表来自 `planar-constraint` 分支的同一次运行。两种误差均为重投影 RMSE，表示预测点与检测点相差多少像素，越小表示位置越接近。

| 方法 | 全量拟合（243 个观测） | 留出预测（234 个预测） |
| --- | ---: | ---: |
| 固定棋盘 | 0.915 px | 0.637 px |
| Strobl 2011 | 0.095 px | 0.215 px |
| 参考 Huang 2024 | 0.095 px | 0.215 px |

`fit` 使用全部角点来求参数，再用这些参数计算角点位置。`held_out` 把棋盘按行列奇偶分成四组，每轮先从三张图移出同一组角点，原有的三个锚点始终参与拟合。用其余角点求参数，再用两张图中的留出点估计其三维位置，预测第三张图里的位置。轮换三张图，得到每图 78 个预测。

两种联合优化方法在这三张图上的结果几乎相同。三种方法共用相同的镜头模型和检测点。两种联合优化允许的棋盘形状相同，主要区别是怎样确定坐标和尺度。

### 参考 Huang 2024 的实现

出版社公开预览给出了“角点相对理想平面的偏移总和为零”的思路。论文第 3 节的完整约束公式未能核对。本分支从固定棋盘的解开始，将角点相对理想网格的三维偏移与相机参数一起拟合。用下面三个条件确定棋盘的坐标和尺度，再用 SciPy 的非线性最小二乘求解。

- 所有角点偏移的和为零，防止棋盘整体平移。
- 偏移对理想棋盘三条转轴的整体转动分量为零，固定坐标方向。
- 偏移在理想网格整体缩放方向上的分量为零，沿用 50 mm 网格给出的尺度。

以棋盘中心为原点，令 (q_i) 为理想角点相对中心的坐标，(d_i) 为角点偏移。实现中的条件为 `Σdᵢ = 0`、`Σ(qᵢ × dᵢ) = 0` 和 `Σ(qᵢ · dᵢ) = 0`。每轮只用当轮参与拟合的角点建立约束。

## 参考

- Zhang, Z. *A Flexible New Technique for Camera Calibration*. IEEE TPAMI, 2000. [论文](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/)
- Strobl, K. H. & Hirzinger, G. *More Accurate Pinhole Camera Calibration with Imperfect Planar Target*. ICCV Workshops, 2011. [论文](https://elib.dlr.de/71888/)
- Huang, W. et al. *A planar constraint optimization method to improve camera calibration for imperfect planar targets*. Optics and Lasers in Engineering, 2024. [出版社页面](https://doi.org/10.1016/j.optlaseng.2024.108273)
