# PJ1 单目相机标定

用同一台相机拍摄的三张棋盘照片，估计镜头焦距、主点和畸变系数。程序还会标出棋盘角点，并生成校正前后的对照图。本分支一次运行即可得到三种方法的对比结果。

| 想运行的实验 | 分支 |
| --- | --- |
| 张正友固定棋盘法 | [main](https://github.com/eggpig/pj1-camera-calibration/tree/main) |
| 固定棋盘法与 Strobl 2011 | [refine-board](https://github.com/eggpig/pj1-camera-calibration/tree/refine-board) |
| 上述两种方法与Huang 等（2024）的单目方法 | [planar-constraint](https://github.com/eggpig/pj1-camera-calibration/tree/planar-constraint) |

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

使用 Python 3.12。该分支使用 NumPy 2.2.6、OpenCV 4.12.0 和 SciPy 1.15.3，安装版本由 `requirements.txt` 固定。程序在 CPU 上运行。本项目在 macOS Apple Silicon 上验证。

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
| `parameters.csv` | `fx, fy` 是水平、垂直像素焦距，`cx, cy` 是主点坐标。 `k1, k2, k3` 描述径向畸变，`p1, p2` 描述切向畸变。 |
| `reprojection_errors.csv` | 三种方法的逐图和整体误差。 `fit` 使用全部角点，`held_out` 使用固定的四折角点划分，`All` 汇总三张图。 |
| `repeat_runs.csv` | 每轮的试验编号、随机种子、方法、图像、点数和 RMSE。 |
| `repeat_summary.csv` | 每种方法的重复次数、整体 RMSE 均值、样本标准差、最小值和最大值。 |
| `corners.png` | 三张照片上的棋盘角点，每张 81 个。 |
| `undistortion.png` | `18.jpg` 的原图和三种校正结果。 |

## 三种方法

程序先找出照片里的棋盘交点，再调整模型参数，使程序算出的交点位置贴近实际检测到的位置。三种方法都采用 Huang 论文式（1）–（3）的针孔模型，估计四项内参和五项畸变系数。

- **固定棋盘（张正友法）**把棋盘角点放在标称间距为 50 mm 的理想网格上，优化相机参数和每张图的棋盘姿态。
- **Strobl 与 Hirzinger（2011）**同时优化相机参数和棋盘角点的三维位置。用 OpenCV 的 `calibrateCameraRO` 固定两个角点的位置和第三个角点的高度，确定棋盘坐标系和尺度。
- **Huang 等（2024）**从固定棋盘的解开始，按原文式（6）为每个角点加入三维偏移，按式（7）要求 X、Y、Z 三个方向的偏移分别加总为零，再按式（8）联合最小化重投影误差平方和。用 Helmert 基底表达零和偏移，通过 SciPy 信赖域算法求解。原文列出了目标与约束，求解器和收敛参数由本项目选定。

Huang 的零和条件固定的是棋盘中心。棋盘绕中心整体旋转或等比缩放时，可以同步调整每张图的棋盘姿态，使零和条件和像素投影同时成立。两种联合优化都能调整同一批三维角点的位置，因此本实验的投影几乎相同。用全量数据求解，两种方法对同一角点的预测位置相差约 0.000011 像素 RMS。

## 重跑结果

| 方法 | 全量拟合 RMSE（像素） | 固定四折留出 RMSE（像素） | 20 次打乱分组留出 RMSE（均值 ± 样本标准差，像素） |
| --- | ---: | ---: | ---: |
| 固定棋盘 | 0.903596 | 0.645471 | 0.665602 ± 0.008555 |
| Strobl 与 Hirzinger（2011） | 0.078619 | 0.180365 | 0.181594 ± 0.002815 |
| Huang 等（2024） | 0.078619 | 0.180366 | 0.181594 ± 0.002815 |

- **全量拟合**用 243 个角点观测求相机参数，再测量预测位置与检测位置之间的误差。
- **固定划分重复 5 次**。四组按行列奇偶划分，每轮用三组角点做标定。把余下的一组同时从三张图的拟合输入中移出，再用两张图里的留出点推算第三张图的像素位置。每次得到 234 个预测。全量拟合和固定留出各执行了五次，逐次结果相同，样本标准差为零。
- **随机划分重复 20 次**。用种子 0 至 19 打乱 78 个可留出的角点，保留四组的点数。每轮三个方法使用相同划分，Strobl 用于定坐标的三个点始终参与拟合。标准差反映这三张图的结果对角点划分有多敏感。

默认执行上述完整流程。可用 `--fixed-repeats` 和 `--shuffled-repeats` 修改次数。只跑一次全量拟合和固定四折时，使用 `--fixed-repeats 1 --shuffled-repeats 0`。论文第 4.2 节的稳定性实验每次从 40 张双目图像中抽取 12 张，重复 20 次。本项目复现的是论文给出的单目模型和优化问题，使用课程提供的三张单目照片。

## 参考

- Zhang, Z. *A Flexible New Technique for Camera Calibration*. IEEE TPAMI, 2000. [论文](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/)
- Strobl, K. H. & Hirzinger, G. *More Accurate Pinhole Camera Calibration with Imperfect Planar Target*. ICCV Workshops, 2011. [论文](https://elib.dlr.de/71888/)
- Huang, W. et al. *A planar constraint optimization method to improve camera calibration for imperfect planar targets*. Optics and Lasers in Engineering, 2024. [论文](https://doi.org/10.1016/j.optlaseng.2024.108273)
