# PJ1 单目相机标定

用同一台相机拍摄的三张棋盘照片，估计镜头焦距、主点和畸变系数。程序还会标出棋盘角点，并生成校正前后的对照图。本分支运行张正友固定棋盘标定。

| 想运行的实验 | 分支 |
| --- | --- |
| 张正友固定棋盘法 | [main](https://github.com/eggpig/pj1-camera-calibration/tree/main) |
| 固定棋盘法与 Strobl 2011 | [refine-board](https://github.com/eggpig/pj1-camera-calibration/tree/refine-board) |
| 上述两种方法与Huang 等（2024）的单目标定 | [planar-constraint](https://github.com/eggpig/pj1-camera-calibration/tree/planar-constraint) |

## 运行代码

### 1. 下载项目

```bash
git clone https://github.com/eggpig/pj1-camera-calibration.git
cd pj1-camera-calibration
```

使用离线 ZIP 时，解压后打开其中的 `main/` 目录，直接进行下一步。

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

使用 Python 3.12。该分支使用 NumPy 2.2.6、OpenCV 4.12.0，安装版本由 `requirements.txt` 固定。程序在 CPU 上运行。本项目在 macOS Apple Silicon 上验证。

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
| `reprojection_errors.csv` | 每张图的拟合误差。 `All` 是三张图的汇总，`rmse_px` 的单位是像素。 |
| `corners.png` | 三张照片上的棋盘角点。每张标出 81 个黑白格交点。 |
| `undistortion.png` | 以 `18.jpg` 为例，展示原图和固定棋盘法校正后的图。 |

## 方法与结果

固定棋盘法把每个角点放在理想的方格上，估计镜头参数和棋盘在每张照片中的姿态。本分支使用两项径向畸变系数，三张图的拟合 RMSE 为 0.915 像素。

三种方法的最新对比统一使用 Huang 原文的五项畸变系数。固定划分重复了 5 次，随机角点划分重复了 20 次。运行方法、每轮记录及汇总见 [三方法对比分支](https://github.com/eggpig/pj1-camera-calibration/tree/planar-constraint#重跑结果)。

## 参考

- Zhang, Z. *A Flexible New Technique for Camera Calibration*. IEEE TPAMI, 2000. [论文](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/)
- Strobl, K. H. & Hirzinger, G. *More Accurate Pinhole Camera Calibration with Imperfect Planar Target*. ICCV Workshops, 2011. [论文](https://elib.dlr.de/71888/)
- Huang, W. et al. *A planar constraint optimization method to improve camera calibration for imperfect planar targets*. Optics and Lasers in Engineering, 2024. [论文](https://doi.org/10.1016/j.optlaseng.2024.108273)
