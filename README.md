# PJ1 单目相机标定

用 Python 和 OpenCV 标定同一台相机拍摄的 `11.jpg`、`12.jpg` 和 `18.jpg`。三张图来自课程的 `PJ1--待标定图片2.rar`，原始分辨率为 1440 × 1080。标定板 1 有 9 × 9 个内角点，相邻内角点的标称间距为 50 mm。

## 环境和运行

本地验证环境为 macOS Apple Silicon、Python 3.12.14、OpenCV 4.12.0、NumPy 2.2.6。依赖已固定，使用 CPU，无需 MATLAB、GPU 或图形界面。安装时需要能访问 Python 包索引。其他平台可使用对应的 Python 3.12 和依赖 wheel。

把课程压缩包解开，定位到直接包含 `11.jpg`、`12.jpg`、`18.jpg` 的目录。以下命令在仓库根目录执行。

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python calibrate.py --images "/path/to/PJ1--待标定图片2" --output results
```

Windows PowerShell 可用 `py -3.12 -m venv .venv` 创建环境，后两条命令把 `.venv/bin/python` 改为 `.\.venv\Scripts\python.exe`。也可把三张图复制到脚本旁的 `images/` 目录，省略 `--images`。输入保持原尺寸和方向。

| 输出文件 | 内容 |
| --- | --- |
| `parameters.csv` | 内参 `fx, fy, cx, cy`，单位 px，以及无量纲畸变系数 `k1, k2` |
| `reprojection_errors.csv` | 逐图和全局重投影 RMSE，单位 px |
| `corners.png` | 三张原图的亚像素角点检测结果 |
| `undistortion.png` | `18.jpg` 的原图与去畸变图 |

## 基线方法和结果

`findChessboardCornersSB` 提取每图 81 个亚像素内角点。按照 50 mm 间距构造平面棋盘坐标，再由 `calibrateCamera` 估计相机内参、两项径向畸变和每张图的位姿。用 `projectPoints` 回投棋盘角点。RMSE 为二维像素残差的平方和除以观测数后开平方。

两种方法的实测结果见下文。数据来自课程附件，复现时可使用本地提供的三张原图。

## 参考

- Zhang, Z. *A Flexible New Technique for Camera Calibration*. IEEE TPAMI, 2000. [论文](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/)
- OpenCV 4.12.0. [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html)

## 联合优化棋盘几何

本分支在一次运行中比较两种方法。两者使用同样的图像、角点、针孔模型与双径向畸变参数。固定棋盘法把所有角点放在间距为 50 mm 的理想平面上。联合优化法用 OpenCV 的 `calibrateCameraRO` 同时拟合相机参数、每张图的位姿以及三张图共享的棋盘三维角点。第一点和首行末点固定，末点的高度固定，以确定坐标系和尺度。第一点至首行末点的 400 mm 距离取自课件给出的标称格长。

这项实验采用 Strobl 和 Hirzinger 的方法在 OpenCV 中的实现。Huang 等人在 2024 年也研究了不完美标定板，提出基于虚拟理想平面的约束优化。GeoCalib 和 CalibAnyView 则用自然图像线索估计内参，面向无标定板的场景。

| 方法 | fx (px) | fy (px) | cx (px) | cy (px) | k1 | k2 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 固定棋盘 | 1084.27 | 1079.38 | 725.06 | 532.90 | 0.15559 | −0.35789 |
| 联合优化 | 1076.24 | 1074.60 | 724.01 | 530.57 | 0.13014 | −0.26810 |

| 图像 | 固定棋盘拟合 | 联合优化拟合 | 固定棋盘留出 | 联合优化留出 |
| --- | ---: | ---: | ---: | ---: |
| 11.jpg | 0.748 | 0.103 | 0.742 | 0.218 |
| 12.jpg | 1.057 | 0.086 | 0.569 | 0.192 |
| 18.jpg | 0.913 | 0.095 | 0.585 | 0.232 |
| 全部 | **0.915** | **0.095** | **0.637** | **0.215** |

表中误差均为 px。拟合评估使用全部 243 个观测。留出评估按棋盘行列的奇偶分成四组，三个坐标锚点始终参与拟合。每轮从三张图同时移除同一组角点。标定完成后，用其中两张图的留出角点三角化，预测第三张图的同名角点，轮换三张图并汇总 234 个预测。两种方法共用这一评估流程。

联合优化后，拟合 RMSE 下降 89.6%，留出 RMSE 下降 66.3%。水平和垂直焦距的变化均小于 1%。结果反映了三张图之间的几何一致性。若要量化内参的绝对精度，需要独立视角或参考内参。

输出文件名和运行命令与主分支相同。本分支的两个 CSV 包含两种方法，`reprojection_errors.csv` 同时给出 `fit` 和 `held_out`。去畸变图依次展示原图、固定棋盘和联合优化的结果。

- Strobl, K. H. & Hirzinger, G. *More Accurate Pinhole Camera Calibration with Imperfect Planar Target*. ICCV Workshops, 2011. [论文](https://elib.dlr.de/71888/)
- Huang, W. et al. *A planar constraint optimization method to improve camera calibration for imperfect planar targets*. Optics and Lasers in Engineering, 2024. [论文](https://doi.org/10.1016/j.optlaseng.2024.108273)
- Veicht, A. et al. *GeoCalib: Learning Single-image Calibration with Geometric Optimization*. ECCV, 2024. [论文](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/05636.pdf)
- *CalibAnyView: Beyond Single-View Camera Calibration in the Wild*. 2026 年预印本. [论文](https://arxiv.org/abs/2605.14615)
