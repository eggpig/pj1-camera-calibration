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

| fx (px) | fy (px) | cx (px) | cy (px) | k1 | k2 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1084.27 | 1079.38 | 725.06 | 532.90 | 0.15559 | −0.35789 |

| 图像 | 观测数 | 拟合 RMSE (px) |
| --- | ---: | ---: |
| 11.jpg | 81 | 0.748 |
| 12.jpg | 81 | 1.057 |
| 18.jpg | 81 | 0.913 |
| 全部 | 243 | 0.915 |

改进方法和对比实验见 [`refine-board` 分支](../../tree/refine-board)。数据来自课程附件，复现时由运行者提供原图。

## 参考

- Zhang, Z. *A Flexible New Technique for Camera Calibration*. IEEE TPAMI, 2000. [论文](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/)
- OpenCV 4.12.0. [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html)
