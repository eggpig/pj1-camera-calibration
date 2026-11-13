"""Calibrate one camera from the three PJ1 checkerboard views."""

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import cv2 as cv
import numpy as np


IMAGE_NAMES = ("11.jpg", "12.jpg", "18.jpg")
PATTERN = (9, 9)  # Inner corners (columns, rows).
SQUARE_MM = 50.0
GRID = np.mgrid[:PATTERN[0], :PATTERN[1]].T.reshape(-1, 2)
BOARD = np.column_stack((SQUARE_MM * GRID, np.zeros(len(GRID)))).astype(np.float32)
DETECT_FLAGS = cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_EXHAUSTIVE | cv.CALIB_CB_ACCURACY
FIT_FLAGS = cv.CALIB_FIX_K3 | cv.CALIB_ZERO_TANGENT_DIST
CRITERIA = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_MAX_ITER, 200, 1e-10)


@dataclass
class View:
    name: str
    image: np.ndarray
    corners: np.ndarray


@dataclass
class Calibration:
    rms: float
    matrix: np.ndarray
    distortion: np.ndarray
    rotations: tuple
    translations: tuple
    board: np.ndarray


def load_views(directory: Path) -> tuple[list[View], tuple[int, int]]:
    views = []
    for name in IMAGE_NAMES:
        path = directory / name
        encoded = np.frombuffer(path.read_bytes(), dtype=np.uint8)
        if not encoded.size:
            raise ValueError(f"Empty image: {path}")
        image = cv.imdecode(encoded, cv.IMREAD_COLOR | cv.IMREAD_IGNORE_ORIENTATION)
        gray = cv.imdecode(encoded, cv.IMREAD_GRAYSCALE | cv.IMREAD_IGNORE_ORIENTATION)
        if image is None or gray is None:
            raise ValueError(f"Cannot decode image: {path}")
        if views and image.shape != views[0].image.shape:
            raise ValueError(f"Image dimensions differ: {path}")
        found, corners = cv.findChessboardCornersSB(gray, PATTERN, DETECT_FLAGS)
        if not found or len(corners) != len(BOARD):
            raise ValueError(f"Expected {len(BOARD)} checkerboard corners: {path}")
        views.append(View(name, image, corners))
    height, width = views[0].image.shape[:2]
    return views, (width, height)


def calibrate(board, observations, size) -> Calibration:
    rms, matrix, distortion, rotations, translations = cv.calibrateCamera(
        [board] * len(observations), observations, size,
        None, None, flags=FIT_FLAGS, criteria=CRITERIA,
    )
    return Calibration(rms, matrix, distortion, rotations, translations, board)


def pose_projection(fit: Calibration, points, index: int) -> np.ndarray:
    pixels, _ = cv.projectPoints(
        points.astype(np.float64), fit.rotations[index], fit.translations[index],
        fit.matrix, fit.distortion,
    )
    return pixels


def squared_errors(fit: Calibration, observations) -> np.ndarray:
    return np.array([
        np.sum((pose_projection(fit, fit.board, index) - corners) ** 2, axis=(1, 2))
        for index, corners in enumerate(observations)
    ])


def rmse(squared: np.ndarray) -> float:
    return float(np.sqrt(np.mean(squared)))


def write_csv(path: Path, headers, rows) -> None:
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(headers)
        writer.writerows(rows)


def write_tables(directory: Path, names, fits, metrics) -> None:
    parameters = [
        (name, fit.matrix[0, 0], fit.matrix[1, 1], fit.matrix[0, 2], fit.matrix[1, 2],
         *fit.distortion.ravel()[:2])
        for name, fit in fits
    ]
    write_csv(
        directory / "parameters.csv",
        ("method", "fx_px", "fy_px", "cx_px", "cy_px", "k1", "k2"),
        parameters,
    )
    errors = []
    for name, evaluation, squared in metrics:
        errors.extend(
            (name, evaluation, view, len(values), rmse(values))
            for view, values in zip(names, squared)
        )
        errors.append((name, evaluation, "All", squared.size, rmse(squared)))
    write_csv(
        directory / "reprojection_errors.csv",
        ("method", "evaluation", "image", "observations", "rmse_px"),
        errors,
    )


def save_panel(path: Path, panels) -> None:
    """Join labeled, equally scaled image panels on white."""
    tiles = []
    for title, image in panels:
        height, width = image.shape[:2]
        tile = cv.resize(image, (900, round(900 * height / width)), interpolation=cv.INTER_AREA)
        tile = cv.copyMakeBorder(tile, 56, 8, 8, 8, cv.BORDER_CONSTANT, value=(255,) * 3)
        cv.putText(tile, title, (12, 38), cv.FONT_HERSHEY_SIMPLEX,
                   1.05, (35, 35, 35), 2, cv.LINE_AA)
        tiles.append(tile)
    ok, encoded = cv.imencode(".png", cv.hconcat(tiles))
    if not ok:
        raise OSError(f"Could not encode figure: {path}")
    encoded.tofile(path)


def write_figures(directory: Path, views: list[View], fits) -> None:
    corner_panels = []
    for view in views:
        annotated = view.image.copy()
        cv.drawChessboardCorners(annotated, PATTERN, view.corners, True)
        corner_panels.append((f"{view.name}  |  {len(view.corners)} corners", annotated))
    save_panel(directory / "corners.png", corner_panels)

    sample = views[-1]
    # Use one output matrix for all methods to keep the comparison at the same zoom.
    display_matrix = fits[0][1].matrix
    panels = [(f"{sample.name}  |  Original", sample.image)]
    panels.extend(
        (name, cv.undistort(sample.image, fit.matrix, fit.distortion, None, display_matrix))
        for name, fit in fits
    )
    save_panel(directory / "undistortion.png", panels)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path(__file__).with_name("images"),
                        help="Directory containing 11.jpg, 12.jpg and 18.jpg")
    parser.add_argument("--output", type=Path, default=Path("results"), help="Output directory")
    args = parser.parse_args()

    cv.ocl.setUseOpenCL(False)
    cv.setNumThreads(1)
    try:
        views, size = load_views(args.images)
    except (OSError, ValueError, cv.error) as error:
        parser.error(str(error))
    observations = [view.corners for view in views]
    fits = [("Fixed board", calibrate(BOARD, observations, size))]
    metrics = [(name, "fit", squared_errors(fit, observations)) for name, fit in fits]
    for (name, fit), (_, _, squared) in zip(fits, metrics):
        if not np.isclose(fit.rms, rmse(squared), atol=1e-6):
            raise RuntimeError(f"Recomputed RMSE disagrees with OpenCV: {name}")

    args.output.mkdir(parents=True, exist_ok=True)
    write_tables(args.output, [view.name for view in views], fits, metrics)
    write_figures(args.output, views, fits)
    for name, evaluation, squared in metrics:
        print(f"{name} | {evaluation}: {rmse(squared):.6f} px")
    print(f"{len(views)} views, {len(BOARD)} corners per view, {size[0]} x {size[1]} px")
    print(f"Results: {args.output.resolve()}")


if __name__ == "__main__":
    main()
