"""Calibrate one camera from the three PJ1 checkerboard views."""

import argparse
import csv
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from statistics import mean, stdev

import cv2 as cv
import numpy as np
from scipy.linalg import helmert
from scipy.optimize import least_squares


IMAGE_NAMES = ("11.jpg", "12.jpg", "18.jpg")
PATTERN = (9, 9)  # Inner corners (columns, rows).
SQUARE_MM = 50.0
GRID = np.mgrid[:PATTERN[0], :PATTERN[1]].T.reshape(-1, 2)
BOARD = np.column_stack((SQUARE_MM * GRID, np.zeros(len(GRID)))).astype(np.float32)
DETECT_FLAGS = cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_EXHAUSTIVE | cv.CALIB_CB_ACCURACY
CRITERIA = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_MAX_ITER, 200, 1e-10)
FIXED_CORNER = PATTERN[0] - 1
ANCHORS = (0, FIXED_CORNER, len(BOARD) - 1)


class Method(Enum):
    FIXED = "Fixed board"
    STROBL = "Refined board"
    HUANG = "Huang 2024"


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


def calibrate(indices, observations, size, method: Method) -> Calibration:
    board = BOARD[indices]
    corners = [points[indices] for points in observations]
    refine_board = method is Method.STROBL
    fixed = int(np.searchsorted(indices, FIXED_CORNER)) if refine_board else -1
    rms, matrix, distortion, rotations, translations, adjusted_board = cv.calibrateCameraRO(
        [board] * len(corners), corners, size, fixed,
        None, None, criteria=CRITERIA,
    )
    if refine_board:
        board = adjusted_board.reshape(-1, 3)
    fit = Calibration(rms, matrix, distortion, rotations, translations, board)
    if method is Method.HUANG:
        return optimize_huang(fit, corners)
    return fit


def optimize_huang(initial: Calibration, observations) -> Calibration:
    """Solve Huang et al. (2024), Eq. (8), with the equalities in Eq. (7)."""
    nominal = initial.board.astype(np.float64)
    # Helmert columns span exactly the zero-sum offsets in each of X, Y and Z.
    basis = helmert(len(nominal)).T
    views = len(observations)
    lens_end = 9
    pose_end = lens_end + 6 * views
    starting_intrinsics = initial.matrix[[0, 1, 0, 1], [0, 1, 2, 2]]
    starting_rotations = np.asarray(initial.rotations).reshape(views, 3)
    starting_translations = np.asarray(initial.translations).reshape(views, 3)
    starting_poses = np.column_stack((starting_rotations, starting_translations))
    start = np.concatenate((starting_intrinsics, initial.distortion.ravel(),
                            starting_poses.ravel(), np.zeros(3 * basis.shape[1])))

    def unpack(values):
        fx, fy, cx, cy = values[:4]
        matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        distortion = values[4:lens_end]
        poses = values[lens_end:pose_end].reshape(views, 6)
        offsets = np.einsum("nk,kc->nc", basis, values[pose_end:].reshape(-1, 3))
        board = nominal + offsets
        return matrix, distortion, poses[:, :3], poses[:, 3:], board

    def residuals(values):
        matrix, distortion, rotations, translations, board = unpack(values)
        return np.concatenate([
            (cv.projectPoints(board, rotation, translation, matrix, distortion)[0] - pixels).ravel()
            for rotation, translation, pixels in zip(rotations, translations, observations)
        ])

    def jacobian(values):
        matrix, distortion, rotations, translations, board = unpack(values)
        block_rows = 2 * len(board)
        result = np.zeros((views * block_rows, values.size))
        for index, (rotation, translation) in enumerate(zip(rotations, translations)):
            _, jac = cv.projectPoints(board, rotation, translation, matrix, distortion)
            rows = slice(index * block_rows, (index + 1) * block_rows)
            pose_column = lens_end + 6 * index
            result[rows, :lens_end] = jac[:, 6:15]  # fx, fy, cx, cy, k1, k2, p1, p2, k3
            result[rows, pose_column:pose_column + 6] = jac[:, :6]
            # Translation derivatives are wrt camera coordinates. Rotate to board coordinates.
            point_jac = jac[:, 3:6].reshape(-1, 2, 3) @ cv.Rodrigues(rotation)[0]
            offset_jac = np.einsum("nac,nk->nakc", point_jac, basis)
            result[rows, pose_end:] = offset_jac.reshape(block_rows, -1)
        return result

    # Eq. (7) leaves global rotation and scale ambiguity. The exact trust-region
    # solver uses SVD so the fit can proceed with a rank-deficient Jacobian.
    solved = least_squares(residuals, start, jac=jacobian, method="trf", tr_solver="exact", x_scale="jac",
                           ftol=1e-10, xtol=1e-10, gtol=1e-10, max_nfev=200)
    if not solved.success:
        raise RuntimeError(f"Planar constraint optimization failed: {solved.message}")
    matrix, distortion, rotations, translations, board = unpack(solved.x)
    rms = rmse(np.sum(solved.fun.reshape(-1, 2) ** 2, axis=1))
    return Calibration(rms, matrix, distortion, tuple(rotations), tuple(translations), board)


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


def corner_folds() -> list[np.ndarray]:
    indices = np.arange(len(BOARD))
    parity = GRID[:, 0] % 2 + 2 * (GRID[:, 1] % 2)
    return [indices[(parity == fold) & ~np.isin(indices, ANCHORS)] for fold in range(4)]


def held_out_errors(observations, size, method: Method, folds) -> np.ndarray:
    """Fit on three groups, then predict omitted corners in the third view."""
    indices = np.arange(len(BOARD))
    squared = [[] for _ in observations]
    for omitted in folds:
        fit = calibrate(np.setdiff1d(indices, omitted), observations, size, method)
        rays = [
            cv.undistortPointsIter(
                points[omitted].astype(np.float64), fit.matrix, fit.distortion, None, None, CRITERIA,
            )
            .reshape(-1, 2).T
            for points in observations
        ]
        poses = [
            np.column_stack((cv.Rodrigues(rotation)[0], translation))
            for rotation, translation in zip(fit.rotations, fit.translations)
        ]
        for index, points in enumerate(observations):
            first, second = [other for other in range(len(observations)) if other != index]
            homogeneous = cv.triangulatePoints(
                poses[first], poses[second], rays[first], rays[second],
            ).T
            reconstructed = homogeneous[:, :3] / homogeneous[:, 3:]
            residual = pose_projection(fit, reconstructed, index) - points[omitted]
            squared[index].extend(np.sum(residual ** 2, axis=(1, 2)))
    return np.asarray(squared)


def rmse(squared: np.ndarray) -> float:
    return float(np.sqrt(np.mean(squared)))


def fit_all(observations, size, method: Method) -> tuple[Calibration, np.ndarray]:
    fit = calibrate(np.arange(len(BOARD)), observations, size, method)
    squared = squared_errors(fit, observations)
    if not np.isclose(fit.rms, rmse(squared), rtol=0, atol=1e-6):
        raise RuntimeError(f"Recomputed RMSE disagrees with the optimizer: {method.value}")
    return fit, squared


def measure_repeats(observations, size, fixed_metrics, fixed_count, shuffled_count):
    folds = corner_folds()
    runs = [("fixed", 1, "", *metric) for metric in fixed_metrics]
    for run in range(2, fixed_count + 1):
        for method in Method:
            _, squared = fit_all(observations, size, method)
            runs.append(("fixed", run, "", method.value, "fit", squared))
            held_out = held_out_errors(observations, size, method, folds)
            runs.append(("fixed", run, "", method.value, "held_out", held_out))

    # Preserve the previous fold sizes and the three shared training anchors.
    eligible = np.sort(np.concatenate(folds))
    split_at = np.cumsum([len(fold) for fold in folds[:-1]])
    for seed in range(shuffled_count):
        shuffled = np.random.default_rng(seed).permutation(eligible)
        shuffled_folds = np.split(shuffled, split_at)
        for method in Method:
            squared = held_out_errors(observations, size, method, shuffled_folds)
            runs.append(("shuffled", seed + 1, seed, method.value, "held_out", squared))
    return runs


def write_csv(path: Path, headers, rows) -> None:
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(headers)
        writer.writerows(rows)


def write_tables(directory: Path, names, fits, metrics) -> None:
    parameters = [
        (name, fit.matrix[0, 0], fit.matrix[1, 1], fit.matrix[0, 2], fit.matrix[1, 2],
         *fit.distortion.ravel())
        for name, fit in fits
    ]
    write_csv(
        directory / "parameters.csv",
        ("method", "fx_px", "fy_px", "cx_px", "cy_px", "k1", "k2", "p1", "p2", "k3"),
        parameters,
    )
    errors = []
    for name, evaluation, squared in metrics:
        errors.extend((name, evaluation, *row) for row in error_rows(names, squared))
    write_csv(
        directory / "reprojection_errors.csv",
        ("method", "evaluation", "image", "observations", "rmse_px"),
        errors,
    )


def error_rows(names, squared):
    rows = [(view, len(values), rmse(values)) for view, values in zip(names, squared)]
    return [*rows, ("All", squared.size, rmse(squared))]


def write_repeats(directory: Path, names, runs) -> None:
    grouped = {}
    rows = []
    for protocol, run, seed, method, evaluation, squared in runs:
        errors = error_rows(names, squared)
        rows.extend((protocol, run, seed, method, evaluation, *row) for row in errors)
        grouped.setdefault((protocol, method, evaluation), []).append(errors[-1][-1])
    write_csv(
        directory / "repeat_runs.csv",
        ("protocol", "run", "seed", "method", "evaluation", "image", "observations", "rmse_px"),
        rows,
    )
    summary = []
    for key, values in grouped.items():
        sd = stdev(values) if len(values) > 1 else ""
        summary.append((*key, len(values), mean(values), sd, min(values), max(values)))
    write_csv(
        directory / "repeat_summary.csv",
        ("protocol", "method", "evaluation", "runs", "mean_rmse_px", "sd_rmse_px",
         "min_rmse_px", "max_rmse_px"),
        summary,
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
    parser.add_argument("--fixed-repeats", type=int, default=5,
                        help="Number of runs with the same inputs and folds (default: 5)")
    parser.add_argument("--shuffled-repeats", type=int, default=20,
                        help="Number of randomized four-fold evaluations (default: 20)")
    args = parser.parse_args()
    if args.fixed_repeats < 1 or args.shuffled_repeats < 0:
        parser.error("--fixed-repeats must be positive and --shuffled-repeats nonnegative")

    cv.ocl.setUseOpenCL(False)
    cv.setNumThreads(1)
    try:
        views, size = load_views(args.images)
    except (OSError, ValueError, cv.error) as error:
        parser.error(str(error))
    observations = [view.corners for view in views]
    fits = []
    metrics = []
    for method in Method:
        name = method.value
        fit, squared = fit_all(observations, size, method)
        fits.append((name, fit))
        metrics.append((name, "fit", squared))
        metrics.append((name, "held_out", held_out_errors(observations, size, method, corner_folds())))

    runs = measure_repeats(observations, size, metrics, args.fixed_repeats, args.shuffled_repeats)
    names = [view.name for view in views]
    args.output.mkdir(parents=True, exist_ok=True)
    write_tables(args.output, names, fits, metrics)
    write_repeats(args.output, names, runs)
    write_figures(args.output, views, fits)
    for name, evaluation, squared in metrics:
        print(f"{name} | {evaluation}: {rmse(squared):.6f} px")
    print(f"{len(views)} views, {len(BOARD)} corners per view, {size[0]} x {size[1]} px")
    print(f"Results: {args.output.resolve()}")


if __name__ == "__main__":
    main()
