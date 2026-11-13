"""Calibrate one camera from the three PJ1 checkerboard views."""

import argparse
import csv
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import cv2 as cv
import numpy as np
from scipy.linalg import null_space
from scipy.optimize import least_squares


IMAGE_NAMES = ("11.jpg", "12.jpg", "18.jpg")
PATTERN = (9, 9)  # Inner corners (columns, rows).
SQUARE_MM = 50.0
GRID = np.mgrid[:PATTERN[0], :PATTERN[1]].T.reshape(-1, 2)
BOARD = np.column_stack((SQUARE_MM * GRID, np.zeros(len(GRID)))).astype(np.float32)
DETECT_FLAGS = cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_EXHAUSTIVE | cv.CALIB_CB_ACCURACY
FIT_FLAGS = cv.CALIB_FIX_K3 | cv.CALIB_ZERO_TANGENT_DIST
CRITERIA = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_MAX_ITER, 200, 1e-10)
FIXED_CORNER = PATTERN[0] - 1
ANCHORS = (0, FIXED_CORNER, len(BOARD) - 1)


class Method(Enum):
    FIXED = "Fixed board"
    STROBL = "Refined board"
    PLANAR = "Planar constraint"


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
        None, None, flags=FIT_FLAGS, criteria=CRITERIA,
    )
    if refine_board:
        board = adjusted_board.reshape(-1, 3)
    fit = Calibration(rms, matrix, distortion, rotations, translations, board)
    if method is Method.PLANAR:
        return optimize_planar_reference(fit, corners)
    return fit


def deviation_basis(board: np.ndarray) -> np.ndarray:
    """Use the ideal grid to fix the origin, orientation and scale of the board."""
    centered = (board - board.mean(axis=0)) / SQUARE_MM
    translations = np.broadcast_to(np.eye(3)[:, None, :], (3, len(board), 3))
    rotations = np.cross(np.eye(3)[:, None, :], centered[None, :, :])
    scale = centered[None, :, :]
    constraints = np.concatenate((translations, rotations, scale)).reshape(7, -1)
    # Coordinates in the null space satisfy all seven constraints exactly.
    return null_space(constraints)


def optimize_planar_reference(initial: Calibration, observations) -> Calibration:
    """Jointly fit per-corner 3D offsets, lens parameters and the board poses."""
    nominal = initial.board.astype(np.float64)
    basis = deviation_basis(nominal)
    point_basis = basis.reshape(len(nominal), 3, -1)
    views = len(observations)
    pose_end = 6 + 6 * views
    starting_intrinsics = initial.matrix[[0, 1, 0, 1], [0, 1, 2, 2]]
    starting_rotations = np.asarray(initial.rotations).reshape(views, 3)
    starting_translations = np.asarray(initial.translations).reshape(views, 3)
    starting_poses = np.column_stack((starting_rotations, starting_translations))
    start = np.concatenate((starting_intrinsics, initial.distortion.ravel()[:2],
                            starting_poses.ravel(), np.zeros(basis.shape[1])))

    def unpack(values):
        fx, fy, cx, cy, k1, k2 = values[:6]
        matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        distortion = np.array([k1, k2, 0.0, 0.0, 0.0])
        poses = values[6:pose_end].reshape(views, 6)
        offsets = np.einsum("nck,k->nc", point_basis, values[pose_end:])
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
            pose_column = 6 + 6 * index
            result[rows, :6] = jac[:, 6:12]  # fx, fy, cx, cy, k1, k2
            result[rows, pose_column:pose_column + 6] = jac[:, :6]
            # Translation derivatives are wrt camera coordinates. Rotate to board coordinates.
            point_jac = jac[:, 3:6].reshape(-1, 2, 3) @ cv.Rodrigues(rotation)[0]
            result[rows, pose_end:] = (point_jac @ point_basis).reshape(block_rows, -1)
        return result

    solved = least_squares(residuals, start, jac=jacobian, method="lm", x_scale="jac",
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


def held_out_errors(observations, size, method: Method) -> np.ndarray:
    """Fit on three parity groups, then predict omitted corners in the third view."""
    indices = np.arange(len(BOARD))
    parity = GRID[:, 0] % 2 + 2 * (GRID[:, 1] % 2)
    squared = [[] for _ in observations]
    for fold in range(4):
        omitted = (parity == fold) & ~np.isin(indices, ANCHORS)
        fit = calibrate(indices[~omitted], observations, size, method)
        rays = [
            cv.undistortPoints(points[omitted].astype(np.float64), fit.matrix, fit.distortion)
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
    fits = []
    metrics = []
    for method in Method:
        name = method.value
        fit = calibrate(np.arange(len(BOARD)), observations, size, method)
        squared = squared_errors(fit, observations)
        if not np.isclose(fit.rms, rmse(squared), atol=1e-6):
            raise RuntimeError(f"Recomputed RMSE disagrees with the optimizer: {name}")
        fits.append((name, fit))
        metrics.append((name, "fit", squared))
        metrics.append((name, "held_out", held_out_errors(observations, size, method)))

    args.output.mkdir(parents=True, exist_ok=True)
    write_tables(args.output, [view.name for view in views], fits, metrics)
    write_figures(args.output, views, fits)
    for name, evaluation, squared in metrics:
        print(f"{name} | {evaluation}: {rmse(squared):.6f} px")
    print(f"{len(views)} views, {len(BOARD)} corners per view, {size[0]} x {size[1]} px")
    print(f"Results: {args.output.resolve()}")


if __name__ == "__main__":
    main()
