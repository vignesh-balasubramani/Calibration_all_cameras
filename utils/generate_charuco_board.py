#!/usr/bin/env python3
"""Generate a print-ready, 1:1 scale ChArUco calibration target as a PDF.

The board is scaled to be as large as the page allows (inside the margin), centred,
and rendered so that printing at "Actual size / 100 %" gives exact metric dimensions.

After printing, MEASURE a square and a marker with calipers and copy the values into
`src/calib_config.py` (SQUARE_LENGTH_M / MARKER_LENGTH_M) - printers rarely hit the
nominal size exactly.

Usage:
    python generate_charuco_board.py                       # uses charuco_config.yaml
    python generate_charuco_board.py --config my.yaml --output board.pdf
"""
import argparse
import io
from pathlib import Path

import cv2
import yaml
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas

HERE = Path(__file__).resolve().parent
MM_PER_INCH = 25.4
PT_PER_MM = 72.0 / MM_PER_INCH

# (short side, long side) in millimetres
PAPER_MM = {
    "A4": (210.0, 297.0),
    "A3": (297.0, 420.0),
    "Letter": (215.9, 279.4),
}

DICTIONARIES = {
    "t36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "t25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "t16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_250": cv2.aruco.DICT_6X6_250,
}


def page_size_mm(size: str, orientation: str):
    """(width, height) of the page in millimetres."""
    if size not in PAPER_MM:
        raise SystemExit(f"Unsupported paper size {size!r}; choose from {sorted(PAPER_MM)}")
    short, long_ = PAPER_MM[size]
    if orientation == "landscape":
        return long_, short
    if orientation == "portrait":
        return short, long_
    raise SystemExit(f"orientation must be 'landscape' or 'portrait', got {orientation!r}")


def generate(config_path: Path, output_path: Path = None) -> Path:
    cfg = yaml.safe_load(config_path.read_text())

    paper, grid = cfg["paper"], cfg["grid"]
    page_w_mm, page_h_mm = page_size_mm(paper["size"], paper.get("orientation", "landscape"))
    margin_mm = float(paper.get("margin_mm", 10.0))
    dpi = int(paper.get("dpi", 600))
    squares_x, squares_y = int(grid["squares_x"]), int(grid["squares_y"])
    marker_ratio = float(grid.get("marker_ratio", 0.75))

    dict_name = cfg["dictionary"].get("type", "t36h11")
    if dict_name not in DICTIONARIES:
        raise SystemExit(f"Unknown dictionary {dict_name!r}; choose from {sorted(DICTIONARIES)}")

    # Largest square that fits the printable area in both directions.
    square_mm = min((page_w_mm - 2 * margin_mm) / squares_x,
                    (page_h_mm - 2 * margin_mm) / squares_y)
    marker_mm = square_mm * marker_ratio
    board_w_mm, board_h_mm = squares_x * square_mm, squares_y * square_mm

    px_per_mm = dpi / MM_PER_INCH
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y), square_mm / 1000.0, marker_mm / 1000.0,
        cv2.aruco.getPredefinedDictionary(DICTIONARIES[dict_name]),
    )
    board_img = board.generateImage(
        (round(board_w_mm * px_per_mm), round(board_h_mm * px_per_mm)),
        marginSize=0, borderBits=1,
    )
    ok, png = cv2.imencode(".png", board_img)
    if not ok:
        raise RuntimeError("Failed to encode board image")

    if output_path is None:
        output_path = HERE / Path(cfg["output"].get("filename", "charuco_target.pdf")).with_suffix(".pdf")

    pdf = canvas.Canvas(str(output_path), pagesize=(page_w_mm * PT_PER_MM, page_h_mm * PT_PER_MM))
    pdf.drawImage(
        ImageReader(io.BytesIO(png.tobytes())),
        (page_w_mm - board_w_mm) / 2 * PT_PER_MM,
        (page_h_mm - board_h_mm) / 2 * PT_PER_MM,
        width=board_w_mm * PT_PER_MM,
        height=board_h_mm * PT_PER_MM,
    )
    pdf.showPage()
    pdf.save()

    print("=" * 60)
    print(f"Wrote 1:1 metric PDF: {output_path}")
    print(f"Board: {squares_x} x {squares_y} squares, dictionary {dict_name}")
    print(f"Nominal square: {square_mm:.2f} mm | marker: {marker_mm:.2f} mm")
    print("Print at 100 % / 'Actual size', measure, then set in src/calib_config.py:")
    print(f"    SQUARE_LENGTH_M = {square_mm / 1000:.4f}")
    print(f"    MARKER_LENGTH_M = {marker_mm / 1000:.4f}")
    print("=" * 60)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--config", type=Path, default=HERE / "charuco_config.yaml",
                        help="board/paper settings (default: charuco_config.yaml next to this script)")
    parser.add_argument("--output", type=Path, help="PDF path (default: from the config's output.filename)")
    args = parser.parse_args()
    generate(args.config, args.output)


if __name__ == "__main__":
    main()
