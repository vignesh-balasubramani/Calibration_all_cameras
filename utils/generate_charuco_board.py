import argparse
import cv2
import numpy as np
from reportlab.lib import pagesizes
from reportlab.pdfgen import canvas
import yaml

PAPER_DIMS = {
    "A4": (pagesizes.A4[1], pagesizes.A4[0]),  # Landscape (w, h) in points (72 pt/inch)
    "A3": (pagesizes.A3[1], pagesizes.A3[0]),
    "Letter": (pagesizes.letter[1], pagesizes.letter[0]),
}

DICTIONARY_MAP = {
    "t36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "t25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_250": cv2.aruco.DICT_6X6_250,
}

def main():
    with open("charuco_config.yaml", "r") as f:
        cfg = yaml.safe_load(f)

    paper_size = cfg["paper"]["size"]
    margin_mm = float(cfg["paper"].get("margin_mm", 5.0))  # Set to 5mm to maximize size
    squares_x = int(cfg["grid"]["squares_x"])
    squares_y = int(cfg["grid"]["squares_y"])
    marker_ratio = float(cfg["grid"].get("marker_ratio", 0.75))
    dict_name = cfg["dictionary"].get("type", "t36h11")

    # Dimensions in mm
    w_mm, h_mm = (297.0, 210.0) if paper_size == "A4" else (420.0, 297.0)
    square_size_mm = min((w_mm - 2 * margin_mm) / squares_x, (h_mm - 2 * margin_mm) / squares_y)
    square_size_m = square_size_mm / 1000.0
    marker_size_m = square_size_m * marker_ratio

    # Render image buffer at 600 DPI for high sub-pixel sharpness
    dpi = 600
    px_per_mm = dpi / 25.4
    board_w_px = int(round(squares_x * square_size_mm * px_per_mm))
    board_h_px = int(round(squares_y * square_size_mm * px_per_mm))

    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY_MAP[dict_name])
    board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_size_m, marker_size_m, dictionary)
    board_img = board.generateImage((board_w_px, board_h_px), marginSize=0, borderBits=1)
    
    cv2.imwrite("/tmp/board_tmp.png", board_img)

    # Place precisely on PDF canvas (1 point = 1/72 inch = 25.4/72 mm)
    pdf_filename = cfg["output"].get("filename", "charuco_target").replace(".png", ".pdf")
    pt_per_mm = 72.0 / 25.4
    c = canvas.Canvas(pdf_filename, pagesize=PAPER_DIMS[paper_size])
    
    # Center the board on the page
    x_offset_pt = ((w_mm - (squares_x * square_size_mm)) / 2.0) * pt_per_mm
    y_offset_pt = ((h_mm - (squares_y * square_size_mm)) / 2.0) * pt_per_mm
    
    c.drawImage(
        "/tmp/board_tmp.png",
        x_offset_pt,
        y_offset_pt,
        width=(squares_x * square_size_mm) * pt_per_mm,
        height=(squares_y * square_size_mm) * pt_per_mm
    )
    c.showPage()
    c.save()

    print("=" * 60)
    print(f"Generated 1:1 Metric PDF: {pdf_filename}")
    print(f"Exact Target Square Size: {square_size_mm:.2f} mm")
    print(f"Exact Target Marker Size: {(marker_size_m * 1000.0):.2f} mm")
    print("=" * 60)

if __name__ == "__main__":
    main()