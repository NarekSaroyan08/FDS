"""Generate the printable 8 x 12 calibration chessboard on A4 paper.

Requires reportlab only when regenerating the PDF, not on the Raspberry Pi.
"""

from pathlib import Path

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


COLS = 8
ROWS = 12
SQUARE_MM = 21
OUTPUT = (
    Path(__file__).resolve().parents[1]
    / "output/pdf/facetracker_chessboard_8x12_21mm_A4.pdf"
)


def main() -> None:
    page_width, page_height = A4
    square = SQUARE_MM * mm
    board_width = COLS * square
    board_height = ROWS * square
    x0 = (page_width - board_width) / 2
    y0 = (page_height - board_height) / 2

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(OUTPUT), pagesize=A4, pageCompression=1)
    pdf.setTitle("FaceTracker calibration chessboard - 8 x 12, 21 mm")
    pdf.setAuthor("FaceTracker")

    pdf.setFont("Helvetica-Bold", 10)
    pdf.drawCentredString(
        page_width / 2,
        page_height - 13 * mm,
        "8 x 12 squares  |  21 mm each  |  Print at 100%",
    )

    pdf.setFillColorRGB(0, 0, 0)
    for row in range(ROWS):
        for col in range(COLS):
            if (row + col) % 2 == 0:
                pdf.rect(
                    x0 + col * square,
                    y0 + (ROWS - 1 - row) * square,
                    square,
                    square,
                    fill=1,
                    stroke=0,
                )

    pdf.setLineWidth(0.15 * mm)
    pdf.rect(x0, y0, board_width, board_height, fill=0, stroke=1)

    scale_length = 100 * mm
    scale_y = 9 * mm
    scale_x = (page_width - scale_length) / 2
    pdf.setLineWidth(0.2 * mm)
    pdf.line(scale_x, scale_y, scale_x + scale_length, scale_y)
    pdf.line(scale_x, scale_y - 1.5 * mm, scale_x, scale_y + 1.5 * mm)
    pdf.line(
        scale_x + scale_length,
        scale_y - 1.5 * mm,
        scale_x + scale_length,
        scale_y + 1.5 * mm,
    )
    pdf.setFont("Helvetica", 8)
    pdf.drawCentredString(page_width / 2, 13 * mm, "Check this line = 100 mm")

    pdf.showPage()
    pdf.save()


if __name__ == "__main__":
    main()
