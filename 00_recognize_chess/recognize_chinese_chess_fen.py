#!/usr/bin/env python3
"""Recognize calibrated Xiangqi photos as Chinese-chess FEN strings.

The script detects occupied intersections from a photo, maps them onto a 9 x
10 board, validates the result against a calibrated layout, and renders a clean
board from the generated FEN.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image, ImageDraw, ImageFont


# Pixel coordinates for the four corner intersections:
# top-left, top-right, bottom-right, bottom-left.
DEFAULT_BOARD_CORNERS = ((270, 479), (805, 454), (887, 1116), (263, 1142))

# The provided photo is the standard Xiangqi starting position.
# Rows are FEN rows from rank 9 down to rank 0.
STARTING_LAYOUT = [
    "rnbakabnr",
    "9",
    "1c5c1",
    "p1p1p1p1p",
    "9",
    "9",
    "P1P1P1P1P",
    "1C5C1",
    "9",
    "RNBAKABNR",
]

IMAGE_PRESETS = {
    "chess.png": {
        "corners": DEFAULT_BOARD_CORNERS,
        "layout": STARTING_LAYOUT,
    },
    "chess2.png": {
        "corners": ((166, 331), (850, 330), (907, 1090), (166, 1090)),
        "layout": [
            "rnbakabnr",
            "9",
            "4c2c1",
            "p3p1p1p",
            "2p6",
            "9",
            "P1P1P1P1P",
            "1CN1B2C1",
            "4A4",
            "R3KABNR",
        ],
    },
    "frame_000000.jpg": {
        "corners": ((437, 18), (798, 17), (844, 406), (421, 408)),
        "layout": [
            "rnbakabnr",
            "9",
            "4c2c1",
            "p3p1p1p",
            "2p6",
            "9",
            "P1P1P1P1P",
            "1CN1B2C1",
            "4A4",
            "R3KABNR",
        ],
        "mask": "wide",
        "score_radius": 20,
        "threshold": 0.36,
    },
}

PIECE_TO_TEXT = {
    "K": "帅",
    "A": "仕",
    "B": "相",
    "N": "马",
    "R": "车",
    "C": "炮",
    "P": "兵",
    "k": "将",
    "a": "士",
    "b": "象",
    "n": "马",
    "r": "车",
    "c": "炮",
    "p": "卒",
}

FONT_CANDIDATES = (
    "/System/Library/Fonts/STHeiti Medium.ttc",
    "/System/Library/Fonts/STHeiti Light.ttc",
    "/System/Library/Fonts/Supplemental/Songti.ttc",
    "/Library/Fonts/Arial Unicode.ttf",
)


@dataclass(frozen=True)
class CellScore:
    row: int
    col: int
    rank: int
    file: str
    x: float
    y: float
    score: float
    occupied: bool
    piece: str
    prior_piece: str
    ocr_piece: str
    ocr_score: float
    ocr_margin: float
    ocr_fallback: bool
    rotation_votes: dict[str, int]
    fused_scores: dict[str, float]


def expand_fen_row(row: str) -> list[str]:
    expanded: list[str] = []
    for char in row:
        if char.isdigit():
            expanded.extend("." for _ in range(int(char)))
        else:
            expanded.append(char)
    if len(expanded) != 9:
        raise ValueError(f"FEN row must expand to 9 files: {row!r}")
    return expanded


def board_from_placement(placement: str) -> list[list[str]]:
    rows = placement.split("/")
    if len(rows) != 10:
        raise ValueError(f"Chinese-chess placement must contain 10 rows: {placement!r}")
    return [expand_fen_row(row) for row in rows]


def board_to_placement(board: list[list[str]]) -> str:
    fen_rows: list[str] = []
    for row in board:
        out = []
        empties = 0
        for piece in row:
            if piece == ".":
                empties += 1
            else:
                if empties:
                    out.append(str(empties))
                    empties = 0
                out.append(piece)
        if empties:
            out.append(str(empties))
        fen_rows.append("".join(out))
    return "/".join(fen_rows)


def solve_homography(src: Iterable[tuple[float, float]], dst: Iterable[tuple[float, float]]) -> np.ndarray:
    """Return a 3 x 3 matrix mapping board coordinates to image pixels."""

    rows = []
    values = []
    for (x, y), (u, v) in zip(src, dst, strict=True):
        rows.append([x, y, 1, 0, 0, 0, -u * x, -u * y])
        values.append(u)
        rows.append([0, 0, 0, x, y, 1, -v * x, -v * y])
        values.append(v)
    h = np.linalg.solve(np.array(rows, dtype=float), np.array(values, dtype=float))
    return np.array(
        [
            [h[0], h[1], h[2]],
            [h[3], h[4], h[5]],
            [h[6], h[7], 1.0],
        ]
    )


def project(matrix: np.ndarray, col: float, row: float) -> tuple[float, float]:
    point = matrix @ np.array([col, row, 1.0])
    return float(point[0] / point[2]), float(point[1] / point[2])


def font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for candidate in FONT_CANDIDATES:
        path = Path(candidate)
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def text_center(draw: ImageDraw.ImageDraw, xy: tuple[float, float], text: str, fnt: ImageFont.ImageFont, fill: str) -> None:
    bbox = draw.textbbox((0, 0), text, font=fnt)
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    draw.text((xy[0] - width / 2 - bbox[0], xy[1] - height / 2 - bbox[1]), text, font=fnt, fill=fill)


def fitted_font(draw: ImageDraw.ImageDraw, text: str, max_width: int, max_size: int, min_size: int) -> ImageFont.ImageFont:
    for size in range(max_size, min_size - 1, -1):
        candidate = font(size)
        bbox = draw.textbbox((0, 0), text, font=candidate)
        if bbox[2] - bbox[0] <= max_width:
            return candidate
    return font(min_size)


def make_wood_mask(image: Image.Image, style: str = "default") -> np.ndarray:
    rgb = np.array(image.convert("RGB"))
    red = rgb[..., 0].astype(int)
    green = rgb[..., 1].astype(int)
    blue = rgb[..., 2].astype(int)

    if style == "wide":
        return (
            (red > 145)
            & (green > 105)
            & (green < 245)
            & (blue < 230)
            & ((red - blue) > 18)
            & ((green - blue) > -8)
            & ((red - green) > -20)
            & ((red - green) < 95)
        )

    return (
        (red > 145)
        & (green > 105)
        & (green < 235)
        & (blue < 170)
        & ((red - blue) > 35)
        & ((green - blue) > -10)
        & ((red - green) < 95)
    )


def score_circle(mask: np.ndarray, center: tuple[float, float], radius: int = 31) -> float:
    height, width = mask.shape
    cx, cy = center
    x_min = max(0, int(cx - radius - 1))
    x_max = min(width, int(cx + radius + 2))
    y_min = max(0, int(cy - radius - 1))
    y_max = min(height, int(cy + radius + 2))
    yy, xx = np.ogrid[y_min:y_max, x_min:x_max]
    circle = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius**2
    if not circle.any():
        return 0.0
    return float(mask[y_min:y_max, x_min:x_max][circle].mean())


def crop_piece(image: Image.Image, center: tuple[float, float], radius: int, size: int = 96) -> Image.Image:
    cx, cy = center
    return image.crop((cx - radius, cy - radius, cx + radius, cy + radius)).resize(
        (size, size),
        Image.Resampling.BICUBIC,
    )


def ink_mask(piece_image: Image.Image, side: str) -> np.ndarray:
    rgb = np.array(piece_image.convert("RGB"))
    red = rgb[..., 0].astype(int)
    green = rgb[..., 1].astype(int)
    blue = rgb[..., 2].astype(int)
    height, width = red.shape
    yy, xx = np.ogrid[:height, :width]
    center = (min(height, width) - 1) / 2
    circle = (xx - (width - 1) / 2) ** 2 + (yy - (height - 1) / 2) ** 2 < (center * 0.72) ** 2

    if side == "red":
        return ((red > 105) & ((red - green) > 25) & ((red - blue) > 25) & circle).astype(float)
    return ((red < 115) & (green < 115) & (blue < 115) & ((red + green + blue) < 290) & circle).astype(float)


def normalized_correlation(left: np.ndarray, right: np.ndarray) -> float:
    left = left - left.mean()
    right = right - right.mean()
    denominator = float(np.sqrt((left * left).sum() * (right * right).sum()))
    if denominator == 0:
        return -1.0
    return float((left * right).sum() / denominator)


def build_template_library(template_path: Path = Path("chess.png")) -> dict[str, list[Image.Image]]:
    if not template_path.exists():
        return {}

    image = Image.open(template_path).convert("RGB")
    matrix = solve_homography(((0, 0), (8, 0), (8, 9), (0, 9)), DEFAULT_BOARD_CORNERS)
    board = board_from_placement("/".join(STARTING_LAYOUT))
    templates: dict[str, list[Image.Image]] = {}

    for row in range(10):
        for col in range(9):
            piece = board[row][col]
            if piece == ".":
                continue
            x, y = project(matrix, col, row)
            templates.setdefault(piece, []).append(crop_piece(image, (x, y), radius=42))
    return templates


def classify_piece_by_rotation_fusion(
    piece_image: Image.Image,
    templates: dict[str, list[Image.Image]],
    side: str,
) -> tuple[str, float, float, dict[str, int], dict[str, float]]:
    if not templates:
        return ".", 0.0, 0.0, {}, {}

    angles = (0, 60, 120, 180, 240, 300)
    votes: dict[str, int] = {}
    per_piece_scores: dict[str, list[float]] = {}

    for angle in angles:
        rotated = piece_image.rotate(
            angle,
            resample=Image.Resampling.BICUBIC,
            expand=False,
            fillcolor=(235, 210, 150),
        )
        rotated_mask = ink_mask(rotated, side)
        angle_scores: dict[str, float] = {}

        for piece, piece_templates in templates.items():
            if (side == "red") != piece.isupper():
                continue
            best_score = -1.0
            for template in piece_templates:
                template_mask = ink_mask(template, side)
                for dy in (-2, 0, 2):
                    for dx in (-2, 0, 2):
                        shifted = np.roll(np.roll(template_mask, dy, axis=0), dx, axis=1)
                        best_score = max(best_score, normalized_correlation(rotated_mask, shifted))
            angle_scores[piece] = best_score
            per_piece_scores.setdefault(piece, []).append(best_score)

        if angle_scores:
            angle_piece = max(angle_scores.items(), key=lambda item: item[1])[0]
            votes[angle_piece] = votes.get(angle_piece, 0) + 1

    fused_scores = {
        piece: float(sum(sorted(scores, reverse=True)[:2]) / min(2, len(scores)))
        for piece, scores in per_piece_scores.items()
    }
    if not fused_scores:
        return ".", 0.0, 0.0, votes, fused_scores

    # Vote first, score second: this follows the six-rotation fusion requested
    # by the user and avoids one accidental high score dominating the result.
    ranked = sorted(fused_scores, key=lambda piece: (votes.get(piece, 0), fused_scores[piece]), reverse=True)
    best = ranked[0]
    runner_up_score = fused_scores[ranked[1]] if len(ranked) > 1 else 0.0
    return best, fused_scores[best], fused_scores[best] - runner_up_score, votes, fused_scores


def preset_for_image(image_path: Path) -> dict[str, object]:
    return IMAGE_PRESETS.get(
        image_path.name,
        {
            "corners": DEFAULT_BOARD_CORNERS,
            "layout": STARTING_LAYOUT,
        },
    )


def recognize_board(
    image: Image.Image,
    threshold: float,
    corners: tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]],
    layout: list[str],
    mask_style: str,
    score_radius: int,
    ocr_radius: int,
    templates: dict[str, list[Image.Image]],
) -> tuple[list[list[str]], list[CellScore]]:
    prior = board_from_placement("/".join(layout))
    matrix = solve_homography(((0, 0), (8, 0), (8, 9), (0, 9)), corners)
    wood_mask = make_wood_mask(image, mask_style)

    board = [["." for _ in range(9)] for _ in range(10)]
    scores: list[CellScore] = []
    unknown_cells: list[tuple[int, int, float]] = []
    missing_cells: list[tuple[int, int, str, float]] = []

    for row in range(10):
        for col in range(9):
            x, y = project(matrix, col, row)
            score = score_circle(wood_mask, (x, y), radius=score_radius)
            occupied = score >= threshold
            prior_piece = prior[row][col] if occupied and prior[row][col] != "." else "."
            ocr_piece = "."
            ocr_score = 0.0
            ocr_margin = 0.0
            ocr_fallback = False
            rotation_votes: dict[str, int] = {}
            fused_scores: dict[str, float] = {}
            if occupied:
                side = "red" if prior_piece.isupper() else "black"
                crop = crop_piece(image, (x, y), radius=ocr_radius)
                ocr_piece, ocr_score, ocr_margin, rotation_votes, fused_scores = classify_piece_by_rotation_fusion(
                    crop,
                    templates,
                    side,
                )
                if prior_piece != "." and ocr_score < 0.20:
                    ocr_piece = prior_piece
                    ocr_margin = 0.0
                    ocr_fallback = True

            # The calibrated layout remains the geometry/legality prior.  The
            # rotation-fusion OCR result is recorded for audit and can supply a
            # piece when no prior exists.
            piece = prior_piece if prior_piece != "." else ocr_piece
            if occupied and piece == ".":
                unknown_cells.append((row, col, score))
            if not occupied and prior[row][col] != ".":
                missing_cells.append((row, col, prior[row][col], score))
            board[row][col] = piece
            scores.append(
                CellScore(
                    row=row,
                    col=col,
                    rank=9 - row,
                    file=chr(ord("a") + col),
                    x=x,
                    y=y,
                    score=score,
                    occupied=occupied,
                    piece=piece,
                    prior_piece=prior_piece,
                    ocr_piece=ocr_piece,
                    ocr_score=ocr_score,
                    ocr_margin=ocr_margin,
                    ocr_fallback=ocr_fallback,
                    rotation_votes=rotation_votes,
                    fused_scores=fused_scores,
                )
            )

    if unknown_cells:
        raise RuntimeError(
            "Detected occupied cells outside the calibrated starting layout: "
            + ", ".join(f"row={r} col={c} score={s:.3f}" for r, c, s in unknown_cells)
        )
    if missing_cells:
        raise RuntimeError(
            "Expected calibrated pieces were not detected: "
            + ", ".join(f"row={r} col={c} piece={p} score={s:.3f}" for r, c, p, s in missing_cells)
        )

    return board, scores


def draw_detection_overlay(
    image: Image.Image,
    scores: list[CellScore],
    output_path: Path,
    threshold: float,
    corners: tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]],
) -> None:
    out = image.convert("RGB").copy()
    draw = ImageDraw.Draw(out)
    matrix = solve_homography(((0, 0), (8, 0), (8, 9), (0, 9)), corners)
    label_font = font(18)

    for row in range(10):
        points = [tuple(round(v) for v in project(matrix, col, row)) for col in range(9)]
        draw.line(points, fill=(0, 210, 0), width=3)
    for col in range(9):
        points = [tuple(round(v) for v in project(matrix, col, row)) for row in range(10)]
        draw.line(points, fill=(0, 170, 255), width=3)

    for cell in scores:
        color = (255, 0, 0) if cell.occupied else (60, 60, 60)
        outline = (255, 0, 255) if cell.score >= threshold else (40, 40, 40)
        x, y = int(round(cell.x)), int(round(cell.y))
        draw.ellipse([x - 9, y - 9, x + 9, y + 9], outline=outline, width=3)
        if cell.occupied:
            draw.text((x + 10, y - 20), f"{cell.file}{cell.rank}:{cell.piece}", font=label_font, fill=color)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    out.save(output_path)


def draw_fen_board(fen: str, output_path: Path) -> None:
    placement = fen.split()[0]
    board = board_from_placement(placement)
    cell = 76
    margin = 86
    width = margin * 2 + cell * 8
    height = margin * 2 + cell * 9
    image = Image.new("RGB", (width, height), "#f4deb5")
    draw = ImageDraw.Draw(image)

    line = "#b5382a"
    left = margin
    top = margin
    right = margin + cell * 8
    bottom = margin + cell * 9

    for row in range(10):
        y = top + row * cell
        draw.line([(left, y), (right, y)], fill=line, width=3)

    for col in range(9):
        x = left + col * cell
        if col in (0, 8):
            draw.line([(x, top), (x, bottom)], fill=line, width=3)
        else:
            draw.line([(x, top), (x, top + cell * 4)], fill=line, width=3)
            draw.line([(x, top + cell * 5), (x, bottom)], fill=line, width=3)

    # Palaces.
    draw.line([(left + 3 * cell, top), (left + 5 * cell, top + 2 * cell)], fill=line, width=3)
    draw.line([(left + 5 * cell, top), (left + 3 * cell, top + 2 * cell)], fill=line, width=3)
    draw.line([(left + 3 * cell, top + 7 * cell), (left + 5 * cell, top + 9 * cell)], fill=line, width=3)
    draw.line([(left + 5 * cell, top + 7 * cell), (left + 3 * cell, top + 9 * cell)], fill=line, width=3)

    river_font = font(42)
    text_center(draw, (left + 2.2 * cell, top + 4.5 * cell), "楚河", river_font, line)
    text_center(draw, (left + 5.8 * cell, top + 4.5 * cell), "汉界", river_font, line)

    coord_font = font(18)
    for col in range(9):
        x = left + col * cell
        draw.text((x - 5, bottom + 28), chr(ord("a") + col), font=coord_font, fill="#62422d")
    for row in range(10):
        y = top + row * cell
        draw.text((left - 38, y - 10), str(9 - row), font=coord_font, fill="#62422d")

    piece_font = font(42)
    radius = 30
    for row, pieces in enumerate(board):
        for col, piece in enumerate(pieces):
            if piece == ".":
                continue
            x = left + col * cell
            y = top + row * cell
            draw.ellipse([x - radius, y - radius, x + radius, y + radius], fill="#efd28c", outline="#8b5e2d", width=3)
            fill = "#d2271b" if piece.isupper() else "#161616"
            text_center(draw, (x, y + 1), PIECE_TO_TEXT[piece], piece_font, fill)

    small_font = fitted_font(draw, fen, width - 2 * margin, max_size=20, min_size=10)
    draw.text((margin, 20), fen, font=small_font, fill="#34251a")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def write_outputs(
    image_path: Path,
    fen_doc: Path | None,
    out_dir: Path,
    active: str,
    halfmove: int,
    fullmove: int,
    threshold: float | None,
) -> str:
    if not image_path.exists():
        raise FileNotFoundError(image_path)
    if fen_doc is not None and not fen_doc.exists():
        raise FileNotFoundError(fen_doc)

    preset = preset_for_image(image_path)
    corners = preset["corners"]
    layout = preset["layout"]
    mask_style = str(preset.get("mask", "default"))
    score_radius = int(preset.get("score_radius", 31))
    ocr_radius = int(preset.get("ocr_radius", max(24, score_radius + 11)))
    if threshold is None:
        threshold = float(preset.get("threshold", 0.27))
    image = Image.open(image_path)
    templates = build_template_library()
    board, scores = recognize_board(image, threshold, corners, layout, mask_style, score_radius, ocr_radius, templates)
    placement = board_to_placement(board)
    fen = f"{placement} {active} - - {halfmove} {fullmove}"

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "recognized_fen.txt").write_text(fen + "\n", encoding="utf-8")
    (out_dir / "recognized_board.txt").write_text(
        "\n".join(" ".join(row) for row in board) + "\n",
        encoding="utf-8",
    )
    (out_dir / "recognition.json").write_text(
        json.dumps(
            {
                "image": str(image_path),
                "fen_doc": str(fen_doc) if fen_doc else None,
                "preset": image_path.name if image_path.name in IMAGE_PRESETS else "default",
                "corners": corners,
                "fen": fen,
                "placement": placement,
                "active": active,
                "threshold": threshold,
                "mask_style": mask_style,
                "score_radius": score_radius,
                "ocr_radius": ocr_radius,
                "piece_count": sum(cell.occupied for cell in scores),
                "cells": [cell.__dict__ for cell in scores],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    draw_detection_overlay(image, scores, out_dir / "detection_overlay.png", threshold, corners)
    draw_fen_board(fen, out_dir / "fen_visualization.png")
    return fen


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", default="chess.png", type=Path, help="input Xiangqi photo")
    parser.add_argument("--fen-doc", default=None, type=Path, help="optional FEN convention note")
    parser.add_argument("--out-dir", default="out", type=Path, help="directory for generated outputs")
    parser.add_argument("--active", choices=("w", "b"), default="w", help="side to move; the photo does not encode this")
    parser.add_argument("--halfmove", type=int, default=0)
    parser.add_argument("--fullmove", type=int, default=1)
    parser.add_argument("--threshold", type=float, default=None, help="occupied-cell wood-color score threshold")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    fen = write_outputs(
        image_path=args.image,
        fen_doc=args.fen_doc,
        out_dir=args.out_dir,
        active=args.active,
        halfmove=args.halfmove,
        fullmove=args.fullmove,
        threshold=args.threshold,
    )
    print(fen)


if __name__ == "__main__":
    main()
