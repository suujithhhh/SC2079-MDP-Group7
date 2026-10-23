"""Targeted dense-checker and distance augmentation for the next circle revision.

This module only prepares synthetic examples; it never launches training.
Source photographs and annotations are not modified. Split sources by scene
before calling this renderer, and keep new real validation captures separate.
"""
from pathlib import Path

import cv2
import numpy as np

from circle_data import read_source, render


def checker_surface(rng, size=512):
    """Dense neutral checker, including frequencies that alias after downsizing."""
    y, x = np.mgrid[:size, :size].astype(np.float32)
    theta = rng.uniform(-0.35, 0.35)
    u = x * np.cos(theta) + y * np.sin(theta)
    v = -x * np.sin(theta) + y * np.cos(theta)
    cell = float(rng.uniform(2.5, 14.0))
    pattern = (np.floor(u / cell) + np.floor(v / cell)) % 2
    dark, light = rng.uniform(20, 110), rng.uniform(170, 245)
    gray = dark + pattern * (light - dark)
    color = gray[..., None] * rng.uniform(0.88, 1.1, (1, 1, 3))
    return np.clip(color, 0, 255).astype(np.uint8), cell


def augment(source, mask_fn, quad_fn, rng, target_width, blank=False):
    """Render a checker card and change apparent glyph size within a 640 frame.

    Returns image, class, normalized polygon and provenance. Hole recovery is
    used for sixes and letters. Circles are filled from their annotated outline.
    Target width is measured in the final 640-pixel image, before optional 160
    downsampling. A safe placement keeps the entire annotated glyph in frame.
    """
    parsed = read_source(Path(source), mask_fn)
    if parsed is None:
        raise ValueError(f"Unsuitable single-symbol source: {source}")
    image, cls, points, mask = parsed
    if cls == 29:
        mask = np.zeros_like(mask)
        cv2.fillPoly(mask, [points], 255)
    patch, cell = checker_surface(rng)
    rendered = render(image, points, mask, rng, "photo", quad_fn,
                      blank=blank, photo_bank=[patch])
    # render() maps its 256px card texture into the source scene.
    x0, y0 = points.min(0).astype(float)
    x1, y1 = points.max(0).astype(float)
    scale = float(target_width / max(x1 - x0, 1))
    angle = float(rng.uniform(-7, 7))
    center = ((x0 + x1) / 2, (y0 + y1) / 2)
    transform = cv2.getRotationMatrix2D(center, angle, scale)
    relative = cv2.transform(points[None].astype(np.float32), transform)[0]
    low, high = relative.min(0), relative.max(0)
    extent = high - low
    if extent.max() > 540:
        scale *= 540 / float(extent.max())
        transform = cv2.getRotationMatrix2D(center, angle, scale)
        relative = cv2.transform(points[None].astype(np.float32), transform)[0]
        low, high = relative.min(0), relative.max(0)
        extent = high - low
    desired = rng.uniform(45, np.maximum(46, 595 - extent))
    translation = desired - low
    transform[:, 2] += translation
    transformed = cv2.transform(points[None].astype(np.float32), transform)[0]
    # Constant fill avoids mirroring or repeating symbols outside the photograph.
    border = tuple(float(v) for v in np.median(image.reshape(-1, 3), axis=0))
    result = cv2.warpAffine(rendered, transform, (640, 640),
                            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                            borderValue=border)
    if rng.random() < 0.6:
        result = cv2.GaussianBlur(result, (0, 0), float(rng.uniform(0.25, 0.9)))
    # Camera exposure and mild color shifts affect the complete scene.
    result = np.clip(result.astype(np.float32) * rng.uniform(0.65, 1.2)
                     + rng.uniform(-9, 9), 0, 255).astype(np.uint8)
    poly = transformed / 640
    assert np.isfinite(poly).all() and (poly >= 0).all() and (poly <= 1).all()
    return result, (None if blank else cls), poly, {
        "source": str(source), "target_width_640": float(target_width),
        "actual_width_640": float(np.ptp(transformed[:, 0])),
        "checker_cell_512": cell, "affine": transform.tolist(),
        "blank": bool(blank), "synthetic": True,
    }


def write_pair(image, cls, polygon, stem, image_dir, label_dir, rng):
    """Save 640/160 variants with equivalent normalized segmentation labels."""
    image_dir, label_dir = Path(image_dir), Path(label_dir)
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    label = "" if cls is None else str(cls) + " " + " ".join(
        f"{v:.7f}" for v in polygon.ravel()) + "\n"
    paths = []
    for size in (640, 160):
        path = image_dir / f"{stem}_r{size}.jpg"
        small = cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)
        quality = int(rng.integers(78, 96))
        if not cv2.imwrite(str(path), small, [cv2.IMWRITE_JPEG_QUALITY, quality]):
            raise OSError(path)
        (label_dir / f"{path.stem}.txt").write_text(label)
        paths.append({"path": str(path.resolve()), "size": size,
                      "jpeg_quality": quality})
    return paths
