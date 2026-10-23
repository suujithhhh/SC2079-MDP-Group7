"""Black arrows on white paper, with explicit left/right labels.

Only NumPy and OpenCV are required. All randomness comes from a supplied
numpy.random.Generator; the functions do not read or write source datasets.

restyle(image, record, rng) retains the source scene and corrected glyph
geometry. record has old_class (27=right, 28=left), normalized polygon and box.
canonical_scene(background, rng, class_id, size, holdout=False,
                negative_kind=None) draws a card on a 640-square scene.
size is the maximum glyph dimension before projective deformation, in pixels.
Both return (BGR uint8 image, [{"class": int, "box": [x0,y0,x1,y1]}]).

The caller must retain source-group split assignments for every derivative.
Restyled photographs remain synthetic examples, not new real photographs.
"""

import cv2
import numpy as np


def _tight(mask):
    ys, xs = np.where(mask > 127)
    if not len(xs):
        raise ValueError("Rendered glyph disappeared at the requested size")
    return [float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1)]


def _polygon_mask(shape, polygon, supersample=3):
    h, w = shape
    canvas = np.zeros((h * supersample, w * supersample), np.uint8)
    cv2.fillPoly(canvas, [np.round(np.asarray(polygon) * supersample).astype(np.int32)], 255)
    return cv2.resize(canvas, (w, h), interpolation=cv2.INTER_AREA)


def _blend(image, replacement, mask):
    alpha = mask.astype(np.float32)[..., None] / 255.0
    return np.clip(image.astype(np.float32) * (1 - alpha) + replacement * alpha, 0, 255).astype(np.uint8)


def restyle(image, record, rng):
    """Replace the old dark arrow card with white paper and black ink.

    A padded axis-aligned rectangle enclosing the complete corrected glyph forms
    the paper. This preserves the photographed arrow's projection exactly;
    it does not rectify, normalize its aspect ratio, or add another warp.
    Source low-frequency illumination and a small paper gradient are retained.
    """
    old_class = int(record['old_class'])
    if old_class not in (27, 28):
        raise ValueError("restyle accepts only legacy left/right arrows")
    image = np.asarray(image, dtype=np.uint8)
    h, w = image.shape[:2]
    polygon = np.asarray(record['polygon'], np.float32) * [w, h]
    if len(polygon) < 3 or not np.isfinite(polygon).all():
        raise ValueError("A valid full-arrow polygon is required")
    glyph_mask = _polygon_mask((h, w), polygon)
    lo, hi = polygon.min(axis=0), polygon.max(axis=0)
    center = (lo + hi) / 2
    rw, rh = hi - lo
    margin = float(rng.uniform(.13, .24))
    # A real paper margin around the original perspective-distorted shape.
    # An arrow's minimum-area rectangle can align to its diagonal head and
    # turn a frontal card into a diamond. Keep the original bounding-box
    # aspect instead; only the ink carries the source camera projection.
    paper_rect = (tuple(map(float, center)), (max(6., float(rw) * (1 + 2 * margin)), max(6., float(rh) * (1 + 2 * margin))), 0.)
    paper_polygon = cv2.boxPoints(paper_rect)
    paper_mask = _polygon_mask((h, w), paper_polygon)
    # Guarantee that rasterized white remnants of the old glyph are covered.
    paper_mask = np.maximum(paper_mask, cv2.dilate(glyph_mask, np.ones((3, 3), np.uint8)))

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32)
    sigma = max(2., min(rw, rh) * .18)
    illumination = cv2.GaussianBlur(gray, (0, 0), sigma)
    local = illumination[paper_mask > 127]
    anchor = float(np.median(local)) if len(local) else 100.
    yy, xx = np.mgrid[:h, :w].astype(np.float32)
    slope_x, slope_y = rng.uniform(-8, 8, 2)
    gradient = slope_x * (xx - center[0]) / max(rw, 8) + slope_y * (yy - center[1]) / max(rh, 8)
    paper_gray = np.clip(float(rng.uniform(224, 249)) + .10 * (illumination - anchor) + gradient, 190, 255)
    paper = np.repeat(paper_gray[..., None], 3, axis=2)
    paper += rng.uniform(-2, 2, (1, 1, 3))
    result = _blend(image, paper, paper_mask)
    ink_level = float(rng.uniform(3, 28))
    ink = np.repeat(np.clip(ink_level + .025 * (illumination - anchor), 0, 40)[..., None], 3, axis=2)
    result = _blend(result, ink, glyph_mask)
    return result, [{'class': 2 if old_class == 27 else 1, 'box': _tight(glyph_mask)}]


def _arrow_mask(rng, class_id, holdout=False):
    """Render a centered canonical arrow on a high-resolution paper canvas."""
    n = 320 if not holdout else 384
    if holdout:
        # Deliberately separate shaft/head proportions and raster construction.
        head_base = float(rng.choice([rng.uniform(.35, .41), rng.uniform(.61, .67)]))
        shaft = float(rng.choice([rng.uniform(.25, .31), rng.uniform(.48, .54)]))
        aspect = float(rng.uniform(.90, 1.32))
    else:
        head_base = float(rng.uniform(.42, .60))
        shaft = float(rng.uniform(.32, .47))
        aspect = float(rng.uniform(.82, 1.45))
    margin = float(rng.uniform(.13, .23))
    span = n * (1 - 2 * margin)
    gw = span * min(1., aspect)
    gh = span * min(1., 1. / aspect)
    x0, y0 = (n - gw) / 2, (n - gh) / 2
    points = np.asarray([[0, (1-shaft)/2], [head_base, (1-shaft)/2],
                         [head_base, 0], [1, .5], [head_base, 1],
                         [head_base, (1+shaft)/2], [0, (1+shaft)/2]], np.float32)
    if class_id == 1:
        points[:, 0] = 1 - points[:, 0]
    points = points * [gw, gh] + [x0, y0]
    mask = np.zeros((n, n), np.uint8)
    if holdout:
        # Union of a shaft quadrilateral and head triangle, not the training
        # seven-vertex contour rendering path.
        shaft_poly = np.asarray([[0, (1-shaft)/2], [head_base, (1-shaft)/2],
                                 [head_base, (1+shaft)/2], [0, (1+shaft)/2]], np.float32)
        head_poly = np.asarray([[head_base, 0], [1, .5], [head_base, 1]], np.float32)
        if class_id == 1:
            shaft_poly[:, 0] = 1 - shaft_poly[:, 0]
            head_poly[:, 0] = 1 - head_poly[:, 0]
        for component in (shaft_poly, head_poly):
            cv2.fillPoly(mask, [np.round(component * [gw, gh] + [x0, y0]).astype(np.int32)], 255, cv2.LINE_AA)
    else:
        cv2.fillPoly(mask, [np.round(points).astype(np.int32)], 255, cv2.LINE_AA)
    return mask, max(gw, gh)


def _camera_degrade(image, rng, size):
    """Mild full-frame camera effects, without further spatial downsampling."""
    if rng.random() < .35:
        image = np.clip(image.astype(np.float32) * rng.uniform(.88, 1.07)
                        + rng.uniform(-6, 6), 0, 255).astype(np.uint8)
    if rng.random() < .25:
        # Tiny arrows already lose detail during the projective warp; cap
        # extra blur there instead of shrinking and enlarging them again.
        sigma = float(rng.uniform(.25, .40 if size < 24 else .60))
        image = cv2.GaussianBlur(image, (3, 3), sigma)
    if rng.random() < .35:
        noise = rng.normal(0, rng.uniform(.25, 1.35), image.shape)
        image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    if rng.random() < .30:
        quality = int(rng.integers(84 if size < 24 else 78, 98))
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    return image


def canonical_scene(background, rng, class_id, size, holdout=False, negative_kind=None):
    """Paste a black glyph on a white card, optionally mounted on a dark block.

    negative_kind is None, 'up', 'down', 'triangle', or 'blank'. Negative
    cards have no detection targets; the background must itself be free of
    left/right/bullseye targets. Targets remain horizontally oriented under
    mild roll and perspective. No horizontal reflection occurs here.
    """
    if class_id not in (1, 2):
        raise ValueError("Canonical positive class_id must be 1 (left) or 2 (right)")
    if not 6 <= float(size) <= 360:
        raise ValueError("size must be between 6 and 360 pixels")
    if negative_kind not in (None, 'up', 'down', 'triangle', 'blank'):
        raise ValueError("Unknown negative card kind")
    image = cv2.resize(np.asarray(background, dtype=np.uint8), (640, 640), interpolation=cv2.INTER_AREA).copy()
    glyph, glyph_span = _arrow_mask(rng, class_id, holdout)
    n = glyph.shape[0]
    if negative_kind in ('up', 'down'):
        # Normalize to a right arrow first, then rotate it vertically.
        if class_id == 1:
            glyph = cv2.flip(glyph, 1)
        glyph = cv2.rotate(glyph, cv2.ROTATE_90_COUNTERCLOCKWISE if negative_kind == 'up' else cv2.ROTATE_90_CLOCKWISE)
    elif negative_kind == 'triangle':
        glyph.fill(0)
        pts = np.asarray([[.23, .22], [.78, .5], [.23, .78]]) * n
        if rng.random() < .5:
            pts[:, 0] = n - pts[:, 0]
        cv2.fillPoly(glyph, [np.round(pts).astype(np.int32)], 255, cv2.LINE_AA)
    elif negative_kind == 'blank':
        glyph.fill(0)

    yy, xx = np.mgrid[:n, :n].astype(np.float32)
    base_white = float(rng.uniform(221, 255))
    shade = (xx / n - .5) * rng.uniform(-24, 24) + (yy / n - .5) * rng.uniform(-18, 18)
    if rng.random() < .20:
        # Soft shadow across the paper; the paper itself stays light.
        shadow_center = float(rng.uniform(.15, .85)) * n
        shade -= float(rng.uniform(5, 22)) * np.exp(-((xx - shadow_center) / (.24 * n)) ** 2)
    card = np.repeat(np.clip(base_white + shade, 183, 255)[..., None], 3, axis=2)
    card += rng.uniform(-2, 2, (1, 1, 3))
    ink = np.full_like(card, rng.uniform(0, 31))
    card = _blend(np.clip(card, 0, 255).astype(np.uint8), ink, glyph)

    corners = np.asarray([[0, 0], [n-1, 0], [n-1, n-1], [0, n-1]], np.float32)
    dest = corners - (n-1)/2
    # A single perspective transform from a frontal print, avoiding double
    # application of the camera perspective present in legacy glyph contours.
    dest[:, 0] *= float(rng.uniform(.67, 1.03))
    dest[:, 1] *= float(rng.uniform(.84, 1.03))
    dest += rng.uniform(-.045*n, .045*n, (4, 2)).astype(np.float32)
    angle = float(rng.uniform(-9, 9)) * np.pi/180
    rotation = np.asarray([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]], np.float32)
    dest = (dest @ rotation.T) * (float(size)/glyph_span)
    radius = np.max(np.abs(dest), axis=0)
    # Large requested prints must still fit completely within the frame.
    fit = min(1., 304. / float(radius.max()))
    dest *= fit
    radius *= fit
    center = np.asarray([rng.uniform(radius[0]+8, 632-radius[0]), rng.uniform(radius[1]+8, 632-radius[1])], np.float32)
    dest += center
    matrix = cv2.getPerspectiveTransform(corners, dest.astype(np.float32))

    if rng.random() < .72:
        # Paper on a dark obstacle face, with a slightly lighter side face.
        top_left = dest[0] + [-size*.25, -size*.22]
        top_right = dest[1] + [size*.25, -size*.22]
        bottom_right = dest[2] + [size*.31, size*rng.uniform(.50, 1.65)]
        bottom_left = dest[3] + [-size*.25, size*rng.uniform(.50, 1.65)]
        obstacle = np.asarray([top_left, top_right, bottom_right, bottom_left], np.float32)
        shade = int(rng.integers(12, 55))
        cv2.fillPoly(image, [np.round(obstacle).astype(np.int32)], (shade, shade, shade), cv2.LINE_AA)
        if rng.random() < .65:
            side_offset = np.asarray([size*rng.uniform(.15, .35), -size*.12])
            side = np.asarray([top_right, top_right+side_offset, bottom_right+side_offset, bottom_right])
            side_shade = int(np.clip(shade+rng.integers(-8, 22), 0, 75))
            cv2.fillPoly(image, [np.round(side).astype(np.int32)], (side_shade,)*3, cv2.LINE_AA)

    warped_card = cv2.warpPerspective(card, matrix, (640, 640), flags=cv2.INTER_LINEAR)
    paper_mask = cv2.warpPerspective(np.full((n, n), 255, np.uint8), matrix, (640, 640), flags=cv2.INTER_LINEAR)
    image = _blend(image, warped_card.astype(np.float32), paper_mask)
    warped_glyph = cv2.warpPerspective(glyph, matrix, (640, 640), flags=cv2.INTER_LINEAR)
    objects = [] if negative_kind else [{'class': int(class_id), 'box': _tight(warped_glyph)}]
    return _camera_degrade(image, rng, size), objects
