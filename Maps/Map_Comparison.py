import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
from textwrap import dedent

# ============================================================
# 0) Config you can tweak (safe defaults)
# ============================================================

# Fine alignment search window (in pixels). Larger = slower, potentially tighter align.
MAX_SHIFT = 15

# If we cannot find any real scale, default to ROS map default resolution (0.05 m/px = 5 cm/px)
DEFAULT_CM_PER_PX = 5.0

# Real-world maze size (meters) from your notes (used as a fallback scale estimator)
# Testing environment: length 2.438 m, width 1.3716 m
MAZE_LENGTH_M = 2.438
MAZE_WIDTH_M  = 1.3716

# ============================================================
# 1) I/O helpers (same as before, with a YAML parser added)
# ============================================================

def load_gray(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f" Could not load file: {path}")
    return img

def fit_and_center_to_canvas(img, canvas_size=(500, 500), bg=255):
    """
    Scale image to fit inside canvas (keep aspect ratio), then center it on a white canvas.
    Works for any input size (bigger/smaller than canvas).
    """
    Hc, Wc = canvas_size[1], canvas_size[0]  # careful: (w,h) vs shape
    h, w = img.shape[:2]
    # scale to fit (allow upscaling and downscaling)
    scale = min(Wc / w, Hc / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
    canvas = np.full((Hc, Wc), bg, dtype=np.uint8)
    # center it
    x0 = (Wc - new_w) // 2
    y0 = (Hc - new_h) // 2
    canvas[y0:y0+new_h, x0:x0+new_w] = resized
    return canvas

def try_read_yaml_resolution(img_path):
    """
    Try to find a sidecar YAML next to img_path and read 'resolution' (meters/pixel).
    Returns meters_per_px or None.
    """
    base, ext = os.path.splitext(img_path)
    cand_paths = [base + ".yaml", base + ".yml"]
    for yp in cand_paths:
        if os.path.exists(yp):
            try:
                with open(yp, "r") as f:
                    data = yaml.safe_load(f)
                if isinstance(data, dict) and "resolution" in data:
                    return float(data["resolution"])
            except Exception:
                pass
    return None

# ============================================================
# 2) Alignment helpers (same logic + tiny improvements)
# ============================================================

def binarize(img):
    """
    Otsu threshold; make obstacles/walls white (invert if needed).
    """
    _, th = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # Heuristic: ensure foreground (walls) are white (255) and background is black (0)
    if np.sum(th == 255) > np.sum(th == 0):
        th = cv2.bitwise_not(th)
    return th

def major_axis_angle(binary):
    """
    Use largest contour's min-area rectangle to get "long-side" angle (degrees).
    """
    cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0.0
    cnt = max(cnts, key=cv2.contourArea)
    rect = cv2.minAreaRect(cnt)         # ((cx,cy), (w,h), angle in [-90,0))
    (w, h) = rect[1]
    angle = rect[2]
    if w < h:
        angle += 90
    return angle

def rotate_image(img, angle_deg):
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w//2, h//2), angle_deg, 1.0)
    return cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_NEAREST, borderValue=255)

def center_image(binary_or_gray):
    """
    Translate centroid to image center.
    Works on binary or grayscale; uses binary moment if it's binary-like.
    """
    img = binary_or_gray
    h, w = img.shape[:2]
    # Compute centroid using a binary mask (treat dark as background)
    bin_img = binarize(img) if img.dtype != np.uint8 else binarize(img)
    M = cv2.moments(bin_img, binaryImage=True)
    if M['m00'] == 0:
        return img
    cx = M['m10'] / M['m00']
    cy = M['m01'] / M['m00']
    tx = (w/2) - cx
    ty = (h/2) - cy
    T = np.float32([[1, 0, tx], [0, 1, ty]])
    return cv2.warpAffine(img, T, (w, h), flags=cv2.INTER_NEAREST, borderValue=255)

def normalize(img):
    """
    1) binarize, 2) rotate so long side is horizontal,
    3) re-center by centroid. Returns (rotated_gray_centered, rotated_bin_centered, angle)
    """
    bin_img = binarize(img)
    ang = major_axis_angle(bin_img)
    rot_g = rotate_image(img, ang)
    rot_b = rotate_image(bin_img, ang)
    rot_g_c = center_image(rot_g)
    rot_b_c = center_image(rot_b)
    return rot_g_c, rot_b_c, ang

def correlation(a, b):
    """
    Normalized cross-correlation of two equal-sized grayscale/binary images.
    """
    a_f = a.astype(np.float32) / 255.0
    b_f = b.astype(np.float32) / 255.0
    num = np.sum((a_f - a_f.mean()) * (b_f - b_f.mean()))
    den = np.sqrt(np.sum((a_f - a_f.mean())**2) * np.sum((b_f - b_f.mean())**2)) + 1e-9
    return num / den

def best_flip_to_match(img_bin, ref_bin):
    """
    Try flips (none, H, V, HV); pick best by correlation vs reference.
    """
    candidates = {
        "none": img_bin,
        "flip_h": cv2.flip(img_bin, 1),
        "flip_v": cv2.flip(img_bin, 0),
        "flip_hv": cv2.flip(img_bin, -1),
    }
    best_name, best_img, best_score = "none", img_bin, -1.0
    for name, candidate in candidates.items():
        score = correlation(candidate, ref_bin)
        if score > best_score:
            best_name, best_img, best_score = name, candidate, score
    return best_img, best_name, best_score

def apply_flip(img, flip_name):
    if flip_name == "flip_h":  return cv2.flip(img, 1)
    if flip_name == "flip_v":  return cv2.flip(img, 0)
    if flip_name == "flip_hv": return cv2.flip(img, -1)
    return img

def translate_image(img, dx, dy, border=255):
    h, w = img.shape[:2]
    T = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(img, T, (w, h), flags=cv2.INTER_NEAREST, borderValue=border)

def fine_align_by_translation(mov_img, ref_img, max_shift=15):
    """
    Brute-force integer (dx,dy) search in [-max_shift, +max_shift] to maximize correlation.
    Returns best_aligned_img, best_dx, best_dy, best_score.
    """
    best_score = -1.0
    best = (mov_img, 0, 0, best_score)
    for dy in range(-max_shift, max_shift+1):
        for dx in range(-max_shift, max_shift+1):
            shifted = translate_image(mov_img, dx, dy, border=255)
            score = correlation(shifted, ref_img)
            if score > best_score:
                best_score = score
                best = (shifted, dx, dy, best_score)
    return best

# ============================================================
# 3) Map-error metrics (edge-based symmetric Chamfer distance)
# ============================================================

def edges(binary):
    """
    Extract edges from a binary occupancy (white=walls/obstacles) using Canny.
    """
    # Ensure binary is 0/255
    b = (binary > 127).astype(np.uint8) * 255
    e = cv2.Canny(b, 50, 150, L2gradient=True)
    return e

def mean_edge_distance(A_edges, B_edges):
    """
    Mean distance from each edge pixel in A to the nearest edge pixel in B (in pixels).
    Uses a distance transform on the inverse of B_edges.
    """
    # Distance transform expects non-edges as positive domain.
    # Create a mask where 0=edges, 255=non-edges, then run distanceTransform.
    invB = np.where(B_edges > 0, 0, 255).astype(np.uint8)
    distB = cv2.distanceTransform(invB, cv2.DIST_L2, 3)

    # Sample distances at edge pixels of A
    ys, xs = np.where(A_edges > 0)
    if len(xs) == 0:
        return 0.0
    dists = distB[ys, xs]
    return float(np.mean(dists))

def symmetric_chamfer_mae(est_bin, gt_bin):
    """
    Symmetric mean surface distance (pixels):
      ( mean_{p in edges(est)} d(p, edges(gt)) + mean_{q in edges(gt)} d(q, edges(est)) ) / 2
    This implements the MAE-style feature-to-feature error used in your poster explanation.
    """
    E_est = edges(est_bin)
    E_gt  = edges(gt_bin)
    d_est_to_gt = mean_edge_distance(E_est, E_gt)
    d_gt_to_est = mean_edge_distance(E_gt, E_est)
    return 0.5 * (d_est_to_gt + d_gt_to_est)

# ============================================================
# 4) Scale estimation (pixels -> centimeters)
# ============================================================

def bbox_content_size(binary):
    ys, xs = np.where(binary > 0)
    if len(xs) == 0:
        return (1, 1)
    w = xs.max() - xs.min() + 1
    h = ys.max() - ys.min() + 1
    return (w, h)

def estimate_cm_per_px_from_drawn(drawn_bin):
    """
    Use drawn (ground truth) content bbox vs real maze size to estimate a pixels->cm scale.
    We don't assume which side is length vs width; we compute both and average.
    """
    bw, bh = bbox_content_size(drawn_bin)  # in pixels
    # Real dimensions in cm
    L_cm = MAZE_LENGTH_M * 100.0
    W_cm = MAZE_WIDTH_M  * 100.0
    # Two possible orientation mappings; take the mean of the two implied scales
    # (This is a reasonable compromise if we don't know which side aligned to which).
    scales = []
    if bw > 0:
        scales.append(L_cm / bw)
        scales.append(W_cm / bw)
    if bh > 0:
        scales.append(L_cm / bh)
        scales.append(W_cm / bh)
    scales = [s for s in scales if np.isfinite(s) and s > 0]
    if not scales:
        return None
    # Median is robust to outliers
    return float(np.median(scales))

def choose_cm_per_px(cart_yaml_mpp, slam_yaml_mpp, drawn_yaml_mpp, drawn_bin):
    """
    Priority:
    1) If any YAML has 'resolution' (m/px), prefer the drawn map's, else average available.
    2) Else estimate from drawn bbox and known maze dimensions.
    3) Else fall back to DEFAULT_CM_PER_PX.
    """
    mpps = [x for x in [drawn_yaml_mpp, cart_yaml_mpp, slam_yaml_mpp] if x is not None]
    if len(mpps) > 0:
        cm_per_px = float(np.mean(mpps) * 100.0)
        return cm_per_px
    # Estimate from geometry
    est = estimate_cm_per_px_from_drawn(drawn_bin)
    if est is not None:
        return est
    return DEFAULT_CM_PER_PX

# ============================================================
# 5) Main (interactive)
# ============================================================

print("  Map Comparison, Alignment & Error Tool")
print("----------------------------------------")
cart_name  = input("Enter the filename for the Cartographer map: ").strip()
slam_name  = input("Enter the filename for the SLAM Toolbox map: ").strip()
drawn_name = input("Enter the filename for the Drawn (manual) map: ").strip()

# Load original (grayscale)
cart_raw  = load_gray(cart_name)
slam_raw  = load_gray(slam_name)
drawn_raw = load_gray(drawn_name)

# Try to read sidecar YAML resolutions (meters/pixel)
cart_mpp  = try_read_yaml_resolution(cart_name)
slam_mpp  = try_read_yaml_resolution(slam_name)
drawn_mpp = try_read_yaml_resolution(drawn_name)

# Force all to 500×500 centered canvases (keeps aspect ratio consistent)
cart_500  = fit_and_center_to_canvas(cart_raw, (500, 500), bg=255)
slam_500  = fit_and_center_to_canvas(slam_raw, (500, 500), bg=255)
drawn_500 = fit_and_center_to_canvas(drawn_raw, (500, 500), bg=255)

# Save preprocessed versions (handy for debugging)
cv2.imwrite("pre_cartographer_500.png", cart_500)
cv2.imwrite("pre_slam_500.png", slam_500)
cv2.imwrite("pre_drawn_500.png", drawn_500)

# Normalize orientation + center (still in 500×500 space)
cart_rot, cart_bin, cart_ang   = normalize(cart_500)
slam_rot, slam_bin, slam_ang   = normalize(slam_500)
drawn_rot, drawn_bin, drawn_ang = normalize(drawn_500)

# Flip SLAM & Drawn to best match Cartographer as reference
_, slam_flip, slam_score0 = best_flip_to_match(slam_bin,  cart_bin)
_, draw_flip, draw_score0 = best_flip_to_match(drawn_bin, cart_bin)

slam_aligned = apply_flip(slam_rot,  slam_flip)
draw_aligned = apply_flip(drawn_rot, draw_flip)

slam_bin_aligned = apply_flip(slam_bin,  slam_flip)
draw_bin_aligned = apply_flip(drawn_bin, draw_flip)

# NEW: fine translation alignment (integer dx,dy search) for tighter overlay
slam_aligned, slam_dx, slam_dy, slam_score = fine_align_by_translation(slam_aligned, cart_rot, MAX_SHIFT)
draw_aligned, draw_dx, draw_dy, draw_score = fine_align_by_translation(draw_aligned, cart_rot, MAX_SHIFT)

slam_bin_aligned = translate_image(slam_bin_aligned, slam_dx, slam_dy, border=255)
draw_bin_aligned = translate_image(draw_bin_aligned, draw_dx, draw_dy, border=255)

# Save outputs
cv2.imwrite("aligned_cartographer.png", cart_rot)
cv2.imwrite("aligned_slam.png",         slam_aligned)
cv2.imwrite("aligned_drawn.png",        draw_aligned)

# Build overlay preview
def colorize(gray, channel):
    c = np.zeros((gray.shape[0], gray.shape[1], 3), dtype=np.uint8)
    c[:, :, channel] = gray
    return c

overlay = cv2.addWeighted(colorize(cart_rot, 0), 0.4, colorize(slam_aligned, 1), 0.4, 0)
overlay = cv2.addWeighted(overlay, 1.0, colorize(draw_aligned, 2), 0.4, 0)
cv2.imwrite("aligned_overlay_preview.png", cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))

# ============================================================
# 6) MAP ERROR (matches your poster's MAE intent)
#     e_m ≈ symmetric mean distance between SLAM edges and GT edges
# ============================================================

# Choose cm-per-pixel scale
cm_per_px = choose_cm_per_px(cart_mpp, slam_mpp, drawn_mpp, draw_bin_aligned)

# Compute symmetric chamfer MAE vs ground-truth (Drawn) for each algorithm
# Reference / ground truth:
gt_bin = draw_bin_aligned

cart_mae_px = symmetric_chamfer_mae(cart_bin, gt_bin)
slam_mae_px = symmetric_chamfer_mae(slam_bin_aligned, gt_bin)

cart_mae_cm = cart_mae_px * cm_per_px
slam_mae_cm = slam_mae_px * cm_per_px

# Recompute correlation vs GT (post fine-alignment) for reporting
cart_corr_gt = correlation(cart_bin, gt_bin)
slam_corr_gt = correlation(slam_bin_aligned, gt_bin)

# ============================================================
# 7) Print & save table for your poster's "RESULTS" box
# ============================================================

rows = [
    ("SLAM Toolbox",   f"{slam_mae_px:0.2f}", f"{slam_mae_cm:0.2f}", f"{slam_corr_gt:0.3f}"),
    ("Cartographer",   f"{cart_mae_px:0.2f}", f"{cart_mae_cm:0.2f}", f"{cart_corr_gt:0.3f}"),
]

col_names = ["Algorithm", "Mean Map Error (px)", "Mean Map Error (cm)*", "Correlation Coefficient"]

def format_table(rows, headers):
    # simple monospace table
    widths = [max(len(h), *(len(r[i]) for r in rows)) for i, h in enumerate(headers)]
    def fmt_row(r): return " | ".join(s.ljust(widths[i]) for i, s in enumerate(r))
    line = "-+-".join("-" * w for w in widths)
    out  = [fmt_row(headers), line]
    out += [fmt_row(r) for r in rows]
    return "\n".join(out)

table_text = format_table(rows, col_names)
print("\n=== RESULTS ===")
print(table_text)
print(f"\n* cm/px used: {cm_per_px:0.3f}  (Derived from YAML if present; otherwise estimated from maze size or default)")

with open("results_table.txt", "w") as f:
    f.write(table_text + f"\n\n* cm/px used: {cm_per_px:0.3f}\n")

# Also save CSV for easy copy into spreadsheets
import csv
with open("results_table.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(col_names)
    for r in rows: w.writerow(r)

# ============================================================
# 8) Show panels (kept same, with minor title tweaks)
# ============================================================

fig, axs = plt.subplots(2, 3, figsize=(12, 8))
axs = axs.ravel()
axs[0].imshow(cart_500, cmap='gray');  axs[0].set_title("Cartographer (500×500)"); axs[0].axis('off')
axs[1].imshow(slam_500, cmap='gray');  axs[1].set_title("SLAM (500×500)");        axs[1].axis('off')
axs[2].imshow(drawn_500, cmap='gray'); axs[2].set_title("Drawn (500×500)");       axs[2].axis('off')
axs[3].imshow(cart_rot, cmap='gray');  axs[3].set_title(f"Cart aligned (rot {cart_ang:.1f}°)"); axs[3].axis('off')
axs[4].imshow(slam_aligned, cmap='gray'); axs[4].set_title(f"SLAM aligned (dx {slam_dx}, dy {slam_dy})"); axs[4].axis('off')
axs[5].imshow(draw_aligned, cmap='gray'); axs[5].set_title(f"Drawn aligned (dx {draw_dx}, dy {draw_dy})"); axs[5].axis('off')
plt.tight_layout()
plt.show()

# Final overlay
plt.figure(figsize=(6,6))
plt.title("Aligned Overlay (Blue=Cart, Green=SLAM, Red=Drawn)")
plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()

print("\nSaved files:")
print(" - pre_cartographer_500.png")
print(" - pre_slam_500.png")
print(" - pre_drawn_500.png")
print(" - aligned_cartographer.png")
print(" - aligned_slam.png")
print(" - aligned_drawn.png")
print(" - aligned_overlay_preview.png")
print(" - results_table.txt")
print(" - results_table.csv")
