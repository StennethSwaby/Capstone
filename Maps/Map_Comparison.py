import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import math
import yaml
import csv

# ============================================================
# 0) Config you can tweak (safe defaults)
# ============================================================

# Fine alignment search window (in pixels). Larger = slower, potentially tighter align.
MAX_SHIFT = 15

# If we cannot find any real scale, default to ROS map default resolution (0.05 m/px = 5 cm/px)
DEFAULT_CM_PER_PX = 5.0

# Real-world maze size (meters) from your notes (used as a fallback scale estimator)
# Testing environment: length 2.438m, width 1.3716m
MAZE_LENGTH_M = 2.438
MAZE_WIDTH_M  = 1.3716

# How many maps per algorithm to process (professor requested 10)
N_PER_ALGO = 10

# ============================================================
# 1) I/O helpers
# ============================================================

def load_gray(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not load file: {path}")
    return img

def fit_and_center_to_canvas(img, canvas_size=(500, 500), bg=255):
    """
    Scale image to fit inside canvas (keep aspect ratio), then center it on a white canvas.
    Works for any input size (bigger/smaller than canvas).
    """
    Wc, Hc = canvas_size
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
# 2) Alignment helpers
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

def center_image(img):
    """
    Translate centroid to image center.
    Works on binary or grayscale; uses binary moments.
    """
    h, w = img.shape[:2]
    bin_img = binarize(img)
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
# 3) Map-error metrics using YOUR FORMULA
# ============================================================

def compute_map_error_with_formula(est_bin, gt_bin):
    """
    ========================================================================
    CRITICAL FUNCTION: MAP ERROR CALCULATION USING YOUR PROVIDED FORMULA
    ========================================================================
    
    Computes map error using the formula:
    
    e_M = (1/N) * Σ sqrt((x_i - x_i^GT)^2 + (y_i - y_i^GT)^2)
    
    where:
    - N is the number of edge pixels in the estimated map
    - (x_i, y_i) are coordinates of edge pixels in the estimated map
    - (x_i^GT, y_i^GT) are coordinates of corresponding nearest pixels in ground truth
    
    This function:
    1) Extracts edge pixels from the estimated map (SLAM/Cartographer output)
    2) Extracts edge pixels from the ground truth map
    3) For each edge pixel in estimated map, finds nearest edge pixel in GT
    4) Computes Euclidean distance between matched pairs
    5) Returns the average (mean) of all these distances
    
    Parameters:
    -----------
    est_bin : numpy array
        Binary estimated map (from SLAM algorithm)
    gt_bin : numpy array
        Binary ground truth map
        
    Returns:
    --------
    float : Map error e_M in pixels (mean distance per edge pixel)
    """
    
    # STEP 1: Extract edges from both maps using Canny edge detection
    # This identifies the walls/obstacles boundaries
    est_edges = cv2.Canny((est_bin > 127).astype(np.uint8) * 255, 50, 150, L2gradient=True)
    gt_edges = cv2.Canny((gt_bin > 127).astype(np.uint8) * 255, 50, 150, L2gradient=True)
    
    # STEP 2: Get coordinates of all edge pixels in estimated map
    # These are our (x_i, y_i) points - N total points
    yi_coords, xi_coords = np.where(est_edges > 0)
    N = len(xi_coords)  # This is N in your formula
    
    if N == 0:
        return 0.0  # No edges found, return zero error
    
    # STEP 3: Build distance transform of ground truth edges
    # This allows us to quickly find distance to nearest GT edge pixel
    # for any point in the image
    inv_gt_edges = np.where(gt_edges > 0, 0, 255).astype(np.uint8)
    dist_transform = cv2.distanceTransform(inv_gt_edges, cv2.DIST_L2, 3)
    
    # STEP 4: For each edge pixel (x_i, y_i) in estimated map,
    # look up its distance to nearest GT edge pixel (x_i^GT, y_i^GT)
    # This is the sqrt((x_i - x_i^GT)^2 + (y_i - y_i^GT)^2) term
    distances = dist_transform[yi_coords, xi_coords]
    
    # STEP 5: Compute mean of all distances - this is (1/N) * Σ(...)
    # This is your e_M formula result in pixels
    e_M = float(np.mean(distances))
    
    return e_M

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
    """
    bw, bh = bbox_content_size(drawn_bin)  # in pixels
    L_cm = MAZE_LENGTH_M * 100.0
    W_cm = MAZE_WIDTH_M  * 100.0
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
    return float(np.median(scales))

def choose_cm_per_px(cart_mpp, slam_mpp, drawn_mpp, drawn_bin):
    """
    Priority:
    1) If any YAML has 'resolution' (m/px), use the mean of available.
    2) Else estimate from drawn bbox and known maze dimensions.
    3) Else fall back to DEFAULT_CM_PER_PX.
    """
    mpps = [x for x in [drawn_mpp, cart_mpp, slam_mpp] if x is not None]
    if len(mpps) > 0:
        cm_per_px = float(np.mean(mpps) * 100.0)
        return cm_per_px
    est = estimate_cm_per_px_from_drawn(drawn_bin)
    if est is not None:
        return est
    return DEFAULT_CM_PER_PX

# ============================================================
# 5) High-level processing helpers
# ============================================================

def preprocess_ground_truth(drawn_path):
    """
    Load, resize, normalize the drawn (manual) ground-truth map.
    Returns gray_500, rot_gray, rot_bin.
    """
    drawn_raw = load_gray(drawn_path)
    drawn_500 = fit_and_center_to_canvas(drawn_raw, (500, 500), bg=255)
    drawn_rot, drawn_bin, drawn_ang = normalize(drawn_500)
    cv2.imwrite("pre_drawn_500.png", drawn_500)
    cv2.imwrite("aligned_drawn.png", drawn_rot)
    return drawn_500, drawn_rot, drawn_bin, drawn_ang

def process_group(file_list, ref_gray, ref_bin, algo_name, prefix):
    """
    For a set of N maps belonging to one algorithm (Cartographer or SLAM):
      - Load, resize (500x500), normalize (rectangular alignment)
      - Flip + fine-translate to align with the ground-truth reference
      - Compute per-run map error e_M using YOUR FORMULA vs GT
      - Stack aligned images and build an average map

    Returns a dict with:
      stack_gray, stack_bin, avg_gray, avg_bin, errors_px, corr_avg
    """
    stack_gray = []
    stack_bin  = []
    errors_px  = []

    print(f"\nProcessing {algo_name} maps...")
    for idx, path in enumerate(file_list):
        print(f"  [{algo_name}] Map {idx+1}/{len(file_list)}: {path}")
        raw    = load_gray(path)
        canvas = fit_and_center_to_canvas(raw, (500, 500), bg=255)
        gray_norm, bin_norm, angle = normalize(canvas)

        # Flip & align vs reference (ground truth)
        best_bin, flip_name, corr0 = best_flip_to_match(bin_norm, ref_bin)
        gray_flipped = apply_flip(gray_norm, flip_name)

        gray_aligned, dx, dy, corr = fine_align_by_translation(
            gray_flipped, ref_gray, MAX_SHIFT
        )
        bin_aligned = translate_image(best_bin, dx, dy, border=255)

        # Save per-run aligned maps
        out_name = f"{prefix}_aligned_{idx+1}.png"
        cv2.imwrite(out_name, gray_aligned)

        # Compute map error e_M (in pixels) for this run using YOUR FORMULA
        err_px = compute_map_error_with_formula(bin_aligned, ref_bin)
        errors_px.append(err_px)

        stack_gray.append(gray_aligned)
        stack_bin.append(bin_aligned)

    stack_gray = np.stack(stack_gray, axis=0)
    stack_bin  = np.stack(stack_bin, axis=0)

    # Average map (grayscale)
    avg_gray = np.mean(stack_gray, axis=0).astype(np.uint8)

    # Average occupancy (binary-ish) then re-threshold to 0/255
    avg_bin_f = np.mean(stack_bin, axis=0)
    avg_bin   = (avg_bin_f > 127).astype(np.uint8) * 255

    # Correlation of average against ground truth
    corr_avg = correlation(avg_bin, ref_bin)

    # Save average map
    cv2.imwrite(f"{prefix}_average.png", avg_gray)

    return {
        "stack_gray": stack_gray,
        "stack_bin":  stack_bin,
        "avg_gray":   avg_gray,
        "avg_bin":    avg_bin,
        "errors_px":  np.array(errors_px, dtype=np.float32),
        "corr_avg":   corr_avg,
    }

def colorize(gray, channel):
    """
    Put grayscale image into one color channel (0=B,1=G,2=R) for overlays.
    """
    c = np.zeros((gray.shape[0], gray.shape[1], 3), dtype=np.uint8)
    c[:, :, channel] = gray
    return c

def build_overlay(gt_gray, algo_avg_gray, name):
    """
    Overlay ground-truth (red) and algorithm average map (green) for visualization.
    """
    gt_c   = colorize(gt_gray, 2)        # red channel
    algo_c = colorize(algo_avg_gray, 1)  # green channel
    overlay = cv2.addWeighted(gt_c, 0.6, algo_c, 0.6, 0)
    out_name = f"overlay_{name}_vs_groundtruth.png"
    cv2.imwrite(out_name, cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
    return overlay, out_name

def compute_stats(errors_px, cm_per_px):
    """
    Given array of e_M values (pixels), compute mean, variance, MSE, RMSE,
    then convert to centimeters.
    """
    mean_px = float(np.mean(errors_px))
    var_px  = float(np.var(errors_px))
    mse_px  = float(np.mean(errors_px**2))
    rmse_px = float(np.sqrt(mse_px))           # RMSE in pixels

    mean_cm = mean_px * cm_per_px
    var_cm  = var_px * (cm_per_px**2)
    mse_cm  = mse_px * (cm_per_px**2)
    rmse_cm = float(np.sqrt(mse_cm))           # RMSE in cm

    return {
        "mean_px": mean_px,
        "var_px":  var_px,
        "mse_px":  mse_px,
        "rmse_px": rmse_px,
        "mean_cm": mean_cm,
        "var_cm":  var_cm,
        "mse_cm":  mse_cm,
        "rmse_cm": rmse_cm,
    }

def format_table(rows, headers):
    # simple monospace table for console + text file
    widths = [max(len(h), *(len(str(r[i])) for r in rows)) for i, h in enumerate(headers)]
    def fmt_row(r): return " | ".join(str(r[i]).ljust(widths[i]) for i in range(len(headers)))
    line = "-+-".join("-" * w for w in widths)
    out  = [fmt_row(headers), line]
    out += [fmt_row(r) for r in rows]
    return "\n".join(out)

# ============================================================
# 6) Main
# ============================================================

def read_all_maps(drawn_filename, n_per_algo=10):
    """
    Helper to collect and validate all required map files:
      - Ground truth (drawn) map
      - Cartographer maps (cartographer1_map.pgm ... cartographerN_map.pgm)
      - SLAM Toolbox maps (slam_toolbox1_map.pgm ... slam_toolboxN_map.pgm)

    Returns:
        drawn_filename (str)
        cart_files (list[str])
        slam_files (list[str])
    """
    # Check ground truth exists
    if not os.path.exists(drawn_filename):
        raise FileNotFoundError(f"Ground truth file not found: {drawn_filename}")

    # Build Cartographer filenames and verify each exists
    cart_files = [f"cartographer{i+1}_map.pgm" for i in range(n_per_algo)]
    for f in cart_files:
        if not os.path.exists(f):
            raise FileNotFoundError(f"Missing Cartographer map: {f}")

    # Build SLAM Toolbox filenames and verify each exists
    slam_files = [f"slam_toolbox{i+1}_map.pgm" for i in range(n_per_algo)]
    for f in slam_files:
        if not os.path.exists(f):
            raise FileNotFoundError(f"Missing SLAM Toolbox map: {f}")

    print("\nGround truth map:")
    print(f"  {drawn_filename}")
    print("\nCartographer maps:")
    for f in cart_files:
        print(f"  {f}")
    print("\nSLAM Toolbox maps:")
    for f in slam_files:
        print(f"  {f}")

    return drawn_filename, cart_files, slam_files

def format_per_run_table(algo_name, file_list, errors_px, cm_per_px, stats, corr, map_size_px=500):
    """
    Build a formatted text table showing per-run map errors for one algorithm.

    algo_name  : "Cartographer" or "SLAM Toolbox"
    file_list  : list of map filenames (same order as errors_px)
    errors_px  : list/array of e_M values in pixels
    cm_per_px  : scale (cm per pixel)
    stats      : dict from compute_stats(...) with mean_px, var_px, mse_px, rmse_px, mean_cm, var_cm, mse_cm, rmse_cm
    corr       : correlation between average map and ground truth (corr_avg)
    map_size_px: assumed width/height of the resized maps (default 500x500)

    % error here is defined as: e_M / diagonal * 100,
    where diagonal = sqrt(map_size_px^2 + map_size_px^2)
    """
    header_lines = []
    header_lines.append(f"=== PER-RUN MAP ERRORS: {algo_name.upper()} ===")
    header_lines.append(f"Scale: {cm_per_px:.3f} cm/pixel")
    header_lines.append("")

    # Table header
    lines = []
    lines.append(f"{'Run':>3}  {'Map filename':<30} {'e_M (px)':>10} {'e_M (cm)':>10} {'e_M (%)':>10}")
    lines.append("-" * 75)

    # Max possible distance ~ diagonal of 500x500 map
    diagonal_px = math.sqrt(map_size_px**2 + map_size_px**2)

    for idx, (fname, e_px) in enumerate(zip(file_list, errors_px), start=1):
        e_cm = e_px * cm_per_px
        e_pct = (e_px / diagonal_px * 100.0) if diagonal_px > 0 else 0.0
        base_name = os.path.basename(fname)
        lines.append(
            f"{idx:3d}  {base_name:<30} {e_px:10.3f} {e_cm:10.3f} {e_pct:10.2f}"
        )

    # Summary rows
    lines.append("-" * 75)
    lines.append(f"{'Mean e_M':<35} {stats['mean_px']:10.3f} {stats['mean_cm']:10.3f}")
    lines.append(f"{'Var(e_M)':<35} {stats['var_px']:10.3f} {stats['var_cm']:10.3f}")
    lines.append(f"{'MSE(e_M^2)':<35} {stats['mse_px']:10.3f} {stats['mse_cm']:10.3f}")
    lines.append(f"{'RMSE(e_M)':<35} {stats['rmse_px']:10.3f} {stats['rmse_cm']:10.3f}")
    lines.append(f"{'Corr(avg vs GT)':<35} {corr:10.3f}")

    return "\n".join(header_lines + lines)

def main():
    print("  Multi-Run Map Comparison, Alignment & Error Tool")
    print("---------------------------------------------------")
    print(f"This script expects {N_PER_ALGO} Cartographer maps and {N_PER_ALGO} SLAM Toolbox maps.")
    print("All maps will be resized to 500x500, normalized, aligned, and compared to a single ground-truth map.\n")

    # ---- Ground truth (drawn) ----
    drawn_name = input("Enter the filename for the Drawn (manual ground-truth) map: ").strip()

    # Use helper to gather and validate all map filenames
    drawn_name, cart_files, slam_files = read_all_maps(drawn_name, N_PER_ALGO)

    # Preprocess the ground-truth map (center, normalize, etc.)
    drawn_500, drawn_rot, drawn_bin, drawn_ang = preprocess_ground_truth(drawn_name)

    # Try reading YAML resolution for ground truth
    drawn_mpp = try_read_yaml_resolution(drawn_name)

    # Read example YAML resolutions from the first map of each group (resolution is the same for all runs)
    cart_mpp = try_read_yaml_resolution(cart_files[0]) if cart_files else None
    slam_mpp = try_read_yaml_resolution(slam_files[0]) if slam_files else None

    # Choose cm/px scale
    cm_per_px = choose_cm_per_px(cart_mpp, slam_mpp, drawn_mpp, drawn_bin)
    print(f"\nUsing scale: {cm_per_px:.3f} cm/pixel")

    # ---- Process Cartographer group ----
    cart_res = process_group(
        file_list=cart_files,
        ref_gray=drawn_rot,
        ref_bin=drawn_bin,
        algo_name="Cartographer",
        prefix="cartographer"
    )

    # ---- Process SLAM group ----
    slam_res = process_group(
        file_list=slam_files,
        ref_gray=drawn_rot,
        ref_bin=drawn_bin,
        algo_name="SLAM_Toolbox",
        prefix="slam"
    )

    # ---- Compute statistics for map error e_M ----
    cart_stats = compute_stats(cart_res["errors_px"], cm_per_px)
    slam_stats = compute_stats(slam_res["errors_px"], cm_per_px)

    # ---- Print per-run tables (new feature) ----
    cart_table = format_per_run_table(
        algo_name="Cartographer",
        file_list=cart_files,
        errors_px=cart_res["errors_px"],
        cm_per_px=cm_per_px,
        stats=cart_stats,
        corr=cart_res["corr_avg"],
        map_size_px=drawn_bin.shape[0]
    )

    slam_table = format_per_run_table(
        algo_name="SLAM Toolbox",
        file_list=slam_files,
        errors_px=slam_res["errors_px"],
        cm_per_px=cm_per_px,
        stats=slam_stats,
        corr=slam_res["corr_avg"],
        map_size_px=drawn_bin.shape[0]
    )

    print()
    print(cart_table)
    print()
    print(slam_table)
    print()

    # ---- Build overlays: average maps vs ground truth ----
    cart_overlay, cart_overlay_name = build_overlay(drawn_rot, cart_res["avg_gray"], "cartographer_avg")
    slam_overlay, slam_overlay_name = build_overlay(drawn_rot, slam_res["avg_gray"], "slam_avg")

    # ---- Prepare summary table for console + file ----
    headers = [
        "Algorithm",
        "Mean e_M (px)",
        "Var(e_M) (px^2)",
        "MSE(e_M^2) (px^2)",
        "RMSE(e_M) (px)",
        "Mean e_M (cm)",
        "Var(e_M) (cm^2)",
        "MSE(e_M^2) (cm^2)",
        "RMSE(e_M) (cm)",
        "Corr(avg vs GT)"
    ]

    rows = [
        [
            "SLAM Toolbox",
            f"{slam_stats['mean_px']:.3f}",
            f"{slam_stats['var_px']:.3f}",
            f"{slam_stats['mse_px']:.3f}",
            f"{slam_stats['rmse_px']:.3f}",
            f"{slam_stats['mean_cm']:.3f}",
            f"{slam_stats['var_cm']:.3f}",
            f"{slam_stats['mse_cm']:.3f}",
            f"{slam_stats['rmse_cm']:.3f}",
            f"{slam_res['corr_avg']:.3f}",
        ],
        [
            "Cartographer",
            f"{cart_stats['mean_px']:.3f}",
            f"{cart_stats['var_px']:.3f}",
            f"{cart_stats['mse_px']:.3f}",
            f"{cart_stats['rmse_px']:.3f}",
            f"{cart_stats['mean_cm']:.3f}",
            f"{cart_stats['var_cm']:.3f}",
            f"{cart_stats['mse_cm']:.3f}",
            f"{cart_stats['rmse_cm']:.3f}",
            f"{cart_res['corr_avg']:.3f}",
        ],
    ]

    print("\n=== RESULTS OVER 10 RUNS PER ALGORITHM ===")
    table_text = format_table(rows, headers)
    print(table_text)
    print(f"\n* e_M is the map error computed using formula: e_M = (1/N) * Σ sqrt((x_i - x_i^GT)^2 + (y_i - y_i^GT)^2)")
    print(f"* Scale used: {cm_per_px:.3f} cm/pixel")
    print(f"* MSE, RMSE, and Variance computed from the {N_PER_ALGO} individual run errors")

    # Save summary results
    with open("results_table.txt", "w") as f:
        f.write("Multi-run map comparison (10 runs each)\n\n")
        f.write(cart_table + "\n\n")
        f.write(slam_table + "\n\n")
        f.write(table_text)
        f.write(f"\n\n* e_M is the map error computed using formula: e_M = (1/N) * Σ sqrt((x_i - x_i^GT)^2 + (y_i - y_i^GT)^2)\n")
        f.write(f"* Scale used: {cm_per_px:.3f} cm/pixel\n")
        f.write(f"* MSE, RMSE, and Variance computed from the {N_PER_ALGO} individual run errors\n")

    with open("results_table.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(headers)
        for r in rows:
            w.writerow(r)

    # Save per-run raw error values
    with open("per_run_errors.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Algorithm", "Run", "error_px", "error_cm"])
        for i, e in enumerate(cart_res["errors_px"], start=1):
            w.writerow(["Cartographer", i, f"{e:.6f}", f"{e * cm_per_px:.6f}"])
        for i, e in enumerate(slam_res["errors_px"], start=1):
            w.writerow(["SLAM Toolbox", i, f"{e:.6f}", f"{e * cm_per_px:.6f}"])

    # ---- Visual Summary ----
    fig1, ax1 = plt.subplots(1, 1, figsize=(8, 8))
    ax1.imshow(cv2.cvtColor(cart_overlay, cv2.COLOR_BGR2RGB))
    ax1.set_title("Cartographer Average vs Ground Truth\n(Red=GT, Green=Cartographer)", fontsize=12)
    ax1.axis('off')
    plt.tight_layout()

    fig2, ax2 = plt.subplots(1, 1, figsize=(8, 8))
    ax2.imshow(cv2.cvtColor(slam_overlay, cv2.COLOR_BGR2RGB))
    ax2.set_title("SLAM Toolbox Average vs Ground Truth\n(Red=GT, Green=SLAM)", fontsize=12)
    ax2.axis('off')
    plt.tight_layout()

    plt.show()

    print("\nSaved files:")
    print(" - pre_drawn_500.png")
    print(" - aligned_drawn.png")
    print(" - cartographer_aligned_*.png (per-run aligned Cartographer maps)")
    print(" - slam_aligned_*.png (per-run aligned SLAM maps)")
    print(" - cartographer_average.png")
    print(" - slam_average.png")
    print(f" - {cart_overlay_name}")
    print(f" - {slam_overlay_name}")
    print(" - results_table.txt")
    print(" - results_table.csv")
    print(" - per_run_errors.csv")

if __name__ == "__main__":
    main()
