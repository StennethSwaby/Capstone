import cv2
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# 1) I/O helpers
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

# ============================================================
# 2) Alignment helpers (same logic, now operating on 500×500)
# ============================================================

def binarize(img):
    # Otsu threshold; make obstacles white (invert if needed)
    _, th = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if np.sum(th == 255) > np.sum(th == 0):
        th = cv2.bitwise_not(th)
    return th

def major_axis_angle(binary):
    # Use largest contour's min-area rectangle to get "long-side" angle
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

def center_image(binary):
    # translate centroid to image center
    h, w = binary.shape[:2]
    M = cv2.moments(binary, binaryImage=True)
    if M['m00'] == 0:
        return binary
    cx = M['m10'] / M['m00']
    cy = M['m01'] / M['m00']
    tx = (w/2) - cx
    ty = (h/2) - cy
    T = np.float32([[1, 0, tx], [0, 1, ty]])
    return cv2.warpAffine(binary, T, (w, h), flags=cv2.INTER_NEAREST, borderValue=255)

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
    a_f = a.astype(np.float32) / 255.0
    b_f = b.astype(np.float32) / 255.0
    num = np.sum((a_f - a_f.mean()) * (b_f - b_f.mean()))
    den = np.sqrt(np.sum((a_f - a_f.mean())**2) * np.sum((b_f - b_f.mean())**2)) + 1e-9
    return num / den

def best_flip_to_match(img_bin, ref_bin):
    # Try no flip, H, V, HV; pick best correlation vs reference
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

# ============================================================
# 3) Main (interactive)
# ============================================================

print("  Map Comparison & Alignment Tool")
print("----------------------------------------")
cart_name = input("Enter the filename for the Cartographer map: ").strip()
slam_name = input("Enter the filename for the SLAM Toolbox map: ").strip()
drawn_name = input("Enter the filename for the Drawn (manual) map: ").strip()

# Load and force all to 500×500 centered canvases first
cart_raw = load_gray(cart_name)
slam_raw = load_gray(slam_name)
drawn_raw = load_gray(drawn_name)

cart_500 = fit_and_center_to_canvas(cart_raw, (500, 500), bg=255)
slam_500 = fit_and_center_to_canvas(slam_raw, (500, 500), bg=255)
drawn_500 = fit_and_center_to_canvas(drawn_raw, (500, 500), bg=255)

# Save these preprocessed versions (handy for debugging)
cv2.imwrite("pre_cartographer_500.png", cart_500)
cv2.imwrite("pre_slam_500.png", slam_500)
cv2.imwrite("pre_drawn_500.png", drawn_500)

# Normalize orientation + center (still in 500×500 space)
cart_rot, cart_bin, cart_ang = normalize(cart_500)
slam_rot, slam_bin, slam_ang = normalize(slam_500)
drawn_rot, drawn_bin, drawn_ang = normalize(drawn_500)

# Flip SLAM & Drawn to best match Cartographer as reference
_, slam_flip, slam_score = best_flip_to_match(slam_bin, cart_bin)
_, draw_flip, draw_score = best_flip_to_match(drawn_bin, cart_bin)

slam_aligned = apply_flip(slam_rot, slam_flip)
draw_aligned = apply_flip(drawn_rot, draw_flip)

# Save outputs
cv2.imwrite("aligned_cartographer.png", cart_rot)
cv2.imwrite("aligned_slam.png", slam_aligned)
cv2.imwrite("aligned_drawn.png", draw_aligned)

# Build overlay preview
def colorize(gray, channel):
    c = np.zeros((gray.shape[0], gray.shape[1], 3), dtype=np.uint8)
    c[:, :, channel] = gray
    return c

overlay = cv2.addWeighted(colorize(cart_rot, 0), 0.4, colorize(slam_aligned, 1), 0.4, 0)
overlay = cv2.addWeighted(overlay, 1.0, colorize(draw_aligned, 2), 0.4, 0)
cv2.imwrite("aligned_overlay_preview.png", cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))

# Show panels
fig, axs = plt.subplots(2, 3, figsize=(12, 8))
axs = axs.ravel()
axs[0].imshow(cart_500, cmap='gray');  axs[0].set_title("Cartographer (500×500)"); axs[0].axis('off')
axs[1].imshow(slam_500, cmap='gray');  axs[1].set_title("SLAM (500×500)");        axs[1].axis('off')
axs[2].imshow(drawn_500, cmap='gray'); axs[2].set_title("Drawn (500×500)");       axs[2].axis('off')
axs[3].imshow(cart_rot, cmap='gray');  axs[3].set_title(f"Cart aligned (rot {cart_ang:.1f}°)"); axs[3].axis('off')
axs[4].imshow(slam_aligned, cmap='gray'); axs[4].set_title(f"SLAM aligned ({slam_flip}, corr {slam_score:.2f})"); axs[4].axis('off')
axs[5].imshow(draw_aligned, cmap='gray'); axs[5].set_title(f"Drawn aligned ({draw_flip}, corr {draw_score:.2f})"); axs[5].axis('off')
plt.tight_layout()
plt.show()

# Show final overlay
plt.figure(figsize=(6,6))
plt.title("Aligned Overlay (Blue=Cart, Green=SLAM, Red=Drawn)")
plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()

print("\n Saved:")
print(" - pre_cartographer_500.png")
print(" - pre_slam_500.png")
print(" - pre_drawn_500.png")
print(" - aligned_cartographer.png")
print(" - aligned_slam.png")
print(" - aligned_drawn.png")
print(" - aligned_overlay_preview.png")
