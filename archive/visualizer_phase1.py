"""
Rover Vision Visualizer
Supports two modes matching the Pico firmware:
  --mode jpeg     Reads JPEG frames (MODE_STREAM_JPEG on Pico)
  --mode rgb      Reads RGB565 frames + detection results (MODE_RGB_DEBUG on Pico)

Usage:
  python visualizer.py --port COM6 --mode jpeg
  python visualizer.py --port COM6 --mode rgb
"""

import serial
import serial.tools.list_ports
import numpy as np
import cv2
import argparse
import time
import struct


def find_ports():
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]


# ═════════════════════════════════════════════════════════════════════════════
#  JPEG MODE
# ═════════════════════════════════════════════════════════════════════════════

# HSV red detection (for PC-side detection in JPEG mode)
RED_LOW1  = np.array([0, 100, 80])
RED_HIGH1 = np.array([10, 255, 255])
RED_LOW2  = np.array([160, 100, 80])
RED_HIGH2 = np.array([180, 255, 255])
BLOB_MIN_SIZE = 50
BLOB_MAX_SIZE = 50000


def read_jpeg_frame(ser):
    prev_byte = b''
    timeout_start = time.time()
    found = False
    while time.time() - timeout_start < 5.0:
        byte = ser.read(1)
        if not byte:
            continue
        if prev_byte == b'\xff' and byte == b'\xaa':
            found = True
            break
        prev_byte = byte

    if not found:
        return None

    size_bytes = ser.read(4)
    if len(size_bytes) < 4:
        return None

    jpeg_len = struct.unpack('<I', size_bytes)[0]
    if jpeg_len < 100 or jpeg_len > 200000:
        return None

    jpeg_data = b''
    remaining = jpeg_len
    read_start = time.time()
    while remaining > 0:
        if time.time() - read_start > 5.0:
            return None
        chunk = ser.read(min(remaining, 4096))
        if not chunk:
            continue
        jpeg_data += chunk
        remaining -= len(chunk)

    ser.read(2)  # footer

    jpg_array = np.frombuffer(jpeg_data, dtype=np.uint8)
    frame = cv2.imdecode(jpg_array, cv2.IMREAD_COLOR)
    return frame


def detect_red_ball(bgr_frame):
    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, RED_LOW1, RED_HIGH1)
    mask2 = cv2.inRange(hsv, RED_LOW2, RED_HIGH2)
    mask = mask1 | mask2
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if BLOB_MIN_SIZE < area < BLOB_MAX_SIZE:
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                blobs.append((cx, cy, area, cnt))
    return blobs, mask


def run_jpeg_mode(ser, scale):
    print("[MODE] JPEG streaming — PC-side detection")
    cv2.namedWindow('Rover Vision', cv2.WINDOW_NORMAL)

    frame_count = 0
    fps_start = time.time()
    fps = 0.0

    while True:
        frame = read_jpeg_frame(ser)
        if frame is None:
            print("[WARN] Frame read failed, retrying...")
            continue

        h, w = frame.shape[:2]
        display = frame.copy()

        blobs, mask = detect_red_ball(frame)
        target_found = len(blobs) > 0
        target_x = 0.0
        target_size = 0

        if target_found:
            best = max(blobs, key=lambda b: b[2])
            cx, cy, area, cnt = best
            target_x = (cx - w / 2) / (w / 2)
            target_size = int(area)
            cv2.drawContours(display, [cnt], -1, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 6, (0, 0, 255), -1)
            cv2.line(display, (cx, 0), (cx, h), (0, 0, 255), 1)
            cv2.line(display, (0, cy), (w, cy), (0, 0, 255), 1)

        cv2.line(display, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)

        display_large = cv2.resize(display, (w * scale, h * scale),
                                   interpolation=cv2.INTER_LINEAR)

        hud_h = 80
        hud = np.zeros((hud_h, w * scale, 3), dtype=np.uint8)
        color = (0, 255, 0) if target_found else (0, 0, 255)
        status = "RED BALL FOUND" if target_found else "NO RED BALL"
        cv2.putText(hud, f"Status: {status}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(hud, f"Target X: {target_x:+.2f}  Size: {target_size}",
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(hud, f"FPS: {fps:.1f}  Frame: {frame_count}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        combined = np.vstack([display_large, hud])
        cv2.imshow('Rover Vision', combined)

        frame_count += 1
        if frame_count % 10 == 0:
            elapsed = time.time() - fps_start
            fps = 10.0 / elapsed if elapsed > 0 else 0
            fps_start = time.time()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite(f"frame_{frame_count:04d}.png", display_large)
            print(f"[SAVED] frame_{frame_count:04d}.png")


# ═════════════════════════════════════════════════════════════════════════════
#  RGB565 DEBUG MODE
# ═════════════════════════════════════════════════════════════════════════════

def read_rgb_debug_frame(ser):
    """
    Read one RGB565 debug packet from Pico.
    Protocol:
      [0xFF 0xCC] [w] [h] [found] [tx(2)] [ty(2)] [size(2)]
      [capture_us(2)] [spi_us(2)] [detect_us(2)] [total_us(2)] [cpu_free(1)]
      [RGB565 data: w*h*2 bytes] [0xFF 0xDD]
    """
    prev_byte = b''
    timeout_start = time.time()
    found_marker = False
    while time.time() - timeout_start < 5.0:
        byte = ser.read(1)
        if not byte:
            continue
        if prev_byte == b'\xff' and byte == b'\xcc':
            found_marker = True
            break
        prev_byte = byte

    if not found_marker:
        return None

    # Read header: w, h, found, tx(2), ty(2), size(2),
    #              capture_us(2), spi_us(2), detect_us(2), total_us(2), cpu_free(1)
    #              = 9 + 9 = 18 bytes
    hdr = ser.read(18)
    if len(hdr) < 18:
        return None

    w = hdr[0]
    h = hdr[1]
    target_found = hdr[2] == 1
    target_x = struct.unpack('<h', hdr[3:5])[0] / 100.0
    target_y = struct.unpack('<h', hdr[5:7])[0] / 100.0
    target_size = struct.unpack('<H', hdr[7:9])[0]

    capture_us  = struct.unpack('<H', hdr[9:11])[0]
    spi_read_us = struct.unpack('<H', hdr[11:13])[0]
    detect_us   = struct.unpack('<H', hdr[13:15])[0]
    total_us    = struct.unpack('<H', hdr[15:17])[0]
    cpu_free    = hdr[17]

    # Read RGB565 frame data
    frame_bytes = w * h * 2
    data = b''
    remaining = frame_bytes
    read_start = time.time()
    while remaining > 0:
        if time.time() - read_start > 5.0:
            return None
        chunk = ser.read(min(remaining, 4096))
        if not chunk:
            continue
        data += chunk
        remaining -= len(chunk)

    ser.read(2)  # footer

    # Convert RGB565 to BGR for OpenCV
    raw = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 2))
    hi = raw[:, :, 0].astype(np.uint16)
    lo = raw[:, :, 1].astype(np.uint16)

    r = ((hi & 0xF8)).astype(np.uint8)
    g = (((hi & 0x07) << 5) | ((lo & 0xE0) >> 3)).astype(np.uint8)
    b = ((lo & 0x1F) << 3).astype(np.uint8)

    bgr = np.stack([b, g, r], axis=2)

    perf = {
        'capture_us': capture_us,
        'spi_read_us': spi_read_us,
        'detect_us': detect_us,
        'total_us': total_us,
        'cpu_free': cpu_free
    }

    return bgr, target_found, target_x, target_y, target_size, perf


def run_rgb_debug_mode(ser, scale):
    print("[MODE] RGB565 debug — Pico-side detection + PC display + performance")
    cv2.namedWindow('Rover Vision (Pico Detection)', cv2.WINDOW_NORMAL)

    frame_count = 0
    fps_start = time.time()
    fps = 0.0

    while True:
        result = read_rgb_debug_frame(ser)
        if result is None:
            print("[WARN] Frame read failed, retrying...")
            continue

        frame, target_found, target_x, target_y, target_size, perf = result
        h, w = frame.shape[:2]

        display = frame.copy()

        # Draw detection results (from Pico)
        if target_found:
            cx = int((target_x + 1.0) / 2.0 * w)
            cy = int((target_y + 1.0) / 2.0 * h)
            cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(display, (cx, 0), (cx, h), (0, 0, 255), 1)
            cv2.line(display, (0, cy), (w, cy), (0, 0, 255), 1)

        # Center crosshair
        cv2.line(display, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        cv2.line(display, (0, h // 2), (w, h // 2), (255, 255, 0), 1)

        display_large = cv2.resize(display, (w * scale, h * scale),
                                   interpolation=cv2.INTER_NEAREST)

        # ── HUD with performance metrics ──────────────────────────────────
        hud_h = 160
        hud = np.zeros((hud_h, w * scale, 3), dtype=np.uint8)

        # Detection status
        color = (0, 255, 0) if target_found else (0, 0, 255)
        status = "PICO: RED FOUND" if target_found else "PICO: NO TARGET"
        cv2.putText(hud, f"Status: {status}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(hud, f"Target X: {target_x:+.2f}  Y: {target_y:+.2f}  Pixels: {target_size}",
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Performance breakdown
        cv2.putText(hud, "--- Pico Performance ---", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)

        cap_ms = perf['capture_us'] / 1000.0
        spi_ms = perf['spi_read_us'] / 1000.0
        det_ms = perf['detect_us'] / 1000.0
        tot_ms = perf['total_us'] / 1000.0
        cpu_free = perf['cpu_free']

        cv2.putText(hud, f"Capture: {cap_ms:.1f}ms  SPI Read: {spi_ms:.1f}ms  Detect: {det_ms:.2f}ms",
                    (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.putText(hud, f"Total loop: {tot_ms:.1f}ms  ({1000.0/tot_ms:.0f} FPS potential)" if tot_ms > 0 else "Total: ---",
                    (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # CPU free bar
        bar_x, bar_y, bar_w, bar_h = 10, 115, 300, 16
        cv2.rectangle(hud, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (80, 80, 80), -1)
        used_w = int(bar_w * (100 - cpu_free) / 100)
        bar_color = (0, 255, 0) if cpu_free > 50 else (0, 200, 255) if cpu_free > 20 else (0, 0, 255)
        cv2.rectangle(hud, (bar_x, bar_y), (bar_x + used_w, bar_y + bar_h), bar_color, -1)
        cv2.putText(hud, f"CPU: {100 - cpu_free}% used  |  {cpu_free}% free for ToF + motors",
                    (10, 148), cv2.FONT_HERSHEY_SIMPLEX, 0.42, bar_color, 1)

        combined = np.vstack([display_large, hud])
        cv2.imshow('Rover Vision (Pico Detection)', combined)

        frame_count += 1
        if frame_count % 10 == 0:
            elapsed = time.time() - fps_start
            fps = 10.0 / elapsed if elapsed > 0 else 0
            fps_start = time.time()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite(f"frame_{frame_count:04d}.png", display_large)
            print(f"[SAVED] frame_{frame_count:04d}.png")


# ═════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='Rover Vision Visualizer')
    parser.add_argument('--port', type=str, default=None)
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--scale', type=int, default=4,
                        help='Display scale (default 4 for 128x128)')
    parser.add_argument('--mode', type=str, default='rgb',
                        choices=['jpeg', 'rgb'],
                        help='jpeg = JPEG streaming, rgb = RGB565 debug')
    args = parser.parse_args()

    port = args.port
    if port is None:
        ports = find_ports()
        if not ports:
            print("[ERROR] No serial ports found.")
            return
        port = ports[0]
        print(f"[INFO] Auto-selected port: {port}")

    print(f"[INFO] Connecting to {port} at {args.baud} baud...")
    print(f"[INFO] Mode: {args.mode}")
    print(f"[INFO] Scale: {args.scale}x")
    print(f"[INFO] Press 'q' to quit, 's' to save frame")
    print()

    try:
        ser = serial.Serial(port, args.baud, timeout=2.0)
        print(f"[OK]   Connected to {port}")
    except Exception as e:
        print(f"[ERROR] Could not open {port}: {e}")
        return

    time.sleep(1.0)
    ser.reset_input_buffer()
    print("[INFO] Waiting for frames...")

    if args.mode == 'jpeg':
        run_jpeg_mode(ser, args.scale)
    else:
        run_rgb_debug_mode(ser, args.scale)

    ser.close()
    cv2.destroyAllWindows()
    print("[DONE] Visualizer closed.")


if __name__ == '__main__':
    main()