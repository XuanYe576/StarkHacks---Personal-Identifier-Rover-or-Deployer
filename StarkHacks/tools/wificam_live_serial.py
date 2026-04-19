#!/usr/bin/env python3
import argparse
import math
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import serial


REPO_ROOT = Path(__file__).resolve().parents[2]
WIFICAM_ROOT = REPO_ROOT / "third_party" / "wificam"
DEFAULT_CKPT = WIFICAM_ROOT / "runs" / "mopoevae_ct" / "bestLoss.ckpt"


def parse_args():
    p = argparse.ArgumentParser(description="Live ESP32 CSI -> WiFiCam deploy runner")
    p.add_argument("--port", required=True, help="Serial port, e.g. /dev/cu.usbserial-XXXX")
    p.add_argument("--baud", type=int, default=921600, help="Serial baud")
    p.add_argument("--port-b", default="", help="Second serial port for dual-node CSI")
    p.add_argument("--baud-b", type=int, default=921600, help="Second serial baud")
    p.add_argument(
        "--fusion",
        default="mean",
        choices=["mean", "diff", "interleave"],
        help="Dual-node fusion: mean/diff/interleave",
    )
    p.add_argument("--bins", type=int, default=64, help="CSI bins from stream")
    p.add_argument("--subcarriers", type=int, default=52, help="WiFiCam feature size")
    p.add_argument("--window-size", type=int, default=151, help="CSI packet window for model")
    p.add_argument("--infer-every", type=int, default=3, help="Run model every N frames")
    p.add_argument("--zdim", type=int, default=128)
    p.add_argument("--aggregate", default="concat", choices=["concat", "gaussian", "uniform"])
    p.add_argument("--checkpoint", default=str(DEFAULT_CKPT), help="WiFiCam .ckpt path")
    p.add_argument("--device", default="cpu", help="cpu or cuda")
    p.add_argument("--disable-model", action="store_true", help="Force heatmap-only mode")
    return p.parse_args()


def encode_time(x, L, window_size):
    window_size *= 3
    frequencies = np.array([2 ** i for i in range(L)], dtype=np.float32)
    x = x / window_size
    pos_enc = np.concatenate(
        [
            np.sin(frequencies[:, None] * np.pi * x),
            np.cos(frequencies[:, None] * np.pi * x),
        ],
        axis=0,
    )
    return pos_enc


def parse_csi_line(line):
    line = line.strip()
    if not line:
        return None
    parts = line.split(",")
    if len(parts) < 6:
        return None

    if parts[0] == "CSIv1":
        try:
            seq = int(parts[1])
            rssi = int(parts[2])
            raw_len = int(parts[3])
            bins = int(parts[4])
        except ValueError:
            return None
        if bins <= 0:
            return None
        expected_len = 7 + (2 * bins)
        if len(parts) != expected_len:
            return None
        if parts[5] != "A":
            return None
        phase_marker = 6 + bins
        if parts[phase_marker] != "P":
            return None
        try:
            amps = [int(v) for v in parts[6:phase_marker]]
            phases = [int(v) for v in parts[phase_marker + 1 :]]
        except ValueError:
            return None
        if len(amps) != bins or len(phases) != bins:
            return None
        return {"seq": seq, "rssi": rssi, "raw_len": raw_len, "bins": bins, "amp": amps, "phase": phases}

    if parts[0] == "CSI":
        try:
            seq = int(parts[1])
            rssi = int(parts[2])
            raw_len = int(parts[3])
            bins = int(parts[4])
            amps = [int(v) for v in parts[5:]]
        except ValueError:
            return None
        if bins <= 0 or len(amps) == 0:
            return None
        return {"seq": seq, "rssi": rssi, "raw_len": raw_len, "bins": bins, "amp": amps, "phase": [0] * len(amps)}

    return None


def to_52_subcarriers(amp_vec, n_sub=52):
    x = np.asarray(amp_vec, dtype=np.float32)
    if x.size >= n_sub:
        start = (x.size - n_sub) // 2
        x = x[start : start + n_sub]
    else:
        out = np.zeros((n_sub,), dtype=np.float32)
        out[: x.size] = x
        x = out
    return x


def build_heatmap(window_mat):
    # window_mat: [subcarriers, time]
    if window_mat.size == 0:
        return np.zeros((128, 128, 3), dtype=np.uint8)
    m = window_mat.copy()
    lo, hi = np.percentile(m, 2), np.percentile(m, 98)
    if hi <= lo:
        hi = lo + 1.0
    m = np.clip((m - lo) / (hi - lo), 0.0, 1.0)
    m = (m * 255).astype(np.uint8)
    m = cv2.resize(m, (256, 128), interpolation=cv2.INTER_LINEAR)
    return cv2.applyColorMap(m, cv2.COLORMAP_INFERNO)


def try_load_model(args):
    if args.disable_model:
        return None, None, None
    ckpt = Path(args.checkpoint)
    if not ckpt.exists():
        print(f"[INFO] checkpoint not found: {ckpt} -> heatmap-only mode")
        return None, None, None

    try:
        import torch
    except Exception as exc:
        print(f"[INFO] torch unavailable ({exc}) -> heatmap-only mode")
        return None, None, None

    import sys

    sys.path.insert(0, str(WIFICAM_ROOT))
    try:
        from mopoevae import MoPoEVAE
    except Exception as exc:
        print(f"[INFO] cannot import WiFiCam model ({exc}) -> heatmap-only mode")
        return None, None, None

    L = int(math.ceil(math.log(args.window_size, 2)))
    try:
        model = MoPoEVAE.load_from_checkpoint(
            str(ckpt),
            weight_ll=True,
            lr=1e-3,
            sequence_length=args.window_size,
            z_dim=args.zdim,
            frequence_L=L,
            aggregate_method=args.aggregate,
            map_location=args.device,
            imgMean=np.array([0.5, 0.5, 0.5], dtype=np.float32),
            imgStd=np.array([0.25, 0.25, 0.25], dtype=np.float32),
            log=False,
        )
        model.to(args.device)
        model.eval()
        print(f"[INFO] WiFiCam model loaded: {ckpt}")
        return model, torch, L
    except Exception as exc:
        print(f"[INFO] failed loading model ({exc}) -> heatmap-only mode")
        return None, None, None


def run_inference(model, torch, L, spectro_np, args):
    # spectro_np shape [subcarriers, window_size]
    spec = torch.from_numpy(spectro_np.astype(np.float32)).unsqueeze(0).unsqueeze(0).to(args.device)

    tenc_base = encode_time(np.array([0], dtype=np.float32), L, args.window_size)
    spect_t = torch.tensor(tenc_base, dtype=torch.float32).unsqueeze(0)  # [1,2L,1]
    image_t = torch.tensor(tenc_base, dtype=torch.float32).unsqueeze(0)  # [1,2L,1]
    tenc = torch.cat((spect_t, image_t), dim=2).unsqueeze(0).to(args.device)  # [1,1,2L,2]

    spectro = (tenc, spec)
    # second modality placeholder (unused when subset=[0])
    image_dummy = (tenc, torch.zeros((1, 3, 128, 128), dtype=torch.float32, device=args.device))

    with torch.no_grad():
        recon = model.decode(model.encode_subset([spectro, image_dummy], [0]))[1][0][1]

    # denorm fallback
    img = recon[0].detach().cpu().permute(1, 2, 0).numpy()
    img = np.clip((img * 0.25 + 0.5), 0.0, 1.0)
    # Stretch contrast for display so non-trivial predictions are not visually black.
    lo = float(np.percentile(img, 1))
    hi = float(np.percentile(img, 99))
    if hi > lo + 1e-6:
        img = np.clip((img - lo) / (hi - lo), 0.0, 1.0)
    img = (img * 255).astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


def fuse_specs(spec_a, spec_b, mode):
    # spec_* shape: [subcarriers, time]
    if spec_b is None:
        return spec_a
    if mode == "diff":
        return np.abs(spec_a - spec_b)
    if mode == "interleave":
        out = np.zeros_like(spec_a)
        out[0::2, :] = spec_a[0::2, :]
        out[1::2, :] = spec_b[1::2, :]
        return out
    return 0.5 * (spec_a + spec_b)


def main():
    args = parse_args()
    ser_a = serial.Serial(args.port, args.baud, timeout=0.01)
    ser_b = serial.Serial(args.port_b, args.baud_b, timeout=0.01) if args.port_b else None

    model, torch, L = try_load_model(args)

    win_a = deque(maxlen=args.window_size)
    win_b = deque(maxlen=args.window_size)
    last_pred = np.zeros((128, 128, 3), dtype=np.uint8)
    model_err = ""
    frame_i = 0
    latest_seq_a = 0
    latest_seq_b = 0
    latest_rssi_a = -127
    latest_rssi_b = -127

    try:
        while True:
            line_a = ser_a.readline().decode("utf-8", errors="ignore").strip()
            parsed_a = parse_csi_line(line_a) if line_a else None
            if parsed_a is not None:
                latest_seq_a = parsed_a["seq"]
                latest_rssi_a = parsed_a["rssi"]
                amp_a = np.array(parsed_a["amp"][: args.bins], dtype=np.float32)
                amp_a = np.clip(amp_a, 0.0, 255.0)
                win_a.append(to_52_subcarriers(amp_a, args.subcarriers))

            parsed_b = None
            if ser_b is not None:
                line_b = ser_b.readline().decode("utf-8", errors="ignore").strip()
                parsed_b = parse_csi_line(line_b) if line_b else None
                if parsed_b is not None:
                    latest_seq_b = parsed_b["seq"]
                    latest_rssi_b = parsed_b["rssi"]
                    amp_b = np.array(parsed_b["amp"][: args.bins], dtype=np.float32)
                    amp_b = np.clip(amp_b, 0.0, 255.0)
                    win_b.append(to_52_subcarriers(amp_b, args.subcarriers))

            need_b = ser_b is not None
            ready_a = len(win_a) >= args.window_size
            ready_b = (len(win_b) >= args.window_size) if need_b else True
            if not (ready_a and ready_b):
                canvas = np.zeros((220, 860, 3), dtype=np.uint8)
                if need_b:
                    msg = f"Buffering A:{len(win_a)}/{args.window_size} B:{len(win_b)}/{args.window_size}"
                else:
                    msg = f"Buffering CSI window: {len(win_a)}/{args.window_size}"
                cv2.putText(canvas, msg, (12, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(canvas, f"A seq={latest_seq_a} rssi={latest_rssi_a}", (12, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (200, 200, 200), 2, cv2.LINE_AA)
                if need_b:
                    cv2.putText(canvas, f"B seq={latest_seq_b} rssi={latest_rssi_b}", (12, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (200, 200, 200), 2, cv2.LINE_AA)
                cv2.imshow("WiFiCam Live Deploy", canvas)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
                continue

            spec_a = np.stack(win_a, axis=0).T  # [subcarriers, time]
            spec_b = np.stack(win_b, axis=0).T if need_b else None
            spec_fused = fuse_specs(spec_a, spec_b, args.fusion)

            heat_a = build_heatmap(spec_a)
            heat_b = build_heatmap(spec_b) if spec_b is not None else np.zeros_like(heat_a)
            heat_fused = build_heatmap(spec_fused)

            frame_i += 1
            if model is not None and (frame_i % max(1, args.infer_every) == 0):
                try:
                    last_pred = run_inference(model, torch, L, spec_fused, args)
                    model_err = ""
                except Exception as exc:
                    model_err = str(exc)[:120]

            if model is None:
                # Explicit fallback visualization instead of black panel.
                pred = cv2.resize(heat_fused, (256, 128), interpolation=cv2.INTER_LINEAR)
            else:
                pred = cv2.resize(last_pred, (256, 128), interpolation=cv2.INTER_LINEAR)
            left = cv2.resize(heat_a, (200, 100), interpolation=cv2.INTER_LINEAR)
            mid = cv2.resize(heat_b, (200, 100), interpolation=cv2.INTER_LINEAR)
            fused = cv2.resize(heat_fused, (200, 100), interpolation=cv2.INTER_LINEAR)
            right = cv2.resize(pred, (200, 100), interpolation=cv2.INTER_LINEAR)

            panel = np.zeros((190, 860, 3), dtype=np.uint8)
            panel[52:152, 12:212] = left
            panel[52:152, 224:424] = mid
            panel[52:152, 436:636] = fused
            panel[52:152, 648:848] = right

            model_tag = "wificam-model" if model is not None else "heatmap-only"
            if need_b:
                status = f"A seq={latest_seq_a} rssi={latest_rssi_a} | B seq={latest_seq_b} rssi={latest_rssi_b} | fusion={args.fusion} | {model_tag}"
            else:
                status = f"A seq={latest_seq_a} rssi={latest_rssi_a} | {model_tag}"
            cv2.putText(panel, status, (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 230, 230), 1, cv2.LINE_AA)
            cv2.putText(panel, "CSI A", (12, 176), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (180, 180, 180), 1, cv2.LINE_AA)
            cv2.putText(panel, "CSI B", (224, 176), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (180, 180, 180), 1, cv2.LINE_AA)
            cv2.putText(panel, "CSI fused", (436, 176), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (180, 180, 180), 1, cv2.LINE_AA)
            cv2.putText(panel, "WiFiCam output", (648, 176), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (180, 180, 180), 1, cv2.LINE_AA)
            if model is None:
                cv2.putText(panel, "MODEL NOT LOADED (fallback)", (648, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.41, (120, 180, 255), 1, cv2.LINE_AA)
            if model_err:
                cv2.putText(panel, f"Model error: {model_err[:58]}", (12, 164), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 180, 255), 1, cv2.LINE_AA)

            cv2.imshow("WiFiCam Live Deploy", panel)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    finally:
        ser_a.close()
        if ser_b is not None:
            ser_b.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
