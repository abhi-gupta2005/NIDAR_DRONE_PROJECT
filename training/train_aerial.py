#!/usr/bin/env python3
"""
Fine-tune a YOLO detector on aerial (VisDrone) imagery for NIDAR.

CPU-only laptops: a full VisDrone training run is NOT practical on CPU
(measured on this project's smoke test -- see training/README.md). Use the
"smoke" preset to prove the pipeline locally, then run the "full" preset on
a GPU (Google Colab / Kaggle free tier) with the exact same script.

Examples:
    # 1) prove the pipeline end to end on CPU (minutes)
    python training/train_aerial.py --data datasets/visdrone_mini/data.yaml --preset smoke

    # 2) real training on a GPU machine / Colab
    python training/train_aerial.py --data datasets/visdrone/data.yaml --preset full --device 0

Output: runs/nidar_aerial/<name>/weights/best.pt  (+ results.csv, plots)
The script never touches the existing yoloe-26n-seg.pt or nidar_v11_windows.py.
"""

from __future__ import annotations

import argparse
import json
import platform
import sys
import time
from pathlib import Path

PRESETS = {
    # tiny, fast, CPU: proves data -> train -> weights -> eval works
    "smoke": dict(epochs=2, imgsz=320, batch=8, workers=0, fraction=1.0, patience=0, mosaic=0.0,
                  plots=False, cache=False),
    # CPU but meaningful: a few hours on a laptop with the NIDAR 4-class subset
    "cpu": dict(epochs=20, imgsz=480, batch=8, workers=2, fraction=1.0, patience=10, cache=False),
    # GPU: the real run. VisDrone objects are tiny -> large input size matters more than epochs.
    "full": dict(epochs=100, imgsz=960, batch=16, workers=8, fraction=1.0, patience=25, cache="ram",
                 close_mosaic=10),
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True, help="data.yaml from prepare_visdrone.py")
    ap.add_argument("--model", default="yolo26n.pt", help="base weights (COCO-pretrained), downloaded if missing")
    ap.add_argument("--preset", choices=sorted(PRESETS), default="smoke")
    ap.add_argument("--device", default="cpu", help="'cpu', '0' for first GPU")
    ap.add_argument("--epochs", type=int)
    ap.add_argument("--imgsz", type=int)
    ap.add_argument("--batch", type=int)
    ap.add_argument("--project", default="runs/nidar_aerial")
    ap.add_argument("--name", default=None)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    from ultralytics import YOLO
    import torch

    cfg = dict(PRESETS[args.preset])
    for key in ("epochs", "imgsz", "batch"):
        if getattr(args, key) is not None:
            cfg[key] = getattr(args, key)
    name = args.name or f"{Path(args.model).stem}_{args.preset}_{time.strftime('%Y%m%d_%H%M%S')}"

    print("=" * 70)
    print("NIDAR AERIAL DETECTOR TRAINING")
    print("=" * 70)
    print(f"data    : {args.data}")
    print(f"model   : {args.model}")
    print(f"preset  : {args.preset} {cfg}")
    print(f"device  : {args.device} | torch {torch.__version__} | cuda {torch.cuda.is_available()}")
    print(f"host    : {platform.platform()} | {platform.processor() or platform.machine()}")
    print("=" * 70, flush=True)
    if args.device == "cpu" and args.preset == "full":
        print("[WARN] 'full' on CPU will take days. Use --preset cpu, or a GPU (see training/README.md).")

    model = YOLO(args.model)
    t0 = time.time()
    model.train(data=args.data, device=args.device, project=str(Path(args.project).resolve()), name=name,
                seed=args.seed, deterministic=True, exist_ok=False, verbose=True, **cfg)
    elapsed = time.time() - t0

    # Use the directory Ultralytics really wrote to (it may differ from project/name).
    run_dir = Path(model.trainer.save_dir)
    best = run_dir / "weights" / "best.pt"
    summary = {"run_dir": str(run_dir), "best_weights": str(best), "train_seconds": round(elapsed, 1),
               "preset": args.preset, "config": cfg, "data": args.data, "base_model": args.model,
               "device": args.device}
    (run_dir / "nidar_training_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if best.exists() else 1


if __name__ == "__main__":
    sys.exit(main())
