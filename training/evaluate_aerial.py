#!/usr/bin/env python3
"""
Compare detectors on the SAME aerial validation images, per NIDAR class.

Different models have different class lists (COCO: person/car/...,
VisDrone: pedestrian/people/car/van/..., NIDAR: person/car/...). This script
maps every model's labels and the ground truth onto common evaluation
classes and computes AP@0.5 and recall@conf for each, with one code path, so
"pretrained vs fine-tuned" is an apples-to-apples comparison.

    python training/evaluate_aerial.py --data datasets/visdrone_mini/data.yaml \
        --models yolo26n.pt runs/nidar_aerial/<run>/weights/best.pt --imgsz 640

Writes a JSON + Markdown report next to --out.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

# label name -> evaluation class
NAME_TO_EVAL = {
    "person": "person", "pedestrian": "person", "people": "person",
    "car": "car", "van": "car",
    "truck": "large_vehicle", "bus": "large_vehicle", "large_vehicle": "large_vehicle",
    "bicycle": "two_wheeler", "motor": "two_wheeler", "motorcycle": "two_wheeler", "two_wheeler": "two_wheeler",
}
EVAL_CLASSES = ["person", "car", "large_vehicle", "two_wheeler"]


def load_yaml(path: Path) -> dict:
    import yaml
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def read_ground_truth(label_file: Path, w: int, h: int, names: dict):
    boxes = []
    if label_file.exists():
        for line in label_file.read_text(encoding="utf-8").split("\n"):
            parts = line.split()
            if len(parts) != 5:
                continue
            cls = NAME_TO_EVAL.get(names[int(parts[0])])
            if cls is None:
                continue
            xc, yc, bw, bh = (float(v) for v in parts[1:])
            boxes.append((cls, (xc - bw / 2) * w, (yc - bh / 2) * h, (xc + bw / 2) * w, (yc + bh / 2) * h))
    return boxes


def iou(a, b):
    ix1, iy1, ix2, iy2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def average_precision(tp: np.ndarray, conf: np.ndarray, n_gt: int) -> float:
    """All-point interpolated AP (VOC2010+/COCO-style at a single IoU)."""
    if n_gt == 0:
        return float("nan")
    if len(tp) == 0:
        return 0.0
    order = np.argsort(-conf)
    tp = tp[order]
    ctp, cfp = np.cumsum(tp), np.cumsum(1 - tp)
    recall = ctp / n_gt
    precision = ctp / np.maximum(ctp + cfp, 1e-9)
    mrec = np.concatenate(([0.0], recall, [1.0]))
    mpre = np.concatenate(([1.0], precision, [0.0]))
    mpre = np.flip(np.maximum.accumulate(np.flip(mpre)))
    idx = np.where(mrec[1:] != mrec[:-1])[0]
    return float(np.sum((mrec[idx + 1] - mrec[idx]) * mpre[idx + 1]))


def evaluate(model_path: str, images, labels_dir: Path, names: dict, imgsz: int, conf: float,
             iou_thr: float = 0.5, device: str = "cpu", recall_conf: float = 0.25) -> dict:
    from ultralytics import YOLO
    from PIL import Image

    model = YOLO(model_path)
    model_names = model.names
    per_class = {c: {"tp": [], "conf": [], "n_gt": 0, "tp_at_conf": 0} for c in EVAL_CLASSES}
    t0 = time.time()
    for img in images:
        with Image.open(img) as im:
            w, h = im.size
        gt = read_ground_truth(labels_dir / (img.stem + ".txt"), w, h, names)
        res = model.predict(str(img), imgsz=imgsz, conf=conf, iou=0.6, device=device, verbose=False,
                            max_det=600)[0]
        preds = []
        if res.boxes is not None and len(res.boxes):
            for (x1, y1, x2, y2), c, k in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy(),
                                               res.boxes.cls.cpu().numpy().astype(int)):
                cls = NAME_TO_EVAL.get(str(model_names[k]).lower())
                if cls is not None:
                    preds.append((cls, float(c), (x1, y1, x2, y2)))
        for cls in EVAL_CLASSES:
            g = [b[1:] for b in gt if b[0] == cls]
            per_class[cls]["n_gt"] += len(g)
            matched = [False] * len(g)
            for _, c, box in sorted([p for p in preds if p[0] == cls], key=lambda p: -p[1]):
                best, best_j = 0.0, -1
                for j, gb in enumerate(g):
                    if not matched[j]:
                        v = iou(box, gb)
                        if v > best:
                            best, best_j = v, j
                hit = best >= iou_thr
                if hit:
                    matched[best_j] = True
                    if c >= recall_conf:
                        per_class[cls]["tp_at_conf"] += 1
                per_class[cls]["tp"].append(1.0 if hit else 0.0)
                per_class[cls]["conf"].append(c)
    elapsed = time.time() - t0
    out = {"model": str(model_path), "images": len(images), "imgsz": imgsz,
           "seconds_per_image": round(elapsed / max(len(images), 1), 4), "classes": {}}
    for cls, d in per_class.items():
        ap = average_precision(np.array(d["tp"]), np.array(d["conf"]), d["n_gt"])
        out["classes"][cls] = {
            "gt_boxes": d["n_gt"],
            "AP50": None if np.isnan(ap) else round(ap, 4),
            f"recall@conf{recall_conf}": round(d["tp_at_conf"] / d["n_gt"], 4) if d["n_gt"] else None,
        }
    return out


def to_markdown(reports) -> str:
    lines = ["| model | class | GT boxes | AP@0.5 | recall@0.25 | s/img |", "|---|---|---|---|---|---|"]
    for r in reports:
        for cls, v in r["classes"].items():
            if v["gt_boxes"] == 0:
                continue
            rec = next(val for k, val in v.items() if k.startswith("recall"))
            lines.append(f"| {Path(r['model']).name if 'runs' not in r['model'] else r['model']} | {cls} | "
                         f"{v['gt_boxes']} | {v['AP50']} | {rec} | {r['seconds_per_image']} |")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True)
    ap.add_argument("--models", nargs="+", required=True)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.01, help="low threshold for AP; recall uses 0.25")
    ap.add_argument("--limit", type=int, default=None, help="evaluate only the first N val images")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--out", default="outputs/training/aerial_eval")
    args = ap.parse_args(argv)

    data = load_yaml(Path(args.data))
    root = Path(data["path"])
    names = data["names"]
    names = {int(k): v for k, v in names.items()} if isinstance(names, dict) else dict(enumerate(names))
    val_images_dir = root / data["val"]
    labels_dir = Path(str(val_images_dir).replace(f"{Path('images')}", "labels", 1))
    if not labels_dir.exists():
        labels_dir = root / "labels" / Path(data["val"]).name
    images = sorted(val_images_dir.glob("*.jpg"))[: args.limit]

    reports = []
    for m in args.models:
        print(f"[eval] {m} on {len(images)} images ...", flush=True)
        r = evaluate(m, images, labels_dir, names, args.imgsz, args.conf, device=args.device)
        reports.append(r)
        print(json.dumps(r, indent=2), flush=True)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix(".json").write_text(json.dumps(reports, indent=2), encoding="utf-8")
    md = to_markdown(reports)
    out.with_suffix(".md").write_text(md + "\n", encoding="utf-8")
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
