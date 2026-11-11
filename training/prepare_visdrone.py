#!/usr/bin/env python3
"""
Prepare the VisDrone2019-DET aerial dataset in YOLO format for NIDAR.

Why VisDrone: the COCO-pretrained YOLO models used so far were trained on
ground-level photos. Seen from a drone, people are small, top-down and
dense, and COCO weights miss many of them. VisDrone is 8,600+ drone images
with boxes for pedestrians, people, cars, vans, trucks, buses, bicycles...

Usage (Windows PowerShell, .venv active, from D:\\NIDAR_DRONE_PROJECT):
    python training\\prepare_visdrone.py --out datasets\\visdrone
    python training\\prepare_visdrone.py --out datasets\\visdrone_mini --subset-train 300 --subset-val 100

Options:
    --classes visdrone   keep the 10 VisDrone classes (default)
    --classes nidar      4 classes useful for NIDAR: person (pedestrian+people),
                         two_wheeler (bicycle+motor), car (car+van), large_vehicle (truck+bus)
    --splits val         only download val (~80 MB) -- enough for evaluation
    --subset-train N     keep a random N training images (CPU smoke runs)

Downloads come from the Ultralytics asset mirror on GitHub
(train 1.4 GB, val 80 MB). Re-running skips anything already present.
"""

from __future__ import annotations

import argparse
import random
import shutil
import sys
import urllib.request
import zipfile
from pathlib import Path

URL = "https://github.com/ultralytics/assets/releases/download/v0.0.0/{name}.zip"
SPLIT_SOURCE = {"train": "VisDrone2019-DET-train", "val": "VisDrone2019-DET-val"}

VISDRONE_NAMES = ["pedestrian", "people", "bicycle", "car", "van", "truck",
                  "tricycle", "awning-tricycle", "bus", "motor"]

# VisDrone category id (1..10) -> NIDAR class id ; None = dropped
NIDAR_NAMES = ["person", "two_wheeler", "car", "large_vehicle"]
VISDRONE_TO_NIDAR = {1: 0, 2: 0, 3: 1, 10: 1, 4: 2, 5: 2, 6: 3, 9: 3, 7: None, 8: None}


def download(name: str, cache: Path) -> Path:
    cache.mkdir(parents=True, exist_ok=True)
    zip_path = cache / f"{name}.zip"
    if zip_path.exists() and zip_path.stat().st_size > 1_000_000:
        print(f"[prepare] using cached {zip_path}")
    else:
        url = URL.format(name=name)
        print(f"[prepare] downloading {url}")
        tmp = zip_path.with_suffix(".part")
        with urllib.request.urlopen(url) as r, tmp.open("wb") as f:
            total = int(r.headers.get("Content-Length", 0))
            done = 0
            while chunk := r.read(1 << 20):
                f.write(chunk)
                done += len(chunk)
                if total:
                    print(f"\r[prepare] {done / 1e6:7.1f} / {total / 1e6:.1f} MB", end="", flush=True)
        print()
        tmp.replace(zip_path)
    extract_dir = cache / name
    if not (extract_dir / "annotations").exists():
        print(f"[prepare] extracting {zip_path.name}")
        with zipfile.ZipFile(zip_path) as z:
            z.extractall(cache)
    return extract_dir


def convert_annotation(ann_file: Path, img_w: int, img_h: int, mode: str):
    """VisDrone row: x,y,w,h,score,category,truncation,occlusion. score==0 -> ignored region."""
    lines = []
    for row in ann_file.read_text(encoding="utf-8").strip().splitlines():
        parts = row.strip().rstrip(",").split(",")
        if len(parts) < 6:
            continue
        x, y, w, h = map(int, parts[:4])
        score, cat = parts[4], int(parts[5])
        if score == "0" or cat == 0 or cat == 11 or w <= 1 or h <= 1:
            continue  # ignored region / "others"
        if mode == "nidar":
            cls = VISDRONE_TO_NIDAR.get(cat)
            if cls is None:
                continue
        else:
            cls = cat - 1
        xc, yc = (x + w / 2) / img_w, (y + h / 2) / img_h
        lines.append(f"{cls} {min(max(xc, 0), 1):.6f} {min(max(yc, 0), 1):.6f} "
                     f"{min(w / img_w, 1):.6f} {min(h / img_h, 1):.6f}")
    return lines


def build_split(src: Path, out: Path, split: str, mode: str, subset: int | None, seed: int) -> int:
    from PIL import Image

    images = sorted((src / "images").glob("*.jpg"))
    if subset:
        random.Random(seed).shuffle(images)
        images = sorted(images[:subset])
    img_dir, lbl_dir = out / "images" / split, out / "labels" / split
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)
    boxes = 0
    for img in images:
        ann = src / "annotations" / (img.stem + ".txt")
        with Image.open(img) as im:
            w, h = im.size
        lines = convert_annotation(ann, w, h, mode) if ann.exists() else []
        boxes += len(lines)
        (lbl_dir / (img.stem + ".txt")).write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        dst = img_dir / img.name
        if not dst.exists():
            shutil.copy2(img, dst)
    print(f"[prepare] {split}: {len(images)} images, {boxes} boxes")
    return len(images)


def write_yaml(out: Path, mode: str, splits) -> Path:
    names = NIDAR_NAMES if mode == "nidar" else VISDRONE_NAMES
    lines = [f"# generated by training/prepare_visdrone.py (classes={mode})",
             f"path: {out.resolve().as_posix()}"]
    lines.append("train: images/train" if "train" in splits else "train: images/val  # no train split prepared")
    lines.append("val: images/val" if "val" in splits else "val: images/train")
    lines.append("names:")
    lines += [f"  {i}: {n}" for i, n in enumerate(names)]
    path = out / "data.yaml"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="datasets/visdrone")
    ap.add_argument("--cache", default="datasets/_downloads")
    ap.add_argument("--classes", choices=["visdrone", "nidar"], default="visdrone")
    ap.add_argument("--splits", nargs="+", choices=["train", "val"], default=["train", "val"])
    ap.add_argument("--subset-train", type=int, default=None)
    ap.add_argument("--subset-val", type=int, default=None)
    ap.add_argument("--source-dir", default=None,
                    help="folder that already contains VisDrone2019-DET-<split>/ (skip download)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    out = Path(args.out)
    for split in args.splits:
        name = SPLIT_SOURCE[split]
        src = Path(args.source_dir) / name if args.source_dir else download(name, Path(args.cache))
        subset = args.subset_train if split == "train" else args.subset_val
        build_split(src, out, split, args.classes, subset, args.seed)
    yaml_path = write_yaml(out, args.classes, args.splits)
    print(f"[prepare] dataset ready: {yaml_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
