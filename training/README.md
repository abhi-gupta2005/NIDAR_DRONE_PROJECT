# NIDAR aerial detector training

Goal: a detector that finds **people and vehicles seen from a drone**. The
COCO-pretrained models used so far were trained on ground-level photos; on
real drone images (VisDrone) stock `yolo26n.pt` finds only **6.4 % of the
people** at confidence 0.25 (AP50 0.17, measured 2026-11-01 on 148 held-out
images — `outputs/training/eval_smoke.md`).

This pipeline does not modify `nidar_v11_windows.py` or `yoloe-26n-seg.pt`.
A trained model is an *additional* option to evaluate, not a silent swap.

## 1. Install (Windows, `.venv` active, in `D:\NIDAR_DRONE_PROJECT`)

```powershell
python -m pip install ultralytics pyyaml pillow
```

## 2. Prepare data

```powershell
# NIDAR 4-class labels: person, two_wheeler, car, large_vehicle  (~1.5 GB download)
python training\prepare_visdrone.py --classes nidar --out datasets\visdrone_nidar
# small CPU subset
python training\prepare_visdrone.py --classes nidar --out datasets\visdrone_mini --subset-train 400 --subset-val 150
```

## 3. Train

| Where | Command | Measured / expected time |
|---|---|---|
| CPU, proof only | `python training\train_aerial.py --data datasets\visdrone_mini\data.yaml --preset smoke` | 3 min on 2 Xeon cores (measured) |
| CPU, small useful run | `python training\train_aerial.py --data datasets\visdrone_mini\data.yaml --preset cpu --epochs 15 --imgsz 640` | ~5.5 min/epoch for 400 images on 2 cores (measured); a 4–8-core laptop is faster |
| GPU (Colab/Kaggle T4) | `python training/train_aerial.py --data datasets/visdrone_nidar/data.yaml --preset full --device 0` | full dataset, 960 px, 100 epochs: several hours on a T4 (not measured here) |

The full 6,471-image dataset at 960 px is **not practical on a CPU** — roughly
16× the images and 2.3× the pixels of the CPU run, i.e. days.

**Colab recipe:** upload the `training/` folder, then in a cell:
`!pip install ultralytics` →
`!python training/prepare_visdrone.py --classes nidar --out datasets/visdrone_nidar` →
`!python training/train_aerial.py --data datasets/visdrone_nidar/data.yaml --preset full --device 0` →
download `runs/nidar_aerial/<run>/weights/best.pt`.

## 4. Evaluate (always on held-out images)

```powershell
python training\evaluate_aerial.py --data datasets\visdrone_mini\data.yaml `
    --models yolo26n.pt runs\nidar_aerial\<run>\weights\best.pt --imgsz 640
```

Reports AP@0.5 and recall@0.25 per class for every model on the same images,
mapping each model's own class names (COCO, VisDrone, NIDAR) onto common
classes.

## 5. Important caveats

- **Domain**: VisDrone is outdoor, 20–100 m altitude. Indoor or low-altitude
  NIDAR flights need their own labelled images; collect frames from the
  drone camera (or the Gazebo relay) and add them to the dataset.
- **Model type**: this trains a closed-set YOLO26 detector. The Windows
  script uses YOLOE open-vocabulary prompts; switching it to a trained model
  is a separate, tested change (class names differ: e.g. `person` only).
- **The smoke split** uses VisDrone *val* images split 400/148 for
  speed. Real evaluation must use the official val split while training on
  the train split.
