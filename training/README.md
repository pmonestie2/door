# Training on the Mac

Training code lives here; image datasets remain in
`~/Documents/code/CATDOOR_TRAINING/{samples,validation}/{obstructed,unobstructed}`.
The existing Mac virtual environment remains in `CATDOOR_TRAINING/.venv`.
No training photos, virtual environments, backups, or generated runs are committed.
The deployed weights and report are in `../fat_cat_model/`.

From the Mac:

```sh
cd ~/Documents/PI3/door/training
~/Documents/code/CATDOOR_TRAINING/.venv/bin/python improve_margin.py
```

Results go to a new timestamped directory under
`~/Documents/code/CATDOOR_TRAINING/training-runs/`. Override locations with
`--data-root`, `--baseline`, and `--output`. An existing output directory is rejected
so a previous run cannot be overwritten accidentally. The baseline defaults to
the repository's deployed `fat_cat_model/model.pt`; training never overwrites it.

Check a generated run:

```sh
~/Documents/code/CATDOOR_TRAINING/.venv/bin/python check_margin_robustness.py \
  ~/Documents/code/CATDOOR_TRAINING/training-runs/RUN_DIRECTORY
```

`requirements.txt` lists dependencies; `requirements-lock.txt` records the original
Mac environment. Rebuild the environment using `python3 -m venv` and
`python -m pip install -r requirements.txt` if necessary.

## Model selection

EXIF orientation → RGB → 384×384 letterbox → ImageNet normalization → frozen
MobileNetV3-Small → 2×2 average pooling → linear two-class head. Only the head is
trained. The search compares 24 combinations of initialization seed, learning rate,
weight decay, and label smoothing, examining each full 600-epoch curve.

Candidates first minimize missed obstructions, false alarms, and training errors,
then maximize the validation logit gap divided by the head's weight-difference norm.
Uniform logit scaling cannot improve this metric. The checkpoint stores its threshold
and preprocessing settings; inference requires no network access.

Validation is used repeatedly for model and threshold selection, not independent
testing. The deployed model passes the 53 original validation images, with highest
clear score 0.582405 and lowest obstructed score 0.934820, threshold 0.817265.
One darkened obstruction was missed in the brightness/contrast/JPEG check.
Scores are uncalibrated and do not establish depth recognition or reliable future
performance. Current metrics and dataset filenames are in `../fat_cat_model/model-report.json`.

`classifier.py train` retains the original simple RGB/average-pooling baseline;
it does not reproduce the selected model. `test_classifier.py` covers preprocessing,
frozen features, and checkpoint loading. Training and validation exact duplicates
are rejected. Superseded tuning scripts and generated experiments have been removed.
