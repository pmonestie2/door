# Deployed obstruction model

`model.pt` contains the deployed PyTorch weights and all inference settings.
`model-report.json` records selection metrics and the dataset filenames.

On the Pi:

```sh
~/door-venv/bin/python ~/door/fat_cat_model/inference.py /path/to/photo.jpg
```

The controller loads this model once, warms it in a background thread, then
reuses it for website checks and checks before normal closing. Its current input
is RGB letterboxed to 384×384, with a frozen MobileNetV3-Small backbone, 2×2
average pooling, and a linear head. Threshold: 0.8172647356987.

The 53 tuning images pass, but a darkened obstruction was missed in additional
perturbation checks. Scores are uncalibrated and tuning results are not independent
test performance. Refer to `../training/README.md` for Mac training commands.

The obsolete ONNX implementation and redundant model backups were removed.
Git retains committed model versions. Training outputs remain on the Mac until
explicitly deployed with matching inference settings.
