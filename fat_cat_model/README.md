## Deployed model (2026-10-05)

`model.pt` and `inference.py` now use grayscale histogram equalization, 384×384 letterboxing, and 2×2 average pooling. The checkpoint supplies all inference settings, including threshold 0.1991468444466591. It passed all 49 tuning images (10 obstructed, 39 clear). See `model-report.json` for details. This is not independent test performance. The prior model and inference code are retained in `backups/`.

The legacy ONNX files and their `model-config.json` describe the older model; the door uses the PyTorch checkpoint directly.

# Cat-door inference

Run from the door directory:

```sh
python3 fat_cat_model/inference.py /path/to/photo.jpg
```

The model path defaults to `model.pt` beside the script, regardless of the working directory. Output is JSON containing `label`, `obstructed_score`, and `threshold`. The score is not a calibrated probability. The checkpoint supplies the selected threshold, pooling, and input size.

For repeated inference, load once:

```python
from fat_cat_model.inference import Classifier
classifier = Classifier()
result = classifier.predict('/path/to/photo.jpg')
```

Dependencies are listed in `requirements.txt`. PyTorch/TorchVision installation and inference latency have not yet been verified on the Pi 3. Inference runs on CPU and does not download weights. An invalid image or checkpoint raises an exception; it does not return a clear-door result.
