"""CPU inference for the cat-door obstruction model."""

import argparse
from functools import partial
import json
import math
from pathlib import Path

import torch
from PIL import Image, ImageOps
from torch import nn
from torchvision import models, transforms

CLASSES = ['obstructed', 'unobstructed']
DEFAULT_MODEL = Path(__file__).resolve().with_name('model.pt')

def load_image(path):
    """Returns:
        Image.Image: An RGB image with its EXIF orientation applied.
    """
    with Image.open(path) as image:
        return ImageOps.exif_transpose(image).convert('RGB')


def letterbox(image, size=224):
    """Returns:
        Image.Image: The complete image fitted into a padded square of the requested size.
    """
    return ImageOps.pad(image, (size, size), method=Image.Resampling.BILINEAR,
                        color=(124, 116, 104))


def normalize_lighting(image, mode="rgb"):
    """Returns:
        Image.Image: RGB channels with the selected brightness normalization.

    Raises:
        ValueError: If the normalization mode is unknown.
    """
    if mode == "rgb":
        return image
    gray = ImageOps.grayscale(image)
    if mode == "gray":
        return gray.convert("RGB")
    if mode == "equalize":
        return ImageOps.equalize(gray).convert("RGB")
    if mode == "autocontrast":
        return ImageOps.autocontrast(gray, cutoff=1).convert("RGB")
    raise ValueError(f"Unknown normalization: {mode}")

def preprocessing(training=False, size=224, normalization="rgb"):
    """Returns:
        transforms.Compose: Full-frame resizing and ImageNet normalization.
    """
    steps = [transforms.Lambda(partial(normalize_lighting, mode=normalization)),
             transforms.Lambda(partial(letterbox, size=size))]
    if training:
        steps.append(transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.1))
    steps.extend([transforms.ToTensor(),
                  transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])
    return transforms.Compose(steps)


def make_model(pretrained=False, pooling="avg"):
    """Returns:
        nn.Module: Frozen MobileNetV3-Small features and a trainable two-class head.
    """
    weights = models.MobileNet_V3_Small_Weights.DEFAULT if pretrained else None
    model = models.mobilenet_v3_small(weights=weights)
    if pooling not in {"avg", "max", "avg2", "max2"}:
        raise ValueError("Pooling must be avg, max, avg2, or max2")
    grid_size = 2 if pooling.endswith("2") else 1
    if pooling.startswith("max"):
        model.avgpool = nn.AdaptiveMaxPool2d(grid_size)
    else:
        model.avgpool = nn.AdaptiveAvgPool2d(grid_size)
    model.requires_grad_(False)
    model.classifier = nn.Linear(model.classifier[0].in_features * grid_size ** 2, len(CLASSES))
    return model


class Classifier:
    """Load once, then classify photos without downloading weights or reloading the model."""

    def __init__(self, checkpoint=DEFAULT_MODEL):
        """Load a local checkpoint onto the CPU."""
        saved = torch.load(checkpoint, map_location='cpu', weights_only=True)
        if (saved['classes'] != CLASSES
                or saved['architecture'] != 'mobilenet_v3_small_linear'
                or saved['preprocessing'] not in {'letterbox224_imagenet_v1', 'letterbox_imagenet_v2'}):
            raise ValueError('Unsupported checkpoint format')
        self.model = make_model(pooling=saved.get("pooling", "avg"))
        self.model.load_state_dict(saved['state_dict'])
        self.model.eval()
        self.transform = preprocessing(size=saved.get("input_size", 224),
                                       normalization=saved.get("normalization", "rgb"))
        self.threshold = saved['threshold']

    @torch.inference_mode()
    def predict(self, image_path, threshold=None):
        """Returns:
            dict: Predicted label, uncalibrated obstruction score, and decision threshold.
        """
        cutoff = self.threshold if threshold is None else threshold
        if not 0 < cutoff < 1:
            raise ValueError('Threshold must be strictly between 0 and 1')
        image = self.transform(load_image(image_path)).unsqueeze(0)
        score = self.model(image).softmax(dim=1)[0, 0].item()
        if not math.isfinite(score):
            raise ValueError('Model returned a non-finite obstruction score')
        return {'label': 'obstructed' if score >= cutoff else 'unobstructed',
                'obstructed_score': score, 'threshold': cutoff}


def main():
    """Classify one image and print the result as JSON."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--threshold', type=float)
    args = parser.parse_args()
    torch.set_num_threads(2)
    print(json.dumps(Classifier(args.model).predict(args.image, args.threshold)))


if __name__ == '__main__':
    main()
