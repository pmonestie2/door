"""Train a small obstruction classifier or classify one photo on demand."""

import argparse
import hashlib
from functools import partial
import json
from pathlib import Path

import torch
from PIL import Image, ImageOps
from torch import nn
from torch.utils.data import DataLoader
from torchvision import datasets, models, transforms

CLASSES = ['obstructed', 'unobstructed']
ROOT = Path(__file__).resolve().parent
DATA_ROOT = Path.home() / "Documents/code/CATDOOR_TRAINING"
DEFAULT_MODEL = ROOT.parent / "fat_cat_model/model.pt"


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
    if pooling not in {"avg", "max", "avg2", "max2", "avg3", "avg4", "avg6"}:
        raise ValueError("Unsupported pooling grid")
    grid_size = int(pooling[-1]) if pooling[-1].isdigit() else 1
    if pooling.startswith("max"):
        model.avgpool = nn.AdaptiveMaxPool2d(grid_size)
    else:
        model.avgpool = nn.AdaptiveAvgPool2d(grid_size)
    model.requires_grad_(False)
    model.classifier = nn.Linear(model.classifier[0].in_features * grid_size ** 2, len(CLASSES))
    return model


def load_dataset(folder, training=False):
    """Returns:
        datasets.ImageFolder: Labeled photos with a fixed class ordering.

    Raises:
        ValueError: The folder does not contain exactly the two expected classes.
    """
    dataset = datasets.ImageFolder(folder, transform=preprocessing(training),
                                   loader=load_image,
                                   is_valid_file=lambda p: not Path(p).name.startswith('.')
                                   and Path(p).suffix.lower() in {'.jpg', '.jpeg', '.png'})
    if dataset.classes != CLASSES:
        raise ValueError(f'Expected folders {CLASSES}, found {dataset.classes}')
    return dataset


@torch.inference_mode()
def evaluate(model, dataset, threshold):
    """Returns:
        dict: Counts of missed obstructions and false alarms, plus obstruction recall.
    """
    model.eval()
    confusion = torch.zeros((2, 2), dtype=torch.int64)
    for images, labels in DataLoader(dataset, batch_size=16):
        scores = model(images).softmax(dim=1)[:, 0]
        predictions = (scores < threshold).long()
        for actual, predicted in zip(labels, predictions):
            confusion[actual, predicted] += 1
    hits, misses = confusion[0].tolist()
    alarms, clears = confusion[1].tolist()
    return {'obstructed_detected': hits, 'obstructed_missed': misses,
            'clear_false_alarm': alarms, 'clear_correct': clears,
            'obstructed_recall': hits / (hits + misses) if hits + misses else None}


def train(samples, output, epochs, learning_rate, validation, threshold):
    """Train only the linear head and save weights, class names, and preprocessing version."""
    torch.manual_seed(42)
    dataset = load_dataset(samples, training=True)
    validation_data = load_dataset(validation) if validation else None
    if validation_data is not None:
        training_hashes = {hashlib.sha256(Path(p).read_bytes()).digest()
                           for p, _ in dataset.samples}
        if any(hashlib.sha256(Path(p).read_bytes()).digest() in training_hashes
               for p, _ in validation_data.samples):
            raise ValueError('Validation contains copies of training photos.')
    counts = torch.bincount(torch.tensor(dataset.targets), minlength=2)
    print('Training photos:', dict(zip(CLASSES, counts.tolist())), flush=True)
    if validation_data is None:
        print('No independent validation set: training fit is NOT validation accuracy.', flush=True)
    model = make_model(pretrained=True)
    optimizer = torch.optim.AdamW(model.classifier.parameters(), lr=learning_rate,
                                 weight_decay=0.01)
    loss_function = nn.CrossEntropyLoss(weight=counts.sum() / (2 * counts.float()))
    loader = DataLoader(dataset, batch_size=16, shuffle=True)
    for epoch in range(epochs):
        # Keep frozen BatchNorm running statistics fixed, even during training.
        model.eval()
        model.classifier.train()
        total_loss = 0.0
        for images, labels in loader:
            optimizer.zero_grad()
            loss = loss_function(model(images), labels)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
        print(f'Epoch {epoch + 1}/{epochs}: loss={total_loss / len(loader):.4f}', flush=True)
    model.eval()
    fit = evaluate(model, load_dataset(samples), threshold)
    validation_metrics = evaluate(model, validation_data, threshold) if validation_data else None
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.save({'state_dict': model.state_dict(), 'classes': CLASSES,
                'architecture': 'mobilenet_v3_small_linear',
                'preprocessing': 'letterbox224_imagenet_v1', 'threshold': threshold}, output)
    report = {'training_fit': fit, 'validation': validation_metrics,
              'threshold': threshold, 'epochs': epochs,
              'training_files': [str(Path(p).resolve()) for p, _ in dataset.samples],
              'validation_files': [str(Path(p).resolve()) for p, _ in validation_data.samples]
              if validation_data else []}
    output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Training fit:', json.dumps(fit))
    if validation_metrics:
        print('Validation:', json.dumps(validation_metrics))
    print(f'Saved {output}')


class Classifier:
    """Load once, then classify photos without downloading weights or reloading the model."""

    def __init__(self, checkpoint):
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
        return {'label': 'obstructed' if score >= cutoff else 'unobstructed',
                'obstructed_score': score, 'threshold': cutoff}


def main():
    """Run training or print a single prediction as JSON."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    training = commands.add_parser('train')
    training.add_argument('--samples', type=Path, default=DATA_ROOT / 'samples')
    training.add_argument('--output', type=Path, default=DATA_ROOT / 'candidate.pt')
    training.add_argument('--validation', type=Path, help='Separate recording, same class folders')
    training.add_argument('--epochs', type=int, default=30)
    training.add_argument('--learning-rate', type=float, default=0.001)
    training.add_argument('--threshold', type=float, default=0.5)
    prediction = commands.add_parser('predict')
    prediction.add_argument('image', type=Path)
    prediction.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    prediction.add_argument('--threshold', type=float)
    args = parser.parse_args()
    torch.set_num_threads(2)
    if args.command == 'train':
        if args.epochs < 1 or args.learning_rate <= 0 or not 0 < args.threshold < 1:
            parser.error('Require epochs >= 1, learning-rate > 0, and 0 < threshold < 1')
        train(args.samples, args.output, args.epochs, args.learning_rate,
              args.validation, args.threshold)
    else:
        print(json.dumps(Classifier(args.model).predict(args.image, args.threshold)))


if __name__ == '__main__':
    main()
