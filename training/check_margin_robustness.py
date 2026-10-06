"""Compare a training run's models on fixed brightness, contrast, and JPEG changes."""

import argparse
from io import BytesIO
import json
from pathlib import Path

import torch
from PIL import ImageEnhance
from classifier import CLASSES, DATA_ROOT, Classifier, load_dataset, load_image


def transformed_image(path, variation):
    """Returns:
        BytesIO: An encoded image with the requested deterministic perturbation.
    """
    image = load_image(path)
    if variation.startswith('brightness_'):
        image = ImageEnhance.Brightness(image).enhance(float(variation.split('_')[1]))
    elif variation.startswith('contrast_'):
        image = ImageEnhance.Contrast(image).enhance(float(variation.split('_')[1]))
    encoded = BytesIO()
    if variation == 'jpeg_75':
        image.save(encoded, format='JPEG', quality=75)
    else:
        image.save(encoded, format='PNG')
    encoded.seek(0)
    return encoded


def main():
    """Evaluate saved baseline and candidate models and write perturbation counts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='Output directory from improve_margin.py')
    parser.add_argument('--data-root', type=Path, default=DATA_ROOT)
    args = parser.parse_args()
    torch.set_num_threads(2)
    data = load_dataset(args.data_root / 'validation')
    models = {'previous': Classifier(args.run / 'baseline.pt'),
              'candidate': Classifier(args.run / 'model.pt')}
    summary = {name: {} for name in models}
    for variation in ('original', 'brightness_0.8', 'brightness_1.2',
                      'contrast_0.8', 'contrast_1.2', 'jpeg_75'):
        for name, model in models.items():
            misses = alarms = 0
            for path, label in data.samples:
                result = model.predict(transformed_image(path, variation))
                misses += CLASSES[label] == 'obstructed' and result['label'] != 'obstructed'
                alarms += CLASSES[label] == 'unobstructed' and result['label'] != 'unobstructed'
            summary[name][variation] = {'misses': misses, 'false_alarms': alarms}
            print(name, variation, summary[name][variation], flush=True)
    (args.run / 'perturbation-check.json').write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    main()
