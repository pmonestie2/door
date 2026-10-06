"""Checks for image boundaries, frozen features, and saved-model predictions."""
import tempfile
import unittest
from pathlib import Path

import torch
from PIL import Image

from classifier import CLASSES, Classifier, letterbox, make_model


class ClassifierTests(unittest.TestCase):
    def test_whole_image_survives_resize(self):
        """Keep top and bottom obstructions visible in a portrait image."""
        image = Image.new('RGB', (100, 200), 'green')
        image.paste('red', (0, 0, 100, 20))
        image.paste('blue', (0, 180, 100, 200))
        result = letterbox(image)
        self.assertEqual(result.size, (224, 224))
        self.assertEqual(result.getpixel((112, 0)), (255, 0, 0))
        self.assertEqual(result.getpixel((112, 223)), (0, 0, 255))

    def test_training_keeps_backbone_fixed(self):
        """Verify head gradients do not update backbone weights or BatchNorm statistics."""
        model = make_model()
        before = {name: value.clone() for name, value in model.features.state_dict().items()}
        model.eval()
        model.classifier.train()
        optimizer = torch.optim.AdamW(model.classifier.parameters())
        loss = torch.nn.functional.cross_entropy(model(torch.randn(2, 3, 224, 224)),
                                                torch.tensor([0, 1]))
        loss.backward()
        optimizer.step()
        self.assertEqual(sum(p.numel() for p in model.parameters() if p.requires_grad), 1154)
        for name, value in model.features.state_dict().items():
            self.assertTrue(torch.equal(value, before[name]), name)

    def test_reload_preserves_class_and_threshold(self):
        """Check that class zero means obstructed and threshold overrides work after loading."""
        model = make_model()
        with torch.no_grad():
            model.classifier.weight.zero_()
            model.classifier.bias.copy_(torch.tensor([1.0, 0.0]))
        with tempfile.TemporaryDirectory() as folder:
            checkpoint = Path(folder) / 'model.pt'
            photo = Path(folder) / 'image.jpg'
            Image.new('RGB', (100, 200)).save(photo)
            torch.save({'state_dict': model.state_dict(), 'classes': CLASSES,
                        'architecture': 'mobilenet_v3_small_linear',
                        'preprocessing': 'letterbox224_imagenet_v1', 'threshold': 0.5}, checkpoint)
            predictor = Classifier(checkpoint)
            self.assertEqual(predictor.predict(photo)['label'], 'obstructed')
            self.assertEqual(predictor.predict(photo, threshold=0.9)['label'], 'unobstructed')
            with self.assertRaises(ValueError):
                predictor.predict(photo, threshold=1.1)


if __name__ == '__main__':
    torch.set_num_threads(2)
    unittest.main()
