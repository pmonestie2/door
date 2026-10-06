"""Compare linear-head training settings using scale-invariant validation separation."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import torch
from torch import nn
from torch.utils.data import DataLoader
from classifier import CLASSES, DATA_ROOT, DEFAULT_MODEL, Classifier, load_dataset, make_model, preprocessing

def features(model, dataset):
    """Returns:
        tuple: Frozen feature tensors and their class labels.
    """
    values, labels = [], []
    with torch.no_grad():
        for images, targets in DataLoader(dataset, batch_size=8):
            values.append(model.avgpool(model.features(images)).flatten(1))
            labels.append(targets)
    return torch.cat(values), torch.cat(labels)


def measure(head, x, y, vx, vy):
    """Returns:
        dict: Errors, threshold, raw scores, and a weight-normalized logit gap.
    """
    with torch.no_grad():
        logits = head(vx)
        differences = logits[:, 0] - logits[:, 1]
        low = float(differences[vy == 0].min())
        high = float(differences[vy == 1].max())
        boundary = (low + high) / 2 if low > high else low - .01
        cutoff = float(torch.sigmoid(torch.tensor(boundary)))
        scores = logits.softmax(1)[:, 0]
        train_scores = head(x).softmax(1)[:, 0]
        norm = float(torch.linalg.vector_norm(head.weight[0] - head.weight[1]))
        return dict(misses=int((scores[vy == 0] < cutoff).sum()),
                    false_alarms=int((scores[vy == 1] >= cutoff).sum()),
                    training_errors=int(((train_scores >= cutoff) != (y == 0)).sum()),
                    threshold=cutoff, min_obstructed=float(scores[vy == 0].min()),
                    max_clear=float(scores[vy == 1].max()),
                    normalized_gap=(low - high) / max(norm, 1e-12))


def main():
    """Search complete head-training curves without training on validation images."""
    torch.set_num_threads(2)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, default=DATA_ROOT)
    parser.add_argument('--baseline', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or args.data_root / 'training-runs' / datetime.now().strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    train = load_dataset(args.data_root / 'samples')
    valid = load_dataset(args.data_root / 'validation')
    hashes = {hashlib.sha256(Path(p).read_bytes()).digest() for p, _ in train.samples}
    assert not any(hashlib.sha256(Path(p).read_bytes()).digest() in hashes for p, _ in valid.samples)
    model = make_model(pretrained=True, pooling='avg2').eval()
    train.transform = valid.transform = preprocessing(size=384, normalization='rgb')
    x, y = features(model, train)
    vx, vy = features(model, valid)
    current = Classifier(args.baseline)
    checkpoint = torch.load(args.baseline, map_location='cpu', weights_only=True)
    if (checkpoint.get('input_size', 224) != 384 or checkpoint.get('pooling') != 'avg2'
            or checkpoint.get('normalization', 'rgb') != 'rgb'
            or any(not torch.equal(value, current.model.features.state_dict()[name])
                   for name, value in model.features.state_dict().items())):
        raise ValueError('Baseline must use the same frozen RGB 384px avg2 feature extractor')
    baseline = measure(current.model.classifier, x, y, vx, vy)
    (output / 'baseline.json').write_text(json.dumps(baseline, indent=2) + '\n')
    print('Baseline:', json.dumps(baseline), flush=True)
    best = (baseline['misses'], baseline['false_alarms'], baseline['training_errors'], -baseline['normalized_gap'])
    shutil.copy2(args.baseline, output / 'baseline.pt')
    checkpoint['threshold'] = baseline['threshold']
    torch.save(checkpoint, output / 'model.pt')
    (output / 'model.json').write_text(json.dumps(dict(baseline, selected='baseline'), indent=2) + '\n')
    counts = torch.bincount(y, minlength=2)
    history = []
    for seed in (42, 7):
        for lr in (.0003, .001, .003):
            for decay in (.01, .1):
                for smoothing in (0., .1):
                    torch.manual_seed(seed)
                    head = nn.Linear(x.shape[1], 2)
                    optimizer = torch.optim.AdamW(head.parameters(), lr=lr, weight_decay=decay)
                    loss_fn = nn.CrossEntropyLoss(weight=counts.sum() / (2 * counts.float()), label_smoothing=smoothing)
                    for epoch in range(1, 601):
                        optimizer.zero_grad()
                        loss = loss_fn(head(x), y)
                        loss.backward()
                        optimizer.step()
                        if epoch % 10:
                            continue
                        row = dict(measure(head, x, y, vx, vy), seed=seed, learning_rate=lr,
                                   weight_decay=decay, label_smoothing=smoothing, epoch=epoch,
                                   normalization='rgb', size=384, pooling='avg2')
                        history.append(row)
                        rank = (row['misses'], row['false_alarms'], row['training_errors'], -row['normalized_gap'])
                        if rank < best:
                            best = rank
                            model.classifier.load_state_dict(head.state_dict())
                            torch.save(dict(state_dict=model.state_dict(), classes=CLASSES,
                                            architecture='mobilenet_v3_small_linear', preprocessing='letterbox_imagenet_v2',
                                            normalization='rgb', input_size=384, pooling='avg2', threshold=row['threshold']), output / 'model.pt')
                            (output / 'model.json').write_text(json.dumps(row, indent=2) + '\n')
                    print('Finished:', seed, lr, decay, smoothing, 'best normalized gap:', -best[3], flush=True)
                    (output / 'history.json').write_text(json.dumps(history, indent=2) + '\n')
    selected = Classifier(output / 'model.pt')
    report = json.loads((output / 'model.json').read_text())
    for name, data in (('training', train), ('validation', valid)):
        rows = [dict(file=Path(p).name, actual=CLASSES[label], **selected.predict(p)) for p, label in data.samples]
        (output / (name + '-predictions.json')).write_text(json.dumps(rows, indent=2) + '\n')
        report[name + '_files'] = [str(Path(p).resolve()) for p, _ in data.samples]
        report[name + '_errors'] = sum(row['actual'] != row['label'] for row in rows)
    report['model_sha256'] = hashlib.sha256((output / 'model.pt').read_bytes()).hexdigest()
    report['validation_use'] = 'Repeated model selection, not independent testing.'
    (output / 'model.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Saved candidate:', output / 'model.pt', flush=True)


if __name__ == '__main__':
    main()
