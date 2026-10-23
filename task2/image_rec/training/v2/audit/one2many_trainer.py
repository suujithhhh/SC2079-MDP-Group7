"""Train exactly the YOLO26 detection head exported by this NCNN version.

Usage: YOLO(v1_best).train(trainer=DeploymentHeadTrainer, ..., end2end=False).
Run this module directly for a CPU-only graph/weight/gradient smoke check;
it performs no optimizer update and does not overwrite any model checkpoint.
"""
from copy import deepcopy
from pathlib import Path
import json
import os

TASK2 = Path(__file__).resolve().parents[2]
os.environ.setdefault("YOLO_CONFIG_DIR", str(TASK2 / "v2" / "config"))
os.environ.setdefault("YOLO_AUTOINSTALL", "false")

from ultralytics.models.yolo.detect.train import DetectionTrainer
from ultralytics.nn.tasks import yaml_model_load


class DeploymentHeadTrainer(DetectionTrainer):
    """Construct without one-to-one heads before loading pretrained weights."""

    def get_model(self, cfg=None, weights=None, verbose=True):
        if cfg is None:
            if weights is None or not hasattr(weights, "yaml"):
                raise ValueError("A YOLO26 model configuration is required")
            cfg = weights.yaml
        cfg = deepcopy(cfg if isinstance(cfg, dict) else yaml_model_load(cfg))
        cfg["end2end"] = False
        model = super().get_model(cfg=cfg, weights=weights, verbose=verbose)
        assert not model.end2end
        assert not any("one2one" in name for name, _ in model.named_parameters())
        assert model.yaml["end2end"] is False
        return model

    def set_model_attributes(self):
        self.args.end2end = False
        super().set_model_attributes()
        assert not self.model.end2end, "Validation must use the deployed head"


def smoke_check():
    from types import SimpleNamespace
    import torch
    from ultralytics import YOLO
    from ultralytics.cfg import get_cfg
    from ultralytics.utils.loss import v8DetectionLoss

    torch.set_num_threads(4)
    torch.manual_seed(2626)
    source = YOLO(str(TASK2 / "weights" / "best.pt")).model.float().cpu()
    source.end2end = False
    trainer = object.__new__(DeploymentHeadTrainer)
    trainer.data = {"nc": 3, "channels": 3, "names": source.names}
    trainer.args = SimpleNamespace(cls_remap=True)
    model = trainer.get_model(cfg=source.yaml, weights=source, verbose=False)
    model.args = get_cfg(overrides={"epochs": 1, "end2end": False})
    model.names = source.names
    source_state, target_state = source.state_dict(), model.state_dict()
    assert set(target_state).issubset(source_state)
    assert all(torch.equal(value, source_state[key]) for key, value in target_state.items())

    sample = torch.rand(2, 3, 192, 192)
    source.eval()
    model.eval()
    with torch.no_grad():
        expected = source(sample)[0]
        actual = model(sample)[0]
    output_max_abs_diff = float((actual - expected).abs().max())
    assert torch.equal(actual, expected), output_max_abs_diff

    model.train()
    batch = {
        "img": sample,
        "batch_idx": torch.tensor([0., 1.]),
        "cls": torch.tensor([[1.], [2.]]),
        "bboxes": torch.tensor([[.45, .5, .25, .22], [.6, .5, .2, .25]]),
    }
    losses, loss_items = model(batch)
    assert isinstance(model.criterion, v8DetectionLoss)
    assert torch.isfinite(losses).all()
    losses.sum().backward()
    groups = {
        "backbone": [parameter for name, parameter in model.named_parameters() if name.startswith("model.0.")],
        "one2many_box": list(model.model[-1].cv2.parameters()),
        "one2many_class": list(model.model[-1].cv3.parameters()),
    }
    gradients = {}
    for group, parameters in groups.items():
        values = [parameter.grad for parameter in parameters if parameter.grad is not None]
        assert values and all(torch.isfinite(value).all() for value in values), group
        magnitude = sum(float(value.abs().sum()) for value in values)
        assert magnitude > 0, group
        gradients[group] = magnitude

    report = {
        "source": str(TASK2 / "weights" / "best.pt"),
        "end2end": model.end2end,
        "yaml_end2end": model.yaml["end2end"],
        "criterion": type(model.criterion).__name__,
        "all_retained_tensors_identical_before_backward": True,
        "one2many_forward_max_abs_difference": output_max_abs_diff,
        "source_parameters": sum(p.numel() for p in source.parameters()),
        "single_head_parameters": sum(p.numel() for p in model.parameters()),
        "finite_nonzero_gradient_l1": gradients,
        "loss_items": {key: float(value) for key, value in loss_items.items()},
        "optimizer_steps": 0,
        "interpretation": "Graph compatibility only; no claim about trained accuracy or real-camera behavior.",
    }
    output = Path(__file__).with_name("one2many_smoke_check.json")
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    smoke_check()
