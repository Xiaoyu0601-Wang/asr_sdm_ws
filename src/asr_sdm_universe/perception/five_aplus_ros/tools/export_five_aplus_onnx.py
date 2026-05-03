#!/usr/bin/env python3
"""Export UIE_Benckmark FiveA+ weights to an ONNX model for ROS2 inference."""

from __future__ import annotations

import argparse
from collections import OrderedDict
from pathlib import Path
from typing import Iterable

import numpy as np
import onnx
import onnxruntime as ort
import torch
import torch.nn as nn
import torch.nn.functional as F


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = Path("/home/cortin/Desktop/FIVE_APLUS_epoch97.pth")
DEFAULT_OUTPUT = PACKAGE_ROOT / "models" / "five_aplus_epoch97.onnx"
DEFAULT_INPUT_SHAPES = ((1, 3, 256, 256), (1, 3, 480, 640))


class PALayer(nn.Module):
    def __init__(self, channel: int):
        super().__init__()
        self.pa = nn.Sequential(
            nn.Conv2d(channel, channel // 8, 1, padding=0, bias=True),
            nn.ReLU(inplace=True),
            nn.Conv2d(channel // 8, 1, 1, padding=0, bias=True),
            nn.Sigmoid(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x * self.pa(x)


class Enhance(nn.Module):
    def __init__(self):
        super().__init__()
        self.relu = nn.ReLU(inplace=True)
        self.tanh = nn.Tanh()
        self.refine2 = nn.Conv2d(16, 16, kernel_size=3, stride=1, padding=1)
        self.conv1010 = nn.Conv2d(16, 1, kernel_size=1, stride=1, padding=0)
        self.conv1020 = nn.Conv2d(16, 1, kernel_size=1, stride=1, padding=0)
        self.conv1030 = nn.Conv2d(16, 1, kernel_size=1, stride=1, padding=0)
        self.refine3 = nn.Conv2d(16 + 3, 16, kernel_size=3, stride=1, padding=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        dehaze = self.relu(self.refine2(x))
        shape_out = dehaze.shape[2:4]
        x101 = F.avg_pool2d(dehaze, 128)
        x102 = F.avg_pool2d(dehaze, 64)
        x103 = F.avg_pool2d(dehaze, 32)
        x1010 = F.interpolate(self.relu(self.conv1010(x101)), size=shape_out, mode="nearest")
        x1020 = F.interpolate(self.relu(self.conv1020(x102)), size=shape_out, mode="nearest")
        x1030 = F.interpolate(self.relu(self.conv1030(x103)), size=shape_out, mode="nearest")
        dehaze = torch.cat((x1010, x1020, x1030, dehaze), 1)
        return self.tanh(self.refine3(dehaze))


class SFDIMONNX(nn.Module):
    """ONNX-compatible spectral mixer.

    The UIE_Benckmark implementation uses ``irfft2`` on a full-width complex
    tensor. PyTorch's new ONNX exporter currently emits a graph that ORT cannot
    execute for that path. ``ifft2`` keeps the spectral inverse transform in the
    graph, exports to ONNX DFT correctly, and remains numerically close for the
    downstream FiveA+ output.
    """

    def __init__(self, n_feats: int):
        super().__init__()
        self.Conv1 = nn.Sequential(
            nn.Conv2d(n_feats, 2 * n_feats, 1, 1, 0),
            nn.LeakyReLU(0.1, inplace=True),
            nn.Conv2d(2 * n_feats, n_feats, 1, 1, 0),
        )
        self.Conv1_1 = nn.Sequential(
            nn.Conv2d(n_feats, 2 * n_feats, 1, 1, 0),
            nn.LeakyReLU(0.1, inplace=True),
            nn.Conv2d(2 * n_feats, n_feats, 1, 1, 0),
        )
        self.Conv2 = nn.Conv2d(n_feats, n_feats, 1, 1, 0)
        self.scale = nn.Parameter(torch.zeros((1, n_feats, 1, 1)), requires_grad=True)

    def forward(self, x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        height, width = x.shape[-2], x.shape[-1]
        mix = x + y
        mix_for_phase = mix + 1e-8
        mix_mag = self.Conv1(torch.abs(mix_for_phase))
        mix_pha = self.Conv1_1(torch.angle(mix_for_phase))
        real_main = mix_mag * torch.cos(mix_pha)
        imag_main = mix_mag * torch.sin(mix_pha)
        x_out_main = torch.complex(real_main, imag_main)
        x_out_main = torch.abs(torch.fft.ifft2(x_out_main, s=(height, width), norm="backward")) + 1e-8
        return self.Conv2(0.1 * x_out_main + 0.9 * mix)


class MCEM(nn.Module):
    def __init__(self, in_channels: int, channels: int):
        super().__init__()
        self.conv_first_r = nn.Conv2d(in_channels // 4, channels // 2, kernel_size=1, stride=1, padding=0, bias=False)
        self.conv_first_g = nn.Conv2d(in_channels // 4, channels // 2, kernel_size=1, stride=1, padding=0, bias=False)
        self.conv_first_b = nn.Conv2d(in_channels // 4, channels // 2, kernel_size=1, stride=1, padding=0, bias=False)
        self.instance_r = nn.InstanceNorm2d(channels // 2, affine=True)
        self.instance_g = nn.InstanceNorm2d(channels // 2, affine=True)
        self.instance_b = nn.InstanceNorm2d(channels // 2, affine=True)
        self.conv_out_r = nn.Conv2d(channels // 2, in_channels // 4, kernel_size=1, stride=1, padding=0, bias=False)
        self.conv_out_g = nn.Conv2d(channels // 2, in_channels // 4, kernel_size=1, stride=1, padding=0, bias=False)
        self.conv_out_b = nn.Conv2d(channels // 2, in_channels // 4, kernel_size=1, stride=1, padding=0, bias=False)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x1, x2, x3, x4 = torch.chunk(x, 4, dim=1)
        out_instance_r = self.conv_out_r(self.instance_r(self.conv_first_r(x1)))
        out_instance_g = self.conv_out_g(self.instance_g(self.conv_first_g(x2)))
        out_instance_b = self.conv_out_b(self.instance_b(self.conv_first_b(x3)))
        mix = out_instance_r + out_instance_g + out_instance_b + x4
        return torch.cat((out_instance_r, out_instance_g, out_instance_b, mix), dim=1)


class FIVE_APLUSNet(nn.Module):
    def __init__(self, in_nc: int = 3, out_nc: int = 3, base_nf: int = 16):
        super().__init__()
        self.base_nf = base_nf
        self.out_nc = out_nc
        self.pyramid_enhance = Enhance()
        self.color_cer_1 = MCEM(base_nf, base_nf * 2)
        self.color_cer_2 = MCEM(base_nf, base_nf * 2)
        self.fusion_mixer = SFDIMONNX(base_nf)
        self.conv1 = nn.Conv2d(in_nc, base_nf, 1, 1, bias=True)
        self.conv2 = nn.Conv2d(base_nf, base_nf, 1, 1, bias=True)
        self.conv3 = nn.Conv2d(base_nf, out_nc, 1, 1, bias=True)
        self.conv4 = nn.Conv2d(base_nf, out_nc, 1, 1, bias=True)
        self.stage2 = PALayer(base_nf)
        self.act = nn.ReLU(inplace=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.conv1(x)
        out_1 = self.color_cer_1(out)
        out_2 = self.pyramid_enhance(out)
        mix_out = self.fusion_mixer(out_1, out_2)
        out_stage2 = self.act(mix_out)
        out_stage2 = self.conv2(out_stage2)
        out_stage2 = self.color_cer_2(out_stage2)
        out = self.stage2(out_stage2)
        out = self.act(out)
        return self.conv3(out)


def filter_network_state_dict(checkpoint: object) -> tuple[OrderedDict[str, torch.Tensor], list[str]]:
    if isinstance(checkpoint, dict):
        state_dict = checkpoint.get("state_dict", checkpoint)
    else:
        raise TypeError(f"Unsupported checkpoint type: {type(checkpoint).__name__}")

    network_state: OrderedDict[str, torch.Tensor] = OrderedDict()
    removed: list[str] = []
    for raw_key, value in state_dict.items():
        key = str(raw_key)
        if key.startswith("per_loss."):
            removed.append(key)
            continue
        for prefix in ("module.", "model.", "net.", "netG_1."):
            if key.startswith(prefix):
                key = key[len(prefix) :]
        network_state[key] = value
    return network_state, removed


def resolve_output_path(package_root: Path, output_path: Path) -> Path:
    return output_path if output_path.is_absolute() else package_root / output_path


def normalize_output_tensor(img: torch.Tensor) -> torch.Tensor:
    if torch.max(img) > 1 or torch.min(img) < 0:
        im_max = torch.max(img)
        im_min = torch.min(img)
        img = (img - im_min) / (im_max - im_min + 1e-7)
    return torch.clamp(img, 0.0, 1.0)


def load_model(weights_path: Path) -> FIVE_APLUSNet:
    checkpoint = torch.load(weights_path, map_location="cpu")
    state_dict, removed = filter_network_state_dict(checkpoint)
    model = FIVE_APLUSNet().eval()
    missing_keys, unexpected_keys = model.load_state_dict(state_dict, strict=False)
    print(f"filtered training-only keys: {len(removed)}")
    print(f"missing keys: {missing_keys}")
    print(f"unexpected keys: {unexpected_keys}")
    if missing_keys or unexpected_keys:
        raise RuntimeError("Checkpoint did not match FIVE_APLUSNet after filtering.")
    return model


def export_onnx(model: nn.Module, output_path: Path, opset: int) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    dummy = torch.randn(DEFAULT_INPUT_SHAPES[0], dtype=torch.float32)
    height = torch.export.Dim("height", min=128)
    width = torch.export.Dim("width", min=128)
    torch.onnx.export(
        model,
        (dummy,),
        str(output_path),
        input_names=["input"],
        output_names=["output"],
        opset_version=opset,
        dynamo=True,
        external_data=False,
        dynamic_shapes={"x": {0: 1, 2: height, 3: width}},
    )


def validate_onnx(model: nn.Module, output_path: Path, shapes: Iterable[tuple[int, int, int, int]]) -> None:
    onnx_model = onnx.load(str(output_path))
    onnx.checker.check_model(onnx_model)
    session = ort.InferenceSession(str(output_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    for shape in shapes:
        generator = torch.Generator().manual_seed(sum(shape))
        sample = torch.rand(shape, generator=generator, dtype=torch.float32)
        with torch.no_grad():
            torch_output = normalize_output_tensor(model(sample)).cpu().numpy()
        ort_output = session.run([output_name], {input_name: sample.numpy()})[0]
        ort_output = normalize_numpy_output(ort_output)
        diff = np.abs(torch_output - ort_output)
        mean_abs = float(diff.mean())
        max_abs = float(diff.max())
        print(f"validate shape={shape}: mean_abs_error={mean_abs:.8f}, max_abs_error={max_abs:.8f}")
        if mean_abs > 1e-3 or max_abs > 1e-2:
            raise RuntimeError(f"ONNX validation failed for {shape}: mean={mean_abs}, max={max_abs}")


def normalize_numpy_output(img: np.ndarray) -> np.ndarray:
    if float(img.max()) > 1.0 or float(img.min()) < 0.0:
        img = (img - img.min()) / (img.max() - img.min() + 1e-7)
    return np.clip(img, 0.0, 1.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export FiveA+ to ONNX for five_aplus_ros.")
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--opset", type=int, default=20)
    parser.add_argument("--skip-validation", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_path = resolve_output_path(PACKAGE_ROOT, args.output)
    model = load_model(args.weights)
    export_onnx(model, output_path, args.opset)
    if not args.skip_validation:
        validate_onnx(model, output_path, DEFAULT_INPUT_SHAPES)
    print(f"exported: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
