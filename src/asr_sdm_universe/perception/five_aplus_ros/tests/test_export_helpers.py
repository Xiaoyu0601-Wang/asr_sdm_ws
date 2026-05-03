import unittest
from collections import OrderedDict
from pathlib import Path
from tempfile import TemporaryDirectory

import torch

from tools.export_five_aplus_onnx import (
    DEFAULT_INPUT_SHAPES,
    filter_network_state_dict,
    normalize_output_tensor,
    resolve_output_path,
)


class ExportHelpersTests(unittest.TestCase):
    def test_filter_network_state_dict_removes_training_loss_weights(self):
        checkpoint = OrderedDict(
            [
                ("per_loss.vgg_layers.0.weight", torch.ones(1)),
                ("module.conv1.weight", torch.ones(1)),
                ("module.conv1.bias", torch.zeros(1)),
            ]
        )

        network_state, removed = filter_network_state_dict(checkpoint)

        self.assertEqual(removed, ["per_loss.vgg_layers.0.weight"])
        self.assertEqual(list(network_state.keys()), ["conv1.weight", "conv1.bias"])

    def test_filter_network_state_dict_accepts_lightning_state_dict_key(self):
        checkpoint = {
            "state_dict": OrderedDict(
                [
                    ("model.conv1.weight", torch.ones(1)),
                    ("model.conv1.bias", torch.zeros(1)),
                ]
            )
        }

        network_state, removed = filter_network_state_dict(checkpoint)

        self.assertEqual(removed, [])
        self.assertEqual(list(network_state.keys()), ["conv1.weight", "conv1.bias"])

    def test_resolve_output_path_handles_relative_paths_from_package_root(self):
        with TemporaryDirectory() as temp_dir:
            package_root = Path(temp_dir)
            output = resolve_output_path(package_root, Path("models/model.onnx"))

        self.assertEqual(output, package_root / "models" / "model.onnx")

    def test_normalize_output_tensor_matches_benchmark_min_max_behavior(self):
        tensor = torch.tensor([[[[-1.0, 1.0], [3.0, 5.0]]]], dtype=torch.float32)

        normalized = normalize_output_tensor(tensor)

        self.assertTrue(torch.allclose(normalized, torch.tensor([[[[0.0, 1.0 / 3.0], [2.0 / 3.0, 1.0]]]])))

    def test_default_input_shapes_cover_square_and_video_frame_sizes(self):
        self.assertEqual(DEFAULT_INPUT_SHAPES, ((1, 3, 256, 256), (1, 3, 480, 640)))


if __name__ == "__main__":
    unittest.main()
