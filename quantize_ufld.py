#!/usr/bin/env python3
"""INT8 candidates of the UFLD model (PROPOSAL ONLY: nothing here changes config or the deployed model).

  python quantize_ufld.py                 # writes models/ufld/candidates/*.onnx
  python test_parity_ufld.py --path models/ufld/candidates/ufld_v2_same_dyn_int8.onnx --no-ref --threshold 0.92

Variants:
  dyn_int8      onnxruntime quantize_dynamic, weights int8, all supported ops (Conv -> ConvInteger, MatMul/Gemm)
  dyn_fc_int8   quantize_dynamic on MatMul/Gemm only (the large fully connected head)
  static_int8   quantize_static QDQ, per-channel, calibrated on 120 TRAINING frames (s01-s07, never s04 = val)
"""
import glob
import os
import sys

import cv2
import numpy as np

import config
from lanes.ufld import UfldModel

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "ufld", "candidates")


class Calib:
    def __init__(self, model, n=120):
        from onnxruntime.quantization import CalibrationDataReader  # noqa: F401
        files = []
        for s in ("s01", "s02", "s03", "s05", "s06", "s07"):
            fs = sorted(glob.glob(os.path.expanduser(f"~/car_project/dataset/lane/{s}/*.jpg")))
            files += fs[:: max(1, len(fs) // (n // 6))][: n // 6]
        self.data = iter([{model.input: model.preprocess(cv2.imread(f))} for f in files])
        print(f"calibration frames: {len(files)}")

    def get_next(self):
        return next(self.data, None)


def main(which):
    from onnxruntime.quantization import QuantFormat, QuantType, quantize_dynamic, quantize_static
    from onnxruntime.quantization.shape_inference import quant_pre_process
    os.makedirs(OUT, exist_ok=True)
    src = config.UFLD_MODELS["v2_same"]
    pre = os.path.join(OUT, "ufld_v2_same_pre.onnx")
    if not os.path.exists(pre):
        quant_pre_process(src, pre, skip_symbolic_shape=True)
    if "dyn" in which:
        quantize_dynamic(pre, os.path.join(OUT, "ufld_v2_same_dyn_int8.onnx"), weight_type=QuantType.QInt8)
        print("dyn_int8 done")
    if "dyn_fc" in which:
        quantize_dynamic(pre, os.path.join(OUT, "ufld_v2_same_dyn_fc_int8.onnx"), weight_type=QuantType.QInt8,
                         op_types_to_quantize=["MatMul", "Gemm"])
        print("dyn_fc_int8 done")
    if "static" in which:
        import onnx
        from onnx import version_converter
        pre13 = os.path.join(OUT, "ufld_v2_same_pre_op13.onnx")
        mm = onnx.load(pre)
        if mm.opset_import[0].version < 13:      # per-channel DequantizeLinear needs opset >= 13
            mm = version_converter.convert_version(mm, 13)
        onnx.save(mm, pre13)
        m = UfldModel(src, threads=4)
        quantize_static(pre13, os.path.join(OUT, "ufld_v2_same_static_int8.onnx"), Calib(m),
                        quant_format=QuantFormat.QDQ, per_channel=True,
                        activation_type=QuantType.QUInt8, weight_type=QuantType.QInt8)
        print("static_int8 done")
    for f in sorted(glob.glob(os.path.join(OUT, "*.onnx"))):
        print(f"{os.path.getsize(f) / 1e6:7.1f} MB  {os.path.basename(f)}")


if __name__ == "__main__":
    main(sys.argv[1:] or ["dyn", "dyn_fc", "static"])
