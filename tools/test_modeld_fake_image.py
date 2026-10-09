#!/usr/bin/env python3
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
os.chdir(REPO)
sys.path[:0] = [str(REPO), str(REPO / "tinygrad_repo")]

from openpilot.selfdrive.modeld.helpers import load_oob, modeld_pkl_path
from openpilot.common.file_chunker import open_file_chunked

model_path = modeld_pkl_path(False)
print(f"model={model_path}")
started = time.perf_counter()
artifact = load_oob(open_file_chunked(model_path))
print(f"load_seconds={time.perf_counter() - started:.3f}")
print("input_specs:")
for name, spec in artifact["input_specs"].items():
  print(f"  {name}: shape={spec[0]} dtype={spec[1]} device={spec[2]}")
print("output_specs:")
for name, spec in artifact["output_specs"].items():
  print(f"  {name}: shape={spec[0]} dtype={spec[1]} device={spec[2]}")

from tinygrad import Device, Tensor

started = time.perf_counter()
cl_result = (Tensor.ones(1024, device="CL") * 3).sum().numpy().item()
Device["CL"].synchronize()
print(f"opencl_sum={cl_result} opencl_seconds={time.perf_counter() - started:.3f} device={Device['CL'].device_name}")

from openpilot.selfdrive.modeld.modeld import ModelState

started = time.perf_counter()
state = ModelState(1344, 760, chestnut=False)
print(f"model_state_init_seconds={time.perf_counter() - started:.3f}")
started = time.perf_counter()
state.warmup()
print(f"fake_nv12_warmup_seconds={time.perf_counter() - started:.3f}")
print("fake_nv12_inference=PASS")
