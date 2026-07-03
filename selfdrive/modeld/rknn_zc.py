"""Zero-copy RKNN runner (ctypes wrapper around librknn_zc.so / RKNN C API).

Per frame: single-pass C conversion (uint8 NCHW -> native fp16 NHWC) straight
into a DMA-mapped rknn_tensor_mem, one rknn_run, fp16->fp32 output copy.
Avoids the Python rknnlite marshaling path (~6-12ms/call).
"""
import ctypes
import numpy as np

_LIB = None

def _lib():
  global _LIB
  if _LIB is None:
    L = ctypes.CDLL('librknn_zc.so')
    L.zc_init.restype = ctypes.c_void_p
    L.zc_init.argtypes = [ctypes.c_char_p, ctypes.c_int]
    L.zc_fill_u8.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    L.zc_fill_f32.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    L.zc_run.argtypes = [ctypes.c_void_p]
    L.zc_out_elems.argtypes = [ctypes.c_void_p, ctypes.c_int]
    L.zc_out_elems.restype = ctypes.c_int
    L.zc_get_output_f32.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    L.zc_release.argtypes = [ctypes.c_void_p]
    L.zc_set_core.argtypes = [ctypes.c_void_p, ctypes.c_int]
    _LIB = L
  return _LIB

class ZCModel:
  def __init__(self, path: str, verbose: int = 0, core_mask: int = 0):
    self.L = _lib()
    self.h = self.L.zc_init(path.encode(), verbose)
    if not self.h:
      raise RuntimeError(f"zc_init failed for {path}")
    if core_mask:  # 1=CORE_0, 7=CORE_0_1_2 (multi-core conv split; verified bit-identical)
      self.L.zc_set_core(self.h, core_mask)

  def fill_u8(self, idx: int, arr: np.ndarray):
    assert arr.dtype == np.uint8 and arr.flags['C_CONTIGUOUS']
    r = self.L.zc_fill_u8(self.h, idx, arr.ctypes.data_as(ctypes.c_void_p))
    assert r == 0, f"fill_u8[{idx}] ret={r}"

  def fill_f32(self, idx: int, arr: np.ndarray):
    if arr.dtype != np.float32 or not arr.flags['C_CONTIGUOUS']:
      arr = np.ascontiguousarray(arr, dtype=np.float32)
    r = self.L.zc_fill_f32(self.h, idx, arr.ctypes.data_as(ctypes.c_void_p))
    assert r == 0, f"fill_f32[{idx}] ret={r}"

  def run(self):
    r = self.L.zc_run(self.h)
    assert r == 0, f"zc_run ret={r}"

  def output(self, idx: int = 0) -> np.ndarray:
    n = self.L.zc_out_elems(self.h, idx)
    out = np.empty(n, dtype=np.float32)
    self.L.zc_get_output_f32(self.h, idx, out.ctypes.data_as(ctypes.c_void_p))
    return out

  def release(self):
    if self.h:
      self.L.zc_release(self.h)
      self.h = None
