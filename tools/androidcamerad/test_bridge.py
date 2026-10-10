#!/usr/bin/env python3

from openpilot.cereal import messaging
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient


state_socket = messaging.sub_sock("narrowRoadCameraState", timeout=10_000)
client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD, True)
if not client.connect(True):
  raise RuntimeError("failed to connect to camerad VisionIPC")

buffers = [client.recv(10_000)]
timestamps = [client.timestamp_sof]
for _ in range(19):
  buffers.append(client.recv(2_000))
  timestamps.append(client.timestamp_sof)

buffer = buffers[-1]
state = messaging.recv_one(state_socket)
if any(item is None for item in buffers) or state is None:
  raise RuntimeError("timed out waiting for an Android camera frame")

camera_state = state.narrowRoadCameraState
print(f"VisionIPC: {client.width}x{client.height}, stride={client.stride}, "
      f"buffers={client.num_buffers}, bytes={len(buffer.data)}")
print(f"Frame: vipc={client.frame_id}, state={camera_state.frameId}, "
      f"timestamp={client.timestamp_sof}, valid={client.valid}")
fps = (len(timestamps) - 1) * 1e9 / (timestamps[-1] - timestamps[0])
sample = bytes(buffer.data[:client.stride * client.height:4096])
uv_sample = bytes(buffer.data[client.uv_offset:client.uv_offset + client.stride * client.height // 2:4096])
print(f"Rate: {fps:.1f} FPS, sampled Y={min(sample)}..{max(sample)}, UV={min(uv_sample)}..{max(uv_sample)}")

if client.width != buffer.width or client.height != buffer.height:
  raise RuntimeError("VisionIPC client and buffer dimensions disagree")
if not client.valid:
  raise RuntimeError("VisionIPC frame is marked invalid")
if not any(sample):
  raise RuntimeError("sampled luma is entirely zero")
if not any(uv_sample):
  raise RuntimeError("sampled chroma is entirely zero")
if not 18.0 <= fps <= 22.0:
  raise RuntimeError(f"unexpected camera frame rate: {fps:.1f} FPS")
