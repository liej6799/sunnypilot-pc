#!/usr/bin/env python3
"""[op9] Publish the minimal state to put the UI 'onroad' and drive modeld's warp:
deviceState(started, tici), pandaStates(ignition), liveCalibration(level).
Importable as openpilot.op9_pub_onroad with a main() so the opmgr/manager can launch it."""
import time
import cereal.messaging as messaging
from cereal import log


def main():
  pm = messaging.PubMaster(['deviceState', 'pandaStates', 'liveCalibration'])
  while True:
    ds = messaging.new_message('deviceState')
    ds.deviceState.deviceType = 'tici'   # enum; makes DEVICE_CAMERAS[(tici, imx689)] resolve
    ds.deviceState.started = True
    pm.send('deviceState', ds)

    ps = messaging.new_message('pandaStates', 1)
    ps.pandaStates[0].pandaType = log.PandaState.PandaType.uno
    ps.pandaStates[0].ignitionLine = True
    pm.send('pandaStates', ps)

    lc = messaging.new_message('liveCalibration')
    lc.liveCalibration.calStatus = log.LiveCalibrationData.Status.calibrated
    lc.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
    lc.liveCalibration.wideFromDeviceEuler = [0.0, 0.0, 0.0]
    lc.liveCalibration.validBlocks = 100
    pm.send('liveCalibration', lc)

    time.sleep(0.05)


if __name__ == "__main__":
  main()
