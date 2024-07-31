#!/usr/bin/env python3

import depthai as dai
import json

#Parse parameters for calibration file name
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-c', '--calibration', type=str, required=True, help="Path to calibration file")
args = parser.parse_args()

# Connect device
with dai.Device(dai.OpenVINO.VERSION_2021_4, dai.UsbSpeed.HIGH) as device:

    print(f'Is EEPROM available: {device.isEepromAvailable()}')

    # User calibration
    try:
        print(f'User calibration: {json.dumps(device.readCalibration2().eepromToJson(), indent=2)}')
        #save calibration to file
        with open(args.calibration, 'w') as f:
            json.dump(device.readCalibration2().eepromToJson(), f, indent=2)
    except Exception as ex:
        print(f'No user calibration: {ex}')

    # Factory calibration
    try:
        print(f'Factory calibration: {json.dumps(device.readFactoryCalibration().eepromToJson(), indent=2)}')
    except Exception as ex:
        print(f'No factory calibration: {ex}')

    print(f'User calibration raw: {json.dumps(device.readCalibrationRaw())}')
    print(f'Factory calibration raw: {json.dumps(device.readFactoryCalibrationRaw())}')
