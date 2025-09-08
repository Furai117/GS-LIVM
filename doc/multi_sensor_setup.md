# Multi-Sensor Setup

This guide describes wiring, calibration, and configuration for a platform equipped with dual cameras, a Livox Mid-360 LiDAR, and a Reach M+ GNSS module.

## Wiring

- **Cameras**: Connect both cameras to USB3 ports on the host computer. Use a hardware trigger to drive the shutters and share a common ground to keep the images synchronized.
- **Livox Mid-360**: Power the sensor with a stable 12–24 V supply. Connect its Ethernet port directly to the PC or through a switch and configure the IP address within the same subnet.
- **Reach M+**: Power the receiver and connect it to the PC via a serial‑to‑USB adapter for NMEA output. Optionally link the event pin to the camera trigger to record shutter times.
- Use a shared power source or properly regulated supplies so all sensors reference the same ground.

## Calibration

1. **Camera intrinsics** – Calibrate each camera using a checkerboard with tools such as `camera_calibration` or `Kalibr`.
2. **Extrinsics** – Estimate rigid transforms from `base_link` to each sensor (left camera, right camera, LiDAR, GNSS) using a calibration toolbox such as Kalibr or a LiDAR–camera calibration package.
3. **GNSS antenna offset** – Measure the physical offset of the Reach M+ antenna relative to the LiDAR frame and include it in the extrinsic parameters.

Record the resulting transformations in the [example configuration](../config/multi_sensor_example.yaml).

## Configuration

The repository provides example configuration and launch files:

- [config/multi_sensor_example.yaml](../config/multi_sensor_example.yaml)
- [launch/multi_sensor_example.launch](../launch/multi_sensor_example.launch)

The YAML file lists topics, frames, and extrinsic parameters. The launch file loads the configuration, publishes static transforms, and starts a node that consumes synchronized camera, LiDAR, and GNSS topics.

### Synchronized Topics

The example node uses approximate time synchronization (`approx_sync: true`) with a configurable `queue_size`. Adjust these values to match your network latency and sensor rates.

### Extrinsic Parameters

Static transforms published in the launch file encode the extrinsic calibration between sensors. Replace the placeholder values in the configuration with the results from your calibration procedure.

