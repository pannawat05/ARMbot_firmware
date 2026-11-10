# Calibration required
`H_camera_to_robot.npy` is intentionally not fabricated. Create it from measured pixel/table points
for the fixed camera at 640x480. `config.json` intentionally has null values for table Z, grasp offsets,
and tray XYZ. Fill these using measurements before expecting complete XYZ or pick-place behavior.
Camera stream URL is a placeholder; replace it with the actual ESP32-CAM firmware URL.
