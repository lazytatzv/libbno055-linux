with open('config/bno055_params.yaml', 'r') as f:
    yaml = f.read()

new_params = """    # ---------------------------------------------------------
    # Advanced Hardware Configuration
    # ---------------------------------------------------------

    # Sensor Fusion / Operation Mode
    # Options: ndof, imu_plus, compass, m4g, amg, acc_gyro, mag_gyro, acc_mag, gyro_only, mag_only, acc_only
    # Note: "imu_plus" (6-axis fusion) is highly recommended for indoor robots (avoids magnetic interference).
    #       "ndof" (9-axis fusion) provides absolute heading using the magnetometer.
    operation_mode: "imu_plus"

    # Axis Remapping (Orientation of the chip on your robot)
    # Options: p0, p1, p2, p3, p4, p5, p6, p7
    # Default is p1. If the sensor is mounted upside down or rotated, change this.
    axis_map_config: "p1"
    axis_map_sign: "p1"

    # External 32.768kHz Crystal Oscillator
    # Setting to true provides highly accurate and stable timing for the sensor fusion algorithms.
    use_external_crystal: true

    # ---------------------------------------------------------
"""

if "operation_mode" not in yaml:
    yaml = yaml.replace("    # Calibration Autoloading", new_params + "    # Calibration Autoloading")
    with open('config/bno055_params.yaml', 'w') as f:
        f.write(yaml)

