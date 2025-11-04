import odrive
odrv0 = odrive.find_any()

print("Current encoder mode:", odrv0.axis0.encoder.config.mode)