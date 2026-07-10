Import("env")
import os, re, shutil, json, datetime

FIRMWARE_NAMES = {
    "esp32-nodisp":     "ESP32_NODISP",
    "esp32-sh1106":     "ESP32_SH1106",
    "esp32-ssd1306":    "ESP32_SSD1306",
    "esp32-noota":      "ESP32_NOOTA",
    "esp32c3-nodisp":   "ESP32C3_NODISP",
    "esp32c3-noota":    "ESP32C3_NOOTA",
    "esp32c3-sh1106":   "ESP32C3_SH1106",
    "esp32c3-ssd1306":  "ESP32C3_SSD1306",
    "esp32s3-sh1106":   "ESP32S3_SH1106",
    "esp32s3-noota":    "ESP32S3_NOOTA",
    "esp32s3-N16R8":    "ESP32S3_N16R8",
    "TTGO-TWR":         "ESP32S3_TWR",
}

def copy_firmware(source, target, env):
    # Extract VERSION and VERSION_BUILD from SCons CPPDEFINES
    defines = {}
    for item in env.get("CPPDEFINES", []):
        if isinstance(item, (list, tuple)) and len(item) == 2:
            defines[str(item[0])] = str(item[1])

    version_raw = re.sub(r'[\\"\']', '', defines.get("VERSION", "0.0"))
    version = version_raw.replace(".", "")
    version_build = re.sub(r'[\\"\']', '', defines.get("VERSION_BUILD", "a"))

    env_name = env["PIOENV"]
    prefix = FIRMWARE_NAMES.get(env_name, env_name)

    output_dir = os.path.join(env.subst("$PROJECT_DIR"), ".pio", "build", "firmware")
    os.makedirs(output_dir, exist_ok=True)

    src = str(target[0])
    dst = os.path.join(output_dir, "{}_V{}{}.bin".format(prefix, version, version_build))

    shutil.copy2(src, dst)
    print("\n*** Firmware saved: {} ***\n".format(dst))

    # Write version info (used by webservice.cpp handle_check_version to detect
    # new firmware releases). JSON is used instead of plain text so it can be
    # parsed easily with ArduinoJson on the device.
    version_info = {
        "version": version_raw,
        "build": version_build,
        "date": datetime.date.today().isoformat(),
    }
    version_info_path = os.path.join(output_dir, "version.json")
    with open(version_info_path, "w") as f:
        json.dump(version_info, f, indent=2)
    print("*** Version info saved: {} ***\n".format(version_info_path))

env.AddPostAction("$BUILD_DIR/firmware.bin", copy_firmware)
