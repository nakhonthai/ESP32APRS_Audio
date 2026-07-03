Import("env")
import os, re, shutil

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
}

def copy_firmware(source, target, env):
    # Extract VERSION and VERSION_BUILD from SCons CPPDEFINES
    defines = {}
    for item in env.get("CPPDEFINES", []):
        if isinstance(item, (list, tuple)) and len(item) == 2:
            defines[str(item[0])] = str(item[1])

    version = re.sub(r'[\\"\']', '', defines.get("VERSION", "0.0")).replace(".", "")
    version_build = re.sub(r'[\\"\']', '', defines.get("VERSION_BUILD", "a"))

    env_name = env["PIOENV"]
    prefix = FIRMWARE_NAMES.get(env_name, env_name)

    output_dir = os.path.join(env.subst("$PROJECT_DIR"), ".pio", "build", "firmware")
    os.makedirs(output_dir, exist_ok=True)

    src = str(target[0])
    dst = os.path.join(output_dir, "{}_V{}{}.bin".format(prefix, version, version_build))

    shutil.copy2(src, dst)
    print("\n*** Firmware saved: {} ***\n".format(dst))

env.AddPostAction("$BUILD_DIR/firmware.bin", copy_firmware)
