import os
from patch import fromfile

env = Import("env")

# PLATFORMIO_BUILD_DIR = ".pio/libdeps"
PATCHES_DIR = "patches"

for patch in os.listdir(PATCHES_DIR):
    patch_path = os.path.join(PATCHES_DIR, patch)
    if (not os.path.isfile(patch_path)):
        continue
    patch = fromfile(patch_path)
    patch.apply()
