import os

Import("env")

PLATFORMIO_BUILD_DIR = ".pio/libdeps"
PATCHES_DIR = "patches"

for patch in os.listdir(PATCHES_DIR):
    patch_path = os.path.join(PATCHES_DIR, patch)

    parts = os.path.splitext(patch)[0].split("-")

    environ = parts[1]
    library = parts[2]
    lib_dir = os.path.join(PLATFORMIO_BUILD_DIR, environ, library)
    file = os.path.join(lib_dir, *parts[3:])

    patch_exists_path = os.path.join(lib_dir, f".{patch}")

    if not os.path.isfile(patch_exists_path):
        env.Execute(f"patch {file} {patch_path}")

        with open(patch_exists_path, "w"):
            pass
