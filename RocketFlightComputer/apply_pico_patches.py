import os

Import("env")

env.AddBuildMiddleware(lambda env, node: env.File("patches/pico/SdFatConfig.h"), "ESP8266SdFat/src/SdFatConfig.h")
