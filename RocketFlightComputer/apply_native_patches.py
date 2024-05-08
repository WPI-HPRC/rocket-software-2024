import os

Import("env")

env.AddBuildMiddleware(lambda env, node: env.File("patches/native/NotSoBasicLinearAlgebra.h"), "BasicLinearAlgebra/impl/NotSoBasicLinearAlgebra.h")
