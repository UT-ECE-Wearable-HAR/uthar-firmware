from distutils.core import setup, Extension
import numpy as np

module1 = Extension(
    "dmp",
    define_macros=[("NPY_NO_DEPRECATED_API", "NPY_1_7_API_VERSION")],
    include_dirs=[np.get_include()],
    sources=["src/dmp.cpp"],
)

setup(
    name="dmp",
    version="1.0",
    description="dmp package",
    ext_modules=[module1],
)
