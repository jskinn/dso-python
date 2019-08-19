from setuptools import setup, Extension
import sys
import platform
import numpy
import os
import re
from pathlib import Path


def get_include_paths():
    """
    Get a set of plausible search paths to look for things.
    """
    paths = {Path(sys.prefix) / 'include'}
    # If we're in an anaconda environment, add it as well (might be the same as sys.prefix)
    if 'CONDA_PREFIX' in os.environ:
        paths.add(Path(os.environ['CONDA_PREFIX']) / 'include')
    return paths


def get_library_paths():
    """
    Get a set of plausible search paths to look for things.
    TODO: Find a way of choosing workable directories on Windows/Mac
    """
    return {Path(sys.prefix) / 'lib'}


def find_path(search_paths, indicator_file):
    """
    Search for a path where a certain relative path exists.
    Use to find plausible include or library directories,
    e.g. `find_path([..], 'libopencv_core.so')`
    will try and find a directory containing libopencv_core.so, which can be added as a library dir.
    Exists because sometimes libraries install things in subdirectories, we need to find a root that works.
    """
    include_dir = None
    for to_search in search_paths:
        if include_dir is None:
            found_path = next(to_search.rglob(indicator_file), None)
            if found_path is not None:
                # Found a directory containing the given relative path
                include_dir = found_path.parents[len(indicator_file.split('/')) - 1] # Strip off the relative path we were searching for
    return include_dir


# Find the required include files and library dirs
search_paths = get_include_paths()
eigen_include = find_path(search_paths, 'signature_of_eigen3_matrix_library')
boost_include = find_path(search_paths, 'boost/thread.hpp')
cholmod_include = find_path(search_paths, 'cholmod.h')
csparse_include = find_path(search_paths, 'cs.h')

search_paths = get_library_paths()
boost_lib = find_path(search_paths, 'libboost_thread.so')
cholmod_lib = find_path(search_paths, 'libcholmod.so')
csparse_lib = find_path(search_paths, 'libcxsparse.so')

boost_libs = [
    'boost_system',
    'boost_thread' if platform.system() is not 'Darwin' else 'boost_thread-mt'
]
suitesparse_libs = ['cholmod', 'cxsparse']

include_dirs = {
    numpy.get_include(),
    str(eigen_include),
    str(boost_include),
    str(cholmod_include),
    str(csparse_include),
    'src',
    'thirdparty/Sophus'
}
library_dirs = {
    str(boost_lib),
    str(cholmod_lib),
    str(csparse_lib),
}
link_libs = boost_libs + suitesparse_libs

extra_compile_args = ['-march=native', '-std=c++11']
if platform.system() == 'Windows':
    # Add exception support on windows
    extra_compile_args.append('/EHsc')

setup(
    name="dso",
    version="0.1.0",
    py_modules=["dso"],
    zip_safe=False,
    ext_modules=[
        Extension(
	    "_dso",
            sources=[
                # DSO
                "src/FullSystem/FullSystem.cpp",
                "src/FullSystem/FullSystemOptimize.cpp",
                "src/FullSystem/FullSystemOptPoint.cpp",
                "src/FullSystem/FullSystemDebugStuff.cpp",
                "src/FullSystem/FullSystemMarginalize.cpp",
                "src/FullSystem/Residuals.cpp",
                "src/FullSystem/CoarseTracker.cpp",
                "src/FullSystem/CoarseInitializer.cpp",
                "src/FullSystem/ImmaturePoint.cpp",
                "src/FullSystem/HessianBlocks.cpp",
                "src/FullSystem/PixelSelector2.cpp",
                "src/OptimizationBackend/EnergyFunctional.cpp",
                "src/OptimizationBackend/AccumulatedTopHessian.cpp",
                "src/OptimizationBackend/AccumulatedSCHessian.cpp",
                "src/OptimizationBackend/EnergyFunctionalStructs.cpp",
                "src/util/settings.cpp",
                "src/util/Undistort.cpp",
                "src/util/globalCalib.cpp",

                # Dummy IO files, so that the symbols are defined.
                "src/IOWrapper/ImageDisplay_dummy.cpp",
                "src/IOWrapper/ImageRW_dummy.cpp",
                
                # Swig
                "dso.i"
            ],
            language="c++",
            swig_opts=['-c++','-threads', '-py3', '-v'],
            extra_compile_args=extra_compile_args,
            include_dirs=list(include_dirs),
            libraries=link_libs,
            library_dirs=list(library_dirs)
        )
    ]
)
