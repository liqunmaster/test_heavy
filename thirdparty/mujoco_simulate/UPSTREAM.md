MuJoCo simulate sources are vendored from google-deepmind/mujoco tag 3.15.0,
commit 9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5.
Source: https://github.com/google-deepmind/mujoco/tree/3.15.0/simulate
The original Apache 2.0 license and copyright notices are preserved.
The project compiles the viewer library via thirdparty/CMakeLists.txt;
the upstream main.cc and CMakeLists.txt are retained as reference only.

PNG support uses lvandeve/lodepng at commit
22561883dd63fd1850f18e1f6adac321e4f609b0, the version used by MuJoCo 3.15.0.
Its license is embedded in lodepng.cpp and lodepng.h.
