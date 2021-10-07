# Argus Cmake files

This folder exists becuase the .cmake files Nvidia provides do not work as
one might expect.

https://forums.developer.nvidia.com/t/could-not-find-argus-error/50895/2

These files provide static paths to where the argus libraries, headers, etc are
installed by the `nvidia-l4t-jetson-multimedia-api` apt package as of JetPack 4.5.
This folder is *appended* to `CMAKE_MODULE_PATH` so if and when changes  this,
It should still "just work". Both Meson and CMake can use these files, with
example usage in this project.