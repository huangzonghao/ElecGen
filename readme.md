## Set up thirdparty dependencies
1. Make sure all the git submodules are initialized and updated. If not, run ` git submodule update --init --recursive`
2. Go to `thirdparty` and create a `build` folder.
3. **console_bridge**
    * Go to `console_bridge`, create `build`, go to `build`, run `cmake ..`
    * Then a `CMakeCache.txt` file would appear in the `build` folder.
    * Open `CMakeCache.txt` and search for `CMAKE_INSTALL_PREFIX`
    * Edit the line so that it would look like `CMAKE_INSTALL_PREFIX=/Abs/path/to/ElecGen/thirdparty/build`
    * Run `make install`
4. **urdfdom_headers**
    * Go to `urdfdom_headers`, create `build`, go to `build`, run `cmake ..`
    * Then a `CMakeCache.txt` file would appear in the `build` folder.
    * Open `CMakeCache.txt` and search for `CMAKE_INSTALL_PREFIX`
    * Edit the line so that it would look like `CMAKE_INSTALL_PREFIX=/Abs/path/to/ElecGen/thirdparty/build`
    * Run `make install`
5. **urdfdom**
    * Go to `urdfdom`, create `build`, go to `build`, run `cmake ..`
    * Then a `CMakeCache.txt` file would appear in the `build` folder.
    * Open `CMakeCache.txt` and search for `console_bridge_DIR`
    * Edit the line so that it would look like `console_bridge_DIR:PATH=/Abs/path/to/ElecGen/thirdparty/build/lib/console_bridge/cmake`
    * Also in `CMakeCache.txt` search for `urdfdom_headers_DIR`
    * Edit the line so that it would look like `urdfdom_headers_DIR:PATH=/Abs/path/to/ElecGen/thirdparty/build/lib/urdfdom_headers/cmake`
    * Also in `CMakeCache.txt` search for `CMAKE_INSTALL_PREFIX`
    * Edit the line so that it would look like `CMAKE_INSTALL_PREFIX=/Abs/path/to/ElecGen/thirdparty/build`
    * Run `make install`

## Compile Project
1. Create a `build` folder under project's root.
2. Go to the `build` folder and run `cmake ..`
3. Open the generated `CMakeCache.txt` file.
4. Link Chrono
    * Search for `Chrono_DIR`
    * Edit the line so that it looks like `Chrono_DIR=/Abs/path/to/chrono/build/cmake`
    * Note Chrono can be launched from either the build directory or the install directory.
    * But whichever directory you choose, make sure the path leads to a `cmake` directory where a `ChronoConfig.cmake` can be found.
5. Link the thirdpart dependencies
    * Search for `urdfdom_DIR`
    * Edit the line (or add the line if search failed) `urdfdom_DIR=/Abs/path/to/ElecGen/thirdparty/build/lib/urdfdom/cmake`
    * Similarly edit or add the line so that
        * `urdfdom_headers_DIR=/Abs/path/to/ElecGen/thirdparty/build/lib/urdfdom_headers/cmake`
        * `console_bridge_DIR=/Abs/path/to/ElecGen/thirdparty/build/lib/console_bridge/cmake`
6. Run `cmake ..` again
7. Run `make`

## Run the project
1. We need to tell system where to find the dynamic libraries of our thirdparty dependencies
    * On Linux, instead of running Matlab directly from command line, we need to run `LD_LIBRARY_PATH=/Abs/path/to/ElecGen/thirdparty/build/lib matlab`
2. Since the mex code is compiled outside of matlab, it will be linked to the system libstdc++. So if your system has a newer version of libstdc++, mex would also fail to run directly from Matlab.
    * On Linux, we need to run `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 matlab` to tell Matlab to use the system-wide libstdc++.
3. Sometimes the above two cases happen together.
    * So on Linux `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 LD_LIBRARY_PATH=/Abs/path/to/ElecGen/thirdparty/build/lib matlab`
