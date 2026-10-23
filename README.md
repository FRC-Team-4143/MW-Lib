# assets/trajgen-issues

Images and benchmark sources referenced by the on-the-fly trajectory generation issues.
Not code; nothing here is built. Don't merge this branch.

- `trajgen/img/` — pictures embedded in the issues
- `trajgen/bench/bench.cpp` — TrajoptLib solve-time benchmark (mainbot's choreo_config.chor values)
- `trajgen/bench/dump.cpp` — dumps TrajoptLib solutions as JSON for the plots
- `trajgen/bench/make_plots.py` — renders the plots from that JSON

Build: TrajoptLib from SleipnirGroup/Choreo `trajoptlib/` (CMake, C++23 with `<print>`: GCC 14+ or clang/libc++ 17+),
then `c++ -std=c++23 -O3 bench.cpp -I<trajoptlib>/include <sleipnir/eigen includes> libTrajoptLib.a libSleipnir.a`.
