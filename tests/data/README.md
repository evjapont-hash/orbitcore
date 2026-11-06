# Validation data (not committed: download it yourself)

Get the supplemental files for "Revisiting Spacetrack Report #3" (Vallado, Crawford,
Hujsak, Kelso; AIAA 2006-6753) from CelesTrak's publications page
(https://celestrak.org/publications/AIAA/2006-6753/) and place:

- `SGP4-VER.TLE`  -- the test element sets
- `tcppver.out`   -- expected TEME positions/velocities from the reference C++ implementation

Deep-space satellites in the set are skipped (orbitcore is near-Earth only). If the layout of
your `tcppver.out` differs from what `read_expected()` in `test_sgp4_vallado.cpp` assumes,
adjust the parser; check the first few lines of the file.
