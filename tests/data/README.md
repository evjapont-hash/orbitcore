# Validation data

These files come from the supplemental package for "Revisiting Spacetrack Report #3"
(Vallado, Crawford, Hujsak, Kelso; AIAA 2006-6753), downloaded from
https://celestrak.org/publications/AIAA/2006-6753/ (AIAA-2006-6753.zip).
CelesTrak's FAQ for the paper states that there is no license on the code and test
cases and asks that users cite the source, which this project does.

- `SGP4-VER.TLE` -- the verification element sets
  (`sgp4/cpp/testsgp4/SGP4-VER.TLE` in the zip)
- `tmatverDec2015.out` -- expected TEME positions and velocities from the reference
  MATLAB implementation (`sgp4/mat/tmatverDec2015.out` in the zip). Rows are:
  minutes since epoch, x y z (km), vx vy vz (km/s); extra columns are ignored.

Deep-space satellites (period >= 225 min) in the set are skipped, because orbitcore
implements only the near-Earth model.