# generalzied-point-calibration

Calibration scripts to help calibrate cameras without checkerboard pattern

## Requirments
1. opencv
2. numpy
3. python 2/3

## Input data
1. Calibration target dimensions. Input dimensions as (x1,y1;x2,y2;...;xn,yn) for n-dimensional calibration target. Must be in same order as clicked points. Default location: data/dimensions/dimensions.csv
2. Saved clicked-points for intrinsic calibration. Rows of saved points in form [(x1, y1), (x2,y2),..,(xn,yn)] (default from [DLTdv digitizing tool](http://biomech.web.unc.edu/dltdv/)). Default location: data/points/input.csv
3. Saved clicked-points for intrinsic calibration. Row(s) of saved points in same for as input.csv, but only first line used, for extrinsic calibration. Should be of identified points with target at the middle. Default location: data/points/extrinsic.csv
4. Images for calibration display (optional). Could add images to display clicked calibration points, for verification. Default location: data/images/

## Output data
If run with --save, K, d, R, t will be saved to output/

## Running
scripts/calibrator.py will first calculate intrinsic calibration, and then estimate [R,t]
