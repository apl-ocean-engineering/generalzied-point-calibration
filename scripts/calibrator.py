#!/usr/bin/env python
import numpy as np
import cv2
import glob
import argparse
import constants
import logging


class Calibrator:
    def __init__(self, dimensions_file, display_images):
        # Parse dimesions as: np.array((x1,y1, 0),(x2,y2, 0),..,(xn,yn, 0))
        self.dimensions = self._load_dimensions(dimensions_file)
        self.display_images = display_images
        self.K = None  # Intrinsic matrix
        self.d = None  # distortion matrix
        self.R = None  # 3X3 Rotation matrix
        self.t = None  # 3X1 Translation matrix

        # self.points is nested list of points:
        # np.array([(img1_x1, img1_y1), (img1_x2, img1_y2), ...,
        # (img1_xn, img1_yn)],
        # [(img2_x1, img2_y1)],.., [(imgn_x1, imgn_y1)])
        self.points = []

        # self.images is list of strings pointing to img,
        # only used if display_images is True and images are found
        self.images = []

    def parse_points(self, path):
        f = open(path, 'r')
        points = []
        for i, line in enumerate(f):
            img_points = []
            if i == 0:
                continue
            line = line.rstrip()

            # Randomly like 50 lines of Nan?
            if np.isnan(float(line.split(',')[0])):
                break
            for i in range(0, len(line.split(',')), 2):
                x = float(line.split(',')[i])
                y = float(line.split(',')[i+1])

                # Not sure why the line ends with nan...
                if np.isnan(x) or np.isnan(y):
                    break
                img_points.append((x, y))

            points.append(np.float32(img_points))
        return points

    def parse_images(self, path):
        self.images = glob.glob(path + '/*')

    def calibrate(self, path):
        self.points = self.parse_points(path)
        if self.display_images:
            cv2.named_window(constants.img_name, cv2.WINDOW_NORMAL)
        for i, img_points in enumerate(self.points):
            img = None
            if self.display_images and i < len(self.images):
                img = cv2.imread(self.images[i])
                if img is None:
                    logging.warn("Specified image does not exist")

            elif self.display_images:
                logging.warn("Display images is specified, \
                              but no image found. Not displaying")

        if self.display_images and img is not None:
            for point in img_points:
                cv2.circle(img, (int(point[0]), int(point[1])),
                           (255, 0, 0))
            cv2.imshow(constants.img_name, img)
            cv2.waitKey(constants.wait_key)

        objpoints = [np.float32(self.dimensions)
                     for i in range(len(self.points))]

        ret, K, d, rvecs, tvecs = cv2.calibrateCamera(
                    objpoints, np.array(self.points),
                    constants.img_size, None, None)

        self.K = K
        self.d = d

    def extrinsic_calibration(self, extrinsic_points_path):
        if self.K is None or self.d is None:
            logging.warning("Have not yet gotten an intrinsic calibration."
                            " Returning ")
            return
        extrinsic_points = self.parse_points(extrinsic_points_path)
        objpoints = np.float32(self.dimensions)

        extrinsic_points = cv2.undistort(extrinsic_points[0], self.K, self.d)

        ret, rvec, tvec = cv2.solvePnP(objpoints, extrinsic_points,
                                       self.K, self.d)

        self.R = cv2.Rodrigues(rvec)[0]
        self.t = tvec

    def dump(self, path):
        if path[-1] != '/':
            path += '/'
        if self.K is not None:
            np.savetxt(path + "K.csv", self.K, delimiter=",")
        else:
            logging.warning("Not saving K, not yet calculated")
        if self.d is not None:
            np.savetxt(path + "d.csv", self.d, delimiter=",")
        else:
            logging.warning("Not saving distortion, not yet calculated")
        if self.R is not None:
            np.savetxt(path + "R.csv", self.R, delimiter=",")
        else:
            logging.warning("Not saving R, not yet calculated")
        if self.t is not None:
            np.savetxt(path + "t.csv", self.t, delimiter=",")
        else:
            logging.warning("Not saving translation, not yet calculated")


    def _load_dimensions(self, dimensions_file):
        calibration_dimensions = open(dimensions_file, 'r')
        points = calibration_dimensions.readlines()[0].split(';')

        dimensions = [(float(val.split(',')[0]),
                       float(val.split(',')[1]),
                       0)
                      for val in points]

        return np.array(dimensions, dtype=np.float64)


if __name__ == '__main__':
    parser = argparse.ArgumentParser("calibrator")
    parser.add_argument('-d', '--dimensions',
                        default='../data/dimensions/dimensions.csv',
                        help="Path to dimensions file")
    parser.add_argument('-p', '--points',
                        default='../data/points/input.csv',
                        help="Path to points file for intrnsic calibration")
    parser.add_argument('-ep', '--extrinsic_points',
                        default='../data/points/extrinsic.csv',
                        help="Path to points file for extrinsic calibration")
    parser.add_argument('-di', '--display_images', action='store_true',
                        help="Display images")
    parser.add_argument('-images', '--i',
                        default='../data/images/',
                        help="Images path, only is display images is true")
    parser.add_argument('-s', '--save',
                        action='store_true',
                        help="Flag to save data")
    parser.add_argument('-o', '--outfile',
                        default='../output/',
                        help="Path to save output data, if dump is true")

    args = parser.parse_args()

    calibrator = Calibrator(args.dimensions,
                            args.display_images)

    if args.display_images:
        calibrator.parse_images(args.images)

    calibrator.calibrate(args.points)

    calibrator.extrinsic_calibration(args.extrinsic_points)

    if args.save:
        calibrator.dump(args.outfile)
