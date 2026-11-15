"""
Tune the tape HSV range on a photo from the car's camera (run on the laptop).

    python tune_hsv.py test_photo.jpg

Move the trackbars until only the tape is white in the mask (inside the ROI box).
Press p to print the values, q/Esc to quit (also prints them).
Paste the printed lines into config.py.
"""
import sys

import cv2
import numpy as np

import config

WIN = "tune_hsv"


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    img = cv2.imread(sys.argv[1])
    if img is None:
        sys.exit(f"cannot read {sys.argv[1]}")
    img = cv2.resize(img, (config.CAM_WIDTH, config.CAM_HEIGHT))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    lo, hi = config.HSV_LOWER, config.HSV_UPPER
    for name, val, mx in (("H min", lo[0], 179), ("S min", lo[1], 255), ("V min", lo[2], 255),
                          ("H max", hi[0], 179), ("S max", hi[1], 255), ("V max", hi[2], 255)):
        cv2.createTrackbar(name, WIN, int(val), mx, lambda _: None)

    def values():
        g = lambda n: cv2.getTrackbarPos(n, WIN)
        return (g("H min"), g("S min"), g("V min")), (g("H max"), g("S max"), g("V max"))

    def dump():
        l, h = values()
        print(f"HSV_LOWER = {l}\nHSV_UPPER = {h}")

    while True:
        l, h = values()
        mask = cv2.inRange(hsv, np.array(l, np.uint8), np.array(h, np.uint8))
        masked = cv2.bitwise_and(img, img, mask=mask)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        for v in (img, mask_bgr, masked):
            cv2.rectangle(v, (0, config.ROI_TOP), (config.CAM_WIDTH - 1, config.ROI_BOTTOM - 1), (255, 255, 0), 1)
        roi_px = cv2.countNonZero(mask[config.ROI_TOP:config.ROI_BOTTOM])
        view = np.hstack([img, mask_bgr, masked])
        cv2.putText(view, f"ROI mask px: {roi_px} (min {config.MIN_MASK_PIXELS})", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow(WIN, view)
        k = cv2.waitKey(30) & 0xFF
        if k == ord("p"):
            dump()
        elif k in (ord("q"), 27):
            dump()
            break
        if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            dump()
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
