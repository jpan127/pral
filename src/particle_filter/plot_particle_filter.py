import glob
import os
import random

from PyQt5.QtCore import Qt
from PyQt5.QtGui import *

BEFORE_CSV = "build/test_particle_filter_before.txt"
AFTER_CSV = "build/test_particle_filter_after.txt"

os.makedirs("plots", exist_ok=True)

# Clear plots because pyqt5 does not overwrite
for fname in glob.iglob("plots/*"):
    os.remove(fname)

for csv in [BEFORE_CSV, AFTER_CSV]:
    image = QImage(100, 100, QImage.Format_RGB16)
    painter = QPainter()
    image.fill(Qt.white)
    painter.begin(image)
    painter.setPen(QPen(Qt.black, 2))

    # Read data and plot line segments
    with open(csv) as data:
        for line in data.readlines():
            x, y = line.split(",")
            painter.drawPoint(float(x), image.height() - float(y))

    # Export image to file
    painter.end()
    image.save(csv.replace("build", "plots").replace("txt", "png"), 'png')
