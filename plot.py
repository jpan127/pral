import glob
import os
import random

from PyQt5.QtCore import Qt
from PyQt5.QtGui import *

os.makedirs("plots", exist_ok=True)

# Clear plots because pyqt5 does not overwrite
for fname in glob.iglob("plots/*"):
    os.remove(fname)

# For each data file, convert
for fname in glob.iglob("data/*_plot.txt"):
    image = QImage(10000, 10000, QImage.Format_RGB16)
    painter = QPainter()
    image.fill(Qt.black)
    painter.begin(image)
    painter.setPen(QPen(Qt.white, 2))

    # Read data and plot lien segments
    with open(fname) as data:
        for line in data.readlines():
            start_x, start_y, end_x, end_y = line.split(",")
            painter.drawLine(float(start_x), float(start_y), float(end_x), float(end_y))

    # Export image to file
    painter.end()
    image.save(fname.replace("data", "plots").replace("txt", "png"), 'png')
