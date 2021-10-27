# importa los paquetes necesarios para el analísis.
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
# construir los argumentos de invocación
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--images", required=True, help="path to images directory")
args = vars(ap.parse_args())
# inicializar el descriptor HOG/detector de peatones
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
# iterar sobre la ruta de las imagenes.
for imagePath in paths.list_images(args["images"]):
    # cargar las imagenes y redimensionarlas para reducir el tiempo de analísis
    # y mejorar la precisión de la detección.
    image = cv2.imread(imagePath)
    image = imutils.resize(image, width=min(400, image.shape[1]))
    orig = image.copy()
    # detección de personas en la imagen
    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        padding=(8, 8), scale=1.05)
    # dibujar los rectangulos limitantes originales.
    for (x, y, w, h) in rects:
        cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
    # aplicar supresión non-maxima a los rectangulos limitantes usando un
    # umbral de superposición bastante grande para tratar de mantener 	
    # el hecho que aún son personas. 
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
    # dibujar los rectangulos limitantes finales
    for (xA, yA, xB, yB) in pick:
        cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
    # añadir información al respecto de los rectangulos.
    filename = imagePath[imagePath.rfind("/") + 1:]
    print("[INFO] {}: {} original boxes, {} after suppression".format(
        filename, len(rects), len(pick)))
while(True):
    # mostrar las imagenes
    cv2.imshow("Before NMS", orig)
    cv2.imshow("After NMS", image)
    if cv2.waitKey(0) and 0xFF == ord('q'):
        break
