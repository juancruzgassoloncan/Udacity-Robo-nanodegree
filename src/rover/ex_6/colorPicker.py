import cv2
import numpy as np
import glob
# cap = cv2.VideoCapture(0)


def nothing(x):
    pass


# Creamos una ventana llamada 'image' en la que habra todos los sliders
cv2.namedWindow('image')
cv2.createTrackbar('Hue Min', 'image', 0, 255, nothing)
cv2.createTrackbar('Hue Max', 'image', 0, 255, nothing)
cv2.createTrackbar('Sat Min', 'image', 0, 255, nothing)
cv2.createTrackbar('Sat Max', 'image', 0, 255, nothing)
cv2.createTrackbar('Val Min', 'image', 0, 255, nothing)
cv2.createTrackbar('Val Max', 'image', 0, 255, nothing)
# set initial values
cv2.setTrackbarPos('Hue Min', 'image', 0)
cv2.setTrackbarPos('Hue Max', 'image', 255)
cv2.setTrackbarPos('Sat Min', 'image', 0)
cv2.setTrackbarPos('Sat Max', 'image', 255)
cv2.setTrackbarPos('Val Min', 'image', 0)
cv2.setTrackbarPos('Val Max', 'image', 160)

path = '../../data/IMG/*'
img_list = glob.glob(path)
# Grab a random image and display it
idx = np.random.randint(0, len(img_list) - 1)
frame = cv2.imread(img_list[idx])


while(1):
    # idx = np.random.randint(0, len(img_list) - 1)
    # frame = cv2.imread(img_list[idx])
    # _,frame = cap.read() #Leer un frame
    # Convertirlo a espacio de color HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Los valores maximo y minimo de H,S y V se guardan en funcion de la posicion de los sliders
    hMin = cv2.getTrackbarPos('Hue Min', 'image')
    hMax = cv2.getTrackbarPos('Hue Max', 'image')
    sMin = cv2.getTrackbarPos('Sat Min', 'image')
    sMax = cv2.getTrackbarPos('Sat Max', 'image')
    vMin = cv2.getTrackbarPos('Val Min', 'image')
    vMax = cv2.getTrackbarPos('Val Max', 'image')

    # Se crea un array con las posiciones minimas y maximas
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Deteccion de colores
    mask = cv2.inRange(hsv, lower, upper)
    roi = np.zeros((frame.shape[0],frame.shape[1]),dtype=np.uint8)
    roi[70:, :] = 1
    mask
    mask_roi = mask * roi

    threshed = cv2.bitwise_and(frame, frame, dst=None, mask=mask_roi)

    # Mostrar los resultados y salir
    cv2.imshow('camara', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('thresed', threshed)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    elif k == 97:
        idx = np.random.randint(0, len(img_list) - 1)
        frame = cv2.imread(img_list[idx])

cv2.destroyAllWindows()
