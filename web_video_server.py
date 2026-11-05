import cv2

url = "http://pi5vdt:8080/stream?topic=/hpad/annotated&type=mjpeg&quality=60"
cap = cv2.VideoCapture(url)
while True:
    ok, frame = cap.read()
    if not ok:
        break
    cv2.imshow("hpad", frame)
    if cv2.waitKey(1) == 27:
        break
cap.release()
cv2.destroyAllWindows()