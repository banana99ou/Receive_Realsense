import socket, numpy as np, cv2

HOST, PORT = '127.0.0.1', 5006
W, H = 320, 240  # must match sender

sock = socket.socket()
sock.bind((HOST, PORT))
sock.listen(1)
conn, _ = sock.accept()
print("Client connected")

while True:
    data = b''
    while len(data) < W*H:
        chunk = conn.recv(W*H - len(data))
        if not chunk:
            break
        data += chunk
    if len(data) != W*H:
        break

    # reconstruct & display
    gray = np.frombuffer(data, dtype=np.uint8).reshape((W, H)).T
    cv2.imshow('frame', gray)
    if cv2.waitKey(1) == 27:
        break

conn.close()
sock.close()
cv2.destroyAllWindows()
