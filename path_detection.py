import cv2
import numpy as np
import math
from collections import defaultdict
import serial
import time

IMAGE_PATH = "final.jpeg"
ARUCO_TYPE = "DICT_4X4_1000"

THRESH_BINARY = 200
MAX_GAP = 3
MAX_SEARCH_FROM_EDGE = 5
STEP_PIXELS = 1

SERIAL_PORT = 'COM3'
BAUD_RATE = 9600

ARUCO_DICT = {
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}

def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def detect_markers(img):
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[ARUCO_TYPE])
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    corners, ids, _ = detector.detectMarkers(img)
    centers, boxes = {}, {}

    if ids is None:
        return centers, boxes

    ids = ids.flatten()
    for c, mid in zip(corners, ids):
        c = c.reshape((4, 2))
        xs, ys = c[:, 0], c[:, 1]

        xmin, xmax = int(xs.min()), int(xs.max())
        ymin, ymax = int(ys.min()), int(ys.max())

        cx = int(xs.mean())
        cy = int(ys.mean())

        centers[int(mid)] = (cx, cy)
        boxes[int(mid)] = (xmin, ymin, xmax, ymax)

    return centers, boxes

def point_in_box(pt, box, margin=2):
    x, y = pt
    xmin, ymin, xmax, ymax = box
    return xmin - margin <= x <= xmax + margin and ymin - margin <= y <= ymax + margin

def find_connection_from_side(bw, start_pt, direction, my_id, boxes):
    h, w = bw.shape
    dx, dy = direction
    x, y = start_pt

    for _ in range(MAX_SEARCH_FROM_EDGE):
        if not (0 <= x < w and 0 <= y < h):
            return None
        if bw[y, x] == 255:
            break
        x += dx
        y += dy
    else:
        return None

    gap = 0
    while 0 <= x < w and 0 <= y < h:
        for other_id, box in boxes.items():
            if other_id != my_id and point_in_box((x, y), box):
                return other_id

        if bw[y, x] == 255:
            gap = 0
        else:
            gap += 1
            if gap > MAX_GAP:
                return None

        x += dx * STEP_PIXELS
        y += dy * STEP_PIXELS

    return None

def build_graph_from_rays(bw, centers, boxes):
    h, w = bw.shape
    edges = set()

    for mid, (cx, cy) in centers.items():
        xmin, ymin, xmax, ymax = boxes[mid]

        for x in range(xmin, xmax + 1):
            for d in [((x, ymin - 1), (0, -1)), ((x, ymax + 1), (0, 1))]:
                if 0 <= d[0][1] < h:
                    o = find_connection_from_side(bw, d[0], d[1], mid, boxes)
                    if o:
                        edges.add(tuple(sorted((mid, o))))

        for y in range(ymin, ymax + 1):
            for d in [((xmin - 1, y), (-1, 0)), ((xmax + 1, y), (1, 0))]:
                if 0 <= d[0][0] < w:
                    o = find_connection_from_side(bw, d[0], d[1], mid, boxes)
                    if o:
                        edges.add(tuple(sorted((mid, o))))

    adj = defaultdict(list)
    for a, b in edges:
        adj[a].append(b)
        adj[b].append(a)

    return dict(adj), edges

def traverse_path(adj, centers):
    if not adj:
        return []

    start = min(adj.keys())
    visited = {start}
    path = [start]
    cur = start

    while True:
        nxt = [n for n in adj[cur] if n not in visited]
        if not nxt:
            break
        nxt = min(nxt, key=lambda n: dist(centers[cur], centers[n]))
        visited.add(nxt)
        path.append(nxt)
        cur = nxt

    return path

def is_collinear(A, B, C, angle_thresh_deg=8):
    AB = np.array([B[0] - A[0], B[1] - A[1]], dtype=float)
    BC = np.array([C[0] - B[0], C[1] - B[1]], dtype=float)

    normAB = np.linalg.norm(AB)
    normBC = np.linalg.norm(BC)
    if normAB == 0 or normBC == 0:
        return False

    cos_angle = np.dot(AB, BC) / (normAB * normBC)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.degrees(np.arccos(cos_angle))

    return angle < angle_thresh_deg

def compute_moves(path, centers):
    if len(path) < 2:
        return []

    moves = ["S"]
    straight_count = 1

    for i in range(len(path) - 2):
        A = centers[path[i]]
        B = centers[path[i + 1]]
        C = centers[path[i + 2]]

        if is_collinear(A, B, C):
            straight_count += 1
            if straight_count >= 2:
                moves.append("O")
        else:
            straight_count = 1
            AB = (B[0] - A[0], B[1] - A[1])
            BC = (C[0] - B[0], C[1] - B[1])
            cross = AB[0] * BC[1] - AB[1] * BC[0]
            moves.append("R" if cross > 0 else "L")

    moves.append("F")
    return moves

def send_moves_to_arduino(moves):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        ser.write((",".join(moves) + "\n").encode())
        ser.close()
        print("Sent:", moves)
    except Exception as e:
        print("Serial Error:", e)

def visualize(img, centers, edges, path, moves):
    vis = img.copy()

    for a, b in edges:
        cv2.line(vis, centers[a], centers[b], (0, 0, 255), 3)

    for mid, (cx, cy) in centers.items():
        cv2.circle(vis, (cx, cy), 6, (0, 255, 0), -1)
        cv2.putText(vis, str(mid), (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    for i in range(len(path) - 1):
        cv2.arrowedLine(vis, centers[path[i]], centers[path[i + 1]],
                        (255, 0, 0), 3, tipLength=0.15)

    cv2.putText(vis, "Path: " + "->".join(map(str, path)), (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)

    cv2.putText(vis, "Moves: " + ",".join(moves), (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)

    cv2.imshow("Detected Path", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    img = cv2.imread(IMAGE_PATH)
    if img is None:
        print("Image not found")
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, THRESH_BINARY, 255, cv2.THRESH_BINARY_INV)

    centers, boxes = detect_markers(img)
    adj, edges = build_graph_from_rays(bw, centers, boxes)
    path = traverse_path(adj, centers)
    moves = compute_moves(path, centers)

    print("Path:", path)
    print("Moves:", moves)

    send_moves_to_arduino(moves)
    visualize(img, centers, edges, path, moves)

if __name__ == "__main__":
    main()
