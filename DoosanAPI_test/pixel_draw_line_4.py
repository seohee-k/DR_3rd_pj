import cv2
import numpy as np

def find_path(points):
    points = points.copy()
    path = [points[0]]
    visited = np.zeros(len(points), dtype=bool)
    visited[0] = True

    for _ in range(1, len(points)):
        last_point = path[-1]
        dists = np.linalg.norm(points - last_point, axis=1)
        dists[visited] = np.inf
        next_idx = np.argmin(dists)
        path.append(points[next_idx])
        visited[next_idx] = True

    return path

# 이미지 읽기
img = cv2.imread('nemo.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 더 강력한 대비 강조 및 이진화
gray = cv2.medianBlur(gray, 3)  # 노이즈 제거
thresh = cv2.adaptiveThreshold(
    gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
    cv2.THRESH_BINARY_INV, 11, 3
)

# 윤곽선 + 내부 포함
contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

points = []
for cnt in contours:
    # 더 정밀한 윤곽 보존
    approx = cv2.approxPolyDP(cnt, 1.5, True)
    for pt in approx:
        points.append(tuple(pt[0]))

points = np.unique(np.array(points), axis=0)

if len(points) < 2:
    print("Too few points to draw path.")
else:
    path = find_path(points)

    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 1)

# 시각화
cv2.imshow('Detailed Path', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
