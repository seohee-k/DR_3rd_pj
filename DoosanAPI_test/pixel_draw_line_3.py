import cv2
import numpy as np

# 이미지 읽기
img = cv2.imread('totoro.png', cv2.IMREAD_COLOR)  # 이미지 파일명 수정 
gray = cv2.imread('totoro.png', cv2.IMREAD_GRAYSCALE)

# 전처리: 이진화 및 에지 검출
_, threshold = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
threshold = cv2.bitwise_not(threshold)  # 배경을 검정, 캐릭터를 흰색으로
# 모폴로지 연산: 에지 연결 및 잡음 제거
kernel = np.ones((5, 5), np.uint8)
threshold = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel, iterations=2)  # 에지 연결
threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel, iterations=1)  # 잡음 제거
edges = cv2.Canny(threshold, 5, 30, apertureSize=5)  # Canny 임계값 최적화

# Hough Transform으로 직선 탐지
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=15, minLineLength=3, maxLineGap=50)

# 윤곽선 검출로 곡선 및 끊어진 부분 보완
contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 이미지에 선 그리기
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 초록색 선
        print(f"Hough Line coordinates: ({x1}, {y1}) to ({x2}, {y2})")

# 윤곽선으로 보완 (Hough로 그려지지 않은 부분)
for cnt in contours:
    approx = cv2.approxPolyDP(cnt, 2, True)  # 윤곽선을 단순화
    for i in range(len(approx) - 1):
        x1, y1 = approx[i][0]
        x2, y2 = approx[i + 1][0]
        # Hough로 이미 그려진 선과 중복되지 않도록 체크 (옵션)
        if not any((x1 == x and y1 == y and x2 == x2 and y2 == y2) for line in lines for x, y, x2, y2 in [line[0]]):
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 초록색 선
            print(f"Contour Line coordinates: ({x1}, {y1}) to ({x2}, {y2})")

# 결과 이미지 표시
cv2.imshow('Detected Lines', img)
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()