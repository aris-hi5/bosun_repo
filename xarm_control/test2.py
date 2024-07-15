import cv2
import numpy as np

# 비디오 캡처 객체 생성 (웹캠)
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# 배경 사진 캡처
print("Press 'b' to capture the background frame.")
while True:
    ret, background = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    cv2.imshow('Background', background)
    key = cv2.waitKey(1)
    if key == ord('b'):  # 'b' 키를 누르면 배경 사진 캡처
        cv2.imwrite('background_frame.png', background)
        print("Background frame captured and saved as 'background_frame.png'")
        break

# 캡처된 배경 사진 읽기
background = cv2.imread('background_frame.png')
if background is None:
    print("Error: Could not read background frame.")
    exit()

# 배경 사진을 HSV 색상 공간으로 변환
background_hsv = cv2.cvtColor(background, cv2.COLOR_BGR2HSV)

# 투명 컵을 나타내는 HSV 색상 범위 (예시)
lower_hsv = np.array([0, 0, 200])  # 낮은 범위 값 (색상, 채도, 명도)
upper_hsv = np.array([180, 50, 255])  # 높은 범위 값 (색상, 채도, 명도)

# 배경 사진에서 특정 범위의 색상에 해당하는 영역 마스킹
background_mask = cv2.inRange(background_hsv, lower_hsv, upper_hsv)

while True:
    # 비디오에서 프레임 읽기
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Could not read frame.")
        break
    
    # BGR 이미지를 HSV 색상 공간으로 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 실시간 이미지에서 특정 범위의 색상에 해당하는 영역 마스킹
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    
    # 배경 마스크와 현재 프레임 마스크의 차이 계산
    difference = cv2.absdiff(background_mask, mask)
    
    # 차이 이미지에서 윤곽선 찾기
    contours, _ = cv2.findContours(difference, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 윤곽선이 있는 영역에 박스 그리기
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # 작은 잡음 제거
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    # 결과 이미지를 윈도우에 표시
    cv2.imshow('Result', frame)
    key = cv2.waitKey(1)
    if key == ord('c'):  # 'c' 키를 누르면 캡처
        # 프레임을 파일로 저장
        cv2.imwrite('captured_frame.png', frame)
        print("Frame captured and saved as 'captured_frame.png'")
    elif key == ord('q'):  # 'q' 키를 누르면 종료
        print("Exiting...")
        break

# 비디오 캡처 객체와 윈도우 해제
cap.release()
cv2.destroyAllWindows()
