import cv2
import numpy as np

# 비디오 캡처 객체 생성 (웹캠)
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

while True:
    # 비디오에서 프레임 읽기
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Could not read frame.")
        break
    
    # BGR 이미지를 HSV 색상 공간으로 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 투명 컵을 나타내는 HSV 색상 범위 (예시)
    lower_hsv = np.array([0, 0, 200])  # 낮은 범위 값 (색상, 채도, 명도)
    upper_hsv = np.array([180, 50, 255])  # 높은 범위 값 (색상, 채도, 명도)
    
    # HSV 이미지에서 특정 범위의 색상에 해당하는 영역 마스킹
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    
    # 마스크를 사용하여 원본 이미지에서 투명 컵 부분을 추출
    result = cv2.bitwise_and(frame, frame, mask=mask)
    
    # 결과 이미지를 윈도우에 표시
    cv2.imshow('Result', result)
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
