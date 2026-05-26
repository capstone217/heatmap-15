from picarx import Picarx
import time

# 1. 파이카 초기화 (저장된 영점 보정치 자동 불러옴)
px = Picarx()

try:
    print("1단계: 바퀴를 정면(0도)으로 자동 정렬합니다.")
    # 바퀴가 어디로 꺾여 있든 강제로 진짜 앞을 바라보게 만듭니다.
    px.set_dir_servo_angle(0)
    
    # 모터가 물리적으로 회전해서 정렬할 시간을 1초 동안 줍니다.
    time.sleep(1.0)
    
    print("2단계: 정렬 완료! 속도 40으로 3초간 직진합니다.")
    # 앞바퀴는 0도를 유지한 상태에서 뒷바퀴 모터를 전진시킵니다.
    px.forward(40)
    time.sleep(3.0) # 3초 동안 주행 유지

finally:
    # 코드가 끝나거나 중간에 강제 종료(Ctrl+C)되어도 안전하게 차를 멈춥니다.
    print("3단계: 주행 종료 및 안전 정지.")
    px.stop()
