import subprocess

# 실행할 스크립트 리스트
scripts = [
    ("/home/lmc/gt3_ws/pcl_visualization/pcl_visualization.py", "python3"),  # 파이썬 파일
    ("/home/lmc/gt3_ws/pcl_visualization/pcl_normal_vector.py", "python3"),   # 파이썬 파일
    ("/home/lmc/sh/test.sh", None),                                      # 쉘 스크립트
]

processes = []
for script, interpreter in scripts:
    if interpreter:
        process = subprocess.Popen([interpreter, script])  # 파이썬 파일은 python3로 실행
    else:
        process = subprocess.Popen([script])                # 쉘 스크립트는 직접 실행
    processes.append(process)

# 모든 프로세스가 종료되지 않도록 대기
for process in processes:
    process.wait()
