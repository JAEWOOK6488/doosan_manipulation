#! python3.12
"""설치: install.py 더블클릭, 또는 명령 프롬프트에서  py -3.12 install.py

가상환경(.venv)을 만들고 패키지(mujoco, numpy, matplotlib)를 설치한 뒤,
Windows 에서는 실행용 run.bat / evaluate.bat 을 이 폴더에 만든다.
(메일 보안 정책상 .bat 파일은 배포 파일에 넣지 않고 설치할 때 만든다)
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WIN = os.name == "nt"
BAT = """@echo off
chcp 65001 >nul
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
.venv\\Scripts\\python.exe {script} %*
"""


def done(code):
    if WIN and sys.stdin is not None and sys.stdin.isatty():   # 더블클릭으로 연 창이 바로 닫히지 않게
        try:
            input("\n엔터를 누르면 창이 닫힙니다.")
        except EOFError:
            pass
    sys.exit(code)


def main():
    os.chdir(HERE)
    print(f"[1/3] Python 확인: {sys.version.split()[0]}")
    if sys.version_info[:2] != (3, 12):
        if WIN:
            print("      Python 3.12 로 다시 실행합니다 (py -3.12)")
            try:
                sys.exit(subprocess.call(["py", "-3.12", os.path.abspath(__file__)]))
            except FileNotFoundError:
                pass
        print("Python 3.12 이 필요합니다. 가이드 부록 A 1단계를 먼저 하세요.")
        done(1)
    print("[2/3] 가상환경 만들기 (.venv)")
    if not os.path.isdir(".venv"):
        subprocess.check_call([sys.executable, "-m", "venv", ".venv"])
    py = os.path.join(".venv", "Scripts", "python.exe") if WIN else os.path.join(".venv", "bin", "python")
    print("[3/3] 패키지 설치 (약 1 분, 인터넷 필요)")
    if subprocess.call([py, "-m", "pip", "install", "--disable-pip-version-check", "-q", "-r", "requirements.txt"]):
        print("패키지 설치 실패: 인터넷(PyPI) 연결을 확인하세요.")
        done(1)
    subprocess.check_call([py, "-c", "import mujoco, numpy, matplotlib; print('설치 완료: mujoco', mujoco.__version__)"])
    if WIN:
        for name, script in (("run.bat", "run.py"), ("evaluate.bat", "evaluate.py")):
            with open(name, "w", encoding="ascii", newline="\r\n") as f:
                f.write(BAT.format(script=script))
        print("실행 파일 생성: run.bat, evaluate.bat  →  이제  run.bat --box 8  로 실행합니다.")
    else:
        print("실행:  .venv/bin/python run.py --box 8")
    done(0)


if __name__ == "__main__":
    main()
