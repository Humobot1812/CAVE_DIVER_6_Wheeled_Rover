#!/usr/bin/env python3
# SIH GCS Launcher
# Usage: python3 launch_gcs.py [--host 0.0.0.0] [--port 5001]
import os, sys, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
SRV = os.path.join(HERE, 'gcs_server.py')

def check_deps():
    missing = []
    for pkg, pip_name in [('flask','flask'),('cv2','opencv-python'),('numpy','numpy')]:
        try:
            __import__(pkg)
        except ImportError as e:
            print(f"[!] Warning: failed to import '{pkg}': {e}")
            missing.append(pip_name)
    if missing:
        print(f"[!] Missing or broken dependencies: pip install {' '.join(missing)}")
        return False
    return True

def main():
    p = argparse.ArgumentParser(description='SIH GCS Launch Tool')
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=5001)
    args = p.parse_args()
    if not check_deps():
        sys.exit(1)
    print(f"[+] SIH GCS starting at http://localhost:{args.port}")
    os.execv(sys.executable, [sys.executable, SRV, '--host', args.host, '--port', str(args.port)])

if __name__ == '__main__':
    main()
