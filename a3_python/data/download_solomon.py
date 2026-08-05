"""下载 Solomon VRPTW 原始实例数据。

数据来源: CervEdin/solomon-vrptw-benchmarks (GitHub)
原始发布: Solomon (1987), Operations Research 35(2), 254-265.

用法 (需要 SOCKS5 代理):
    python a3_python/data/download_solomon.py

输出: a3_python/data/solomon/*.json + *.txt (完整 100 客户实例)
"""

import os, sys, json, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(HERE, "solomon")

URLS = [
    # JSON format (CervEdin) — 便于程序解析
    ("https://raw.githubusercontent.com/CervEdin/solomon-vrptw-benchmarks/main/r/1/r101.json", "r101.json"),
    ("https://raw.githubusercontent.com/CervEdin/solomon-vrptw-benchmarks/main/c/1/c101.json", "c101.json"),
    ("https://raw.githubusercontent.com/CervEdin/solomon-vrptw-benchmarks/main/rc/1/rc101.json", "rc101.json"),
    # TXT format (yue957) — 便于 vrplib 读取
    ("https://raw.githubusercontent.com/yue957/py-ga-VRPTW/master/data/text/R101.txt", "R101.txt"),
    ("https://raw.githubusercontent.com/yue957/py-ga-VRPTW/master/data/text/C101.txt", "C101.txt"),
    ("https://raw.githubusercontent.com/yue957/py-ga-VRPTW/master/data/text/RC101.txt", "RC101.txt"),
]


def setup_proxy(host: str = "127.0.0.1", port: int = 10808):
    """尝试配置 SOCKS5 代理 (如果 PySocks 可用)"""
    try:
        import socks, socket
        socks.set_default_proxy(socks.SOCKS5, host, port)
        socket.socket = socks.socksocket
        print(f"[proxy] SOCKS5 {host}:{port} configured")
        return True
    except ImportError:
        print("[proxy] PySocks not installed, using direct connection")
        return False


def download(url: str, dest: str) -> bool:
    try:
        r = urllib.request.urlopen(url, timeout=30)
        if r.status == 200:
            content = r.read()
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            with open(dest, "wb") as f:
                f.write(content)
            print(f"  OK  {os.path.basename(dest)} ({len(content)} bytes)")
            return True
        else:
            print(f"  FAIL {os.path.basename(dest)}: HTTP {r.status}")
            return False
    except Exception as e:
        print(f"  FAIL {os.path.basename(dest)}: {type(e).__name__}: {e}")
        return False


def main():
    # Try SOCKS5 proxy first
    setup_proxy()

    print(f"Downloading Solomon instances to {DATA_DIR}...\n")
    ok = 0
    for url, filename in URLS:
        dest = os.path.join(DATA_DIR, filename)
        if os.path.exists(dest):
            print(f"  SKIP {filename} (already exists)")
            ok += 1
            continue
        if download(url, dest):
            ok += 1

    print(f"\nDownloaded {ok}/{len(URLS)} files.")
    if ok < len(URLS):
        print("Some files failed. Retry with proxy or check network.")


if __name__ == "__main__":
    main()
