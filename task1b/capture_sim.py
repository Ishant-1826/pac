import subprocess
import struct
import sys
from PIL import Image

def main():
    print("Grabbing simulator window...")
    # Get window id for Micromouse sim
    out = subprocess.check_output(['xwininfo', '-display', ':1', '-tree', '-root']).decode()
    sim_id = None
    for line in out.splitlines():
        if 'Micromouse sim' in line:
            sim_id = line.strip().split()[0]
            break
    print(f"Found sim window: {sim_id}")
    if not sim_id:
        sys.exit(1)

    # Grab window using xwd
    subprocess.run(['xwd', '-display', ':1', '-id', sim_id, '-silent', '-out', '/tmp/sim_win.xwd'], check=True)
    with open('/tmp/sim_win.xwd', 'rb') as f:
        hdr = struct.unpack('>25I', f.read(100))
        hdr_size, w, h, bpl, ncolors = hdr[0], hdr[4], hdr[5], hdr[12], hdr[19]
        f.seek(hdr_size + ncolors * 12)
        raw = f.read()
    img = Image.frombytes('RGB', (w, h), raw, 'raw', 'BGRX', bpl)
    out_path = '/home/ishant/.gemini/antigravity-ide/brain/67da92d7-f31e-41f1-a3fe-d96490312cc4/sim_win.png'
    img.save(out_path)
    print(f"Successfully saved {out_path} ({w}x{h})")

if __name__ == '__main__':
    main()
