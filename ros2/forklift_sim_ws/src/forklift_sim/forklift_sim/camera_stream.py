"""camera_stream: faz o papel da camera IP -- imagem de uma camera do Gazebo -> stream H.264 (MPEG-TS
em UDP) que o camera_drivers ORIGINAL do astro abre pelo <camera>_ip_adress do astro.ini.

    camera do Gazebo --gz.msgs.Image--> camera_stream --ffmpeg/libx264--> udp://127.0.0.1:<porta>
        --> ./camera_drivers forklift_camN N --> mensagem cameraN (viewer_3D, camera_viewer, logger)

UDP: tanto faz quem sobe primeiro, e o driver pode reiniciar sem derrubar o stream.
Le' o topico do Gazebo direto (gz.transport), sem passar pelo ROS.

Uso: camera_stream --topic forklift_cam1/image --url udp://127.0.0.1:5601 --rate 15
"""

import argparse
import shutil
import subprocess
import sys
import threading
import time

from forklift_sim.gz_compat import Node, msg

Image = msg('image', 'Image')

# gz.msgs.PixelFormatType -> pix_fmt do ffmpeg (o sensor camera do Gazebo manda R8G8B8 por padrao)
PIX_FMT = {3: 'rgb24', 8: 'bgr24', 1: 'gray', 4: 'rgba', 9: 'bgra'}


class Streamer:
    def __init__(self, url, rate, bitrate):
        self.url = url
        self.rate = rate
        self.bitrate = bitrate
        self.proc = None
        self.shape = None
        self.lock = threading.Lock()
        self.frames = 0
        self.dropped = 0

    def start(self, width, height, pix_fmt):
        # zerolatency + GOP de 1 s: o driver consegue abrir o stream (precisa de um quadro-chave)
        # em ate' 1 s, e o atraso fica em poucos quadros
        cmd = ['ffmpeg', '-hide_banner', '-loglevel', 'error',
               '-f', 'rawvideo', '-pix_fmt', pix_fmt, '-s', '%dx%d' % (width, height), '-r', str(self.rate),
               '-i', '-', '-an',
               '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency', '-pix_fmt', 'yuv420p',
               '-g', str(int(self.rate)), '-b:v', self.bitrate,
               '-f', 'mpegts', self.url + '?pkt_size=1316']
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.shape = (width, height, pix_fmt)
        print('[camera_stream] %dx%d %s a %g Hz -> %s' % (width, height, pix_fmt, self.rate, self.url), flush=True)

    def on_image(self, msg):
        pix_fmt = PIX_FMT.get(msg.pixel_format_type)
        if pix_fmt is None:
            self.dropped += 1
            return
        with self.lock:
            if self.proc is None or self.proc.poll() is not None or self.shape != (msg.width, msg.height, pix_fmt):
                self.stop()
                self.start(msg.width, msg.height, pix_fmt)
            try:
                self.proc.stdin.write(msg.data)
                self.frames += 1
            except (BrokenPipeError, OSError):
                self.stop()

    def stop(self):
        if self.proc is not None:
            try:
                self.proc.stdin.close()
            except OSError:
                pass
            self.proc.terminate()
            self.proc = None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', required=True, help='topico gz.msgs.Image da camera no Gazebo')
    ap.add_argument('--url', required=True, help='destino do stream, ex.: udp://127.0.0.1:5601')
    ap.add_argument('--rate', type=float, default=15.0, help='Hz da camera (so para o encoder)')
    ap.add_argument('--bitrate', default='3M')
    # o launch do ROS acrescenta --ros-args ...
    a, _ = ap.parse_known_args()

    if shutil.which('ffmpeg') is None:
        sys.exit('[camera_stream] ffmpeg nao encontrado')

    streamer = Streamer(a.url, a.rate, a.bitrate)
    node = Node()
    if not node.subscribe(Image, a.topic, streamer.on_image):
        sys.exit('[camera_stream] nao assinou ' + a.topic)
    print('[camera_stream] esperando imagens em %s' % a.topic, flush=True)

    last = 0
    try:
        while True:
            time.sleep(10.0)
            n = streamer.frames
            if n == last:
                print('[camera_stream] %s: SEM imagens do Gazebo' % a.topic, flush=True)
            last = n
    except KeyboardInterrupt:
        pass
    finally:
        streamer.stop()


if __name__ == '__main__':
    main()
