#!/usr/bin/env python3
"""Nagrywanie czterech mikrofonów PCB przez USB (profil esp32-s3-recorder)."""
import argparse
from array import array
from contextlib import ExitStack
from datetime import datetime
import math
from pathlib import Path
import struct
import sys
import time
import wave

import serial

RATE = 32000
HEADER = struct.Struct('<IHHIIHBBIQ')
MAGIC = struct.pack('<I', 0x314D4350)
# Existing square layout: TL=MK1, TR=MK4, BL=MK2, BR=MK3.
SOURCE_TO_MIC = {1: 1, 2: 4, 3: 2, 4: 3}


def read_exact(port, size):
    result = bytearray()
    while len(result) < size:
        chunk = port.read(size - len(result))
        if not chunk:
            raise RuntimeError('Brak danych. Wgraj profil esp32-s3-recorder i zamknij Serial Monitor.')
        result.extend(chunk)
    return bytes(result)


def read_packet(port):
    prefix = bytearray(read_exact(port, 4))
    while prefix[-4:] != MAGIC:
        if prefix.endswith(b'\n'):
            line = bytes(prefix).splitlines()[-1]
            if line.startswith(b'ERR '):
                raise RuntimeError('ESP32: ' + line.decode(errors='replace'))
        if len(prefix) > 4096:
            raise RuntimeError('Brak pakietów audio. Sprawdź, czy wgrany jest profil recorder.')
        prefix.extend(read_exact(port, 1))
    fields = HEADER.unpack(MAGIC + read_exact(port, HEADER.size - 4))
    _, version, header_size, sequence, rate, count, source, _, frame, timestamp = fields
    if version != 3 or header_size != HEADER.size:
        raise RuntimeError('Niepasujący protokół. Wgraj profil esp32-s3-recorder z tego repo.')
    if rate != RATE or source not in SOURCE_TO_MIC or not 0 < count <= 512:
        raise RuntimeError('Nieprawidłowy format pakietu audio.')
    return sequence, frame, SOURCE_TO_MIC[source], timestamp, read_exact(port, count * 2)


def stop_device(port):
    port.write(b'STOP\n')
    port.flush()
    old_timeout = port.timeout
    port.timeout = 0.3
    response = bytearray()
    deadline = time.monotonic() + 5
    try:
        while time.monotonic() < deadline:
            response.extend(port.read(1024))
            if b'OK STOP ' in response:
                port.reset_input_buffer()
                return
            response = response[-8192:]
        raise RuntimeError('Brak odpowiedzi STOP. Wgraj profil esp32-s3-recorder z nowego repo.')
    finally:
        port.timeout = old_timeout


def open_wav(stack, path, channels):
    wav = stack.enter_context(wave.open(str(path), 'wb'))
    wav.setnchannels(channels)
    wav.setsampwidth(2)
    wav.setframerate(RATE)
    return wav


def capture(port, output, frames):
    output.mkdir(parents=True, exist_ok=False)
    received = 0
    sequence = None
    next_frame = None
    current_frame = None
    timestamp = None
    pending = {}
    sums, squares, peaks = [0] * 4, [0] * 4, [0] * 4
    with ExitStack() as stack:
        combined = open_wav(stack, output / 'mikrofony.wav', 4)
        mono = [open_wav(stack, output / f'MK{i+1}.wav', 1) for i in range(4)]
        print(f'Nagrywam {frames / RATE:g} s z MK1–MK4...', flush=True)
        while received < frames:
            seq, frame, mic, packet_time, payload = read_packet(port)
            if sequence is not None and seq != sequence:
                raise RuntimeError('Zgubiono pakiet USB; zapis zawiera niepełne nagranie.')
            sequence = (seq + 1) & 0xffffffff
            if not pending:
                if next_frame is not None and frame != next_frame:
                    raise RuntimeError('Brakuje bloku audio.')
                current_frame, timestamp = frame, packet_time
            if frame != current_frame or mic in pending or packet_time != timestamp:
                raise RuntimeError('Niekompletny lub powtórzony blok mikrofonów.')
            pending[mic] = payload
            if len(pending) != 4:
                continue
            sizes = {len(data) for data in pending.values()}
            if len(sizes) != 1:
                raise RuntimeError('Różne długości kanałów audio.')
            count = min(len(payload) // 2, frames - received)
            interleaved = array('h', [0]) * (count * 4)
            for channel in range(4):
                data = pending[channel + 1][:count * 2]
                mono[channel].writeframesraw(data)
                values = array('h')
                values.frombytes(data)
                if sys.byteorder != 'little':
                    values.byteswap()
                interleaved[channel::4] = values
                sums[channel] += sum(values)
                squares[channel] += sum(x * x for x in values)
                peaks[channel] = max(peaks[channel], max(abs(x) for x in values))
            if sys.byteorder != 'little':
                interleaved.byteswap()
            combined.writeframesraw(interleaved.tobytes())
            received += count
            pending.clear()
            next_frame = (frame + 1) & 0xffffffff
    print(f'Zapisano: {output.resolve()}')
    for channel in range(4):
        rms = math.sqrt(max(0, squares[channel] / received - (sums[channel] / received) ** 2))
        level = 20 * math.log10(rms / 32768) if rms else float('-inf')
        print(f'MK{channel+1}: RMS AC {level:.1f} dBFS, peak {peaks[channel]}'
              + (' — brak zmiennego sygnału' if rms == 0 else ''))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--output', type=Path,
                        default=Path('nagrania') / datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 295:
        parser.error('--seconds musi być w zakresie (0, 295]')
    frames = int(args.seconds * RATE)
    if frames < 1 or args.output.exists():
        parser.error('Za krótki czas lub folder docelowy już istnieje.')
    with serial.Serial(args.port, 115200, timeout=3, write_timeout=3, exclusive=True) as port:
        time.sleep(1)
        stop_device(port)
        port.write(f'START {math.ceil(args.seconds) + 3}\n'.encode())
        port.flush()
        try:
            capture(port, args.output, frames)
        finally:
            stop_device(port)


if __name__ == '__main__':
    try:
        main()
    except (serial.SerialException, RuntimeError, OSError, KeyboardInterrupt) as error:
        raise SystemExit(str(error) or 'Przerwano nagrywanie; zapisano pliki częściowe.')
