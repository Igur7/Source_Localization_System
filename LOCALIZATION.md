# Lokalizacja źródła dźwięku na ESP32-S3

Firmware oblicza kierunek nadejścia dźwięku lokalnie. Nie wysyła próbek PCM.
Wynik na porcie szeregowym ma postać:

```text
ANGLE 123.45
```

Kąt jest podawany w zakresie `0..360` stopni. `0` oznacza prawą stronę
macierzy, a `90` stronę górnej pary mikrofonów.

## Dwa niezależne kontrolery I2S

I2S1 i I2S0 pracują jako niezależne mastery. Nie należy łączyć ich linii BCLK
ani WS/LRCLK. Każdy kontroler obsługuje jedną przekątną prostokąta, dlatego
algorytm nie musi korelować kanałów należących do różnych zegarów.

| Para | Kontroler | BCLK | WS/LRCLK | DOUT |
|---|---:|---:|---:|---:|
| top-left + bottom-right | I2S1 | GPIO46 | GPIO10 | GPIO12 |
| top-right + bottom-left | I2S0 | GPIO1 | GPIO42 | GPIO40 |

Mapowanie slotów I2S:

| Mikrofon | Kontroler | Slot | L/R / SEL |
|---|---:|---:|---:|
| top-left | I2S1 | LEFT | GND |
| bottom-right | I2S1 | RIGHT | 3V3 |
| top-right | I2S0 | LEFT | GND |
| bottom-left | I2S0 | RIGHT | 3V3 |

## Uruchomienie

```bash
pio run -t upload --upload-port /dev/ttyACM0
pio run -e esp32-s3-devkitc-1 -t upload --upload-port /dev/ttyACM0
pio device monitor -b 115200 --port /dev/ttyACM0
```

W monitorze należy wysłać na przykład:

```text
START 300
```

Komendy `STATUS` i `STOP` służą odpowiednio do sprawdzenia i zatrzymania
serwisu.

## Diagnostyczne nagrywanie czterech mikrofonów

Osobny profil `esp32-s3-recorder` wgrywa firmware przesyłający cztery kanały
PCM16 przy 16 kHz i 2 Mb/s. Służy do sprawdzania mikrofonów oraz kalibracji;
zastępuje na czas testu firmware lokalizacyjny.

```bash
pio run -e esp32-s3-recorder -t upload --upload-port /dev/ttyACM0
```

Odbiór i zapis czterech kanałów uruchamia się z katalogu głównego repozytorium:

```bash
python3 pc_tools/serial_triangle_audio_capture.py \
  --port /dev/ttyACM0 --baud 2000000 --sample-rate 16000 \
  --layout square --request-seconds 30 --output-dir square_capture_test
```

Powstają katalogi `top_left`, `top_right`, `bottom_left` i `bottom_right`.
Powrót do lokalizacji wymaga ponownego wgrania domyślnego środowiska:

```bash
pio run -e esp32-s3-devkitc-1 -t upload --upload-port /dev/ttyACM0
```

## Algorytm

Serwis analizuje bloki 512 próbek przy 48 kHz. GCC-PHAT jest wykonywany tylko
dla dwóch zsynchronizowanych przekątnych: top-left--bottom-right oraz
top-right--bottom-left. Zmierzony lag jest interpolowany podpróbkowo i
ograniczony fizyczną długością przekątnej.

Kąt jest wybierany przez ważoną minimalizację błędu pomiędzy dwoma zmierzonymi
TDOA a opóźnieniami przewidywanymi przez model fali płaskiej dla kierunków
0--360 stopni. Pomiary z dużym residuum są odrzucane, a pięć ostatnich
zaakceptowanych kierunków jest uśrednianych kołowo. Wymiary mierzone między
portami akustycznymi wynoszą 0,134 m lewo-prawo oraz 0,140 m góra-dół i są
ustawione w `src/main.cpp`.
