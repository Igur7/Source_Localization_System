# Lokalizacja źródła dźwięku na ESP32-S3

Firmware oblicza kierunek nadejścia dźwięku lokalnie. Nie wysyła próbek PCM.
Wynik na porcie szeregowym ma postać:

```text
ANGLE 123.45
```

Kąt jest podawany w zakresie `0..360` stopni. `0` oznacza prawą stronę
macierzy, a `90` stronę górnej pary mikrofonów.

## Dwa niezależne kontrolery I2S

I2S0 i I2S1 pracują jako niezależne mastery. Nie należy łączyć ich linii BCLK
ani WS/LRCLK. Każdy kontroler obsługuje jedną przekątną prostokąta, dlatego
algorytm nie musi korelować kanałów należących do różnych zegarów.

| Para | Kontroler | BCLK | WS/LRCLK | DOUT |
|---|---:|---:|---:|---:|
| top-left + bottom-right (MK1 + MK3) | I2S0 | GPIO11 | GPIO10 | GPIO12 |
| top-right + bottom-left (MK4 + MK2) | I2S1 | GPIO1 | GPIO42 | GPIO40 |

Mapowanie slotów I2S:

| Mikrofon | Kontroler | Slot | L/R / SEL |
|---|---:|---:|---:|
| top-left / MK1 | I2S0 | LEFT | GND |
| bottom-right / MK3 | I2S0 | RIGHT | 3V3 |
| top-right / MK4 | I2S1 | LEFT | GND |
| bottom-left / MK2 | I2S1 | RIGHT | 3V3 |

Nazwy narożników są logicznymi nazwami istniejącego algorytmu. Przed pomiarem
kąta trzeba dopasować je oraz wymiary macierzy do faktycznego rozmieszczenia
portów akustycznych PCB.

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
PCM16 przy 32 kHz przez natywny USB Serial/JTAG. Służy do sprawdzania mikrofonów oraz kalibracji;
zastępuje na czas testu firmware lokalizacyjny.

```bash
~/.platformio/penv/bin/pio run -e esp32-s3-recorder -t upload
```

Odbiór i zapis czterech kanałów uruchamia się z katalogu głównego repozytorium:

```bash
~/.platformio/penv/bin/python pc_tools/record_mics.py --seconds 10
```

Skrypt sam wysyła `STOP`, następnie `START`, odbiera cztery kanały i zatrzymuje
nagrywanie. Zamknij Serial Monitor i pozostałe skrypty korzystające z portu.
W nowym katalogu `nagrania/<data_czas>/` powstają `MK1.wav`–`MK4.wav` oraz
`mikrofony.wav` z kanałami w kolejności MK1, MK2, MK3, MK4.
Skrypt sprawdza kolejność pakietów i bloków; błędy transmisji przerywają zapis.
Wspólny numer bloku nie oznacza synchronizacji próbek pomiędzy dwoma
niezależnymi zegarami I2S.

Opcje: `--port /dev/ttyACM1`, `--seconds 30`, `--output nagrania/test_1`.
Folder docelowy musi być nowy. Zależności PC: `pc_tools/requirements.txt`
(`pyserial`; dostępny także w środowisku PlatformIO).

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
