# Source Localization System — lokalizacja kierunku źródła dźwięku

System lokalizacji kierunku nadejścia dźwięku (**DOA, Direction of Arrival**) oparty na **ESP32-S3** i czterech cyfrowych mikrofonach MEMS **SPH0645LM4H**. Firmware napisany w **C/C++** przetwarza sygnały bezpośrednio na mikrokontrolerze i wyznacza kąt źródła dźwięku w zakresie **0–360°**.

Projekt został opracowany w ramach działalności **AGH Drone Engineering**. Działająca wersja prototypowa wykorzystuje cztery mikrofony rozmieszczone w narożnikach kwadratu. Osobno rozwijana jest [czterowarstwowa płytka PCB](https://github.com/Igur7/AngleCircle) do integracji systemu.

## Najważniejsze funkcje

- Równoczesny odczyt dwóch par mikrofonów za pomocą **dwóch niezależnych kontrolerów I²S** w ESP32-S3.
- Wyznaczanie opóźnienia dotarcia dźwięku (**TDOA**) z użyciem **GCC-PHAT** dla każdej zsynchronizowanej pary.
- Oszacowanie kierunku na podstawie geometrii układu i modelu propagacji fali dźwiękowej.
- Filtrowanie niepewnych pomiarów i kołowe uśrednianie kolejnych kątów.
- Sterowanie przez port szeregowy oraz przesyłanie wyników w formacie `ANGLE <kąt>`.
- Oddzielny tryb diagnostyczny do odczytu surowych próbek z czterech mikrofonów.

## Sprzęt i architektura

- **Mikrokontroler:** ESP32-S3
- **Mikrofony:** 4 × SPH0645LM4H, cyfrowy interfejs I²S
- **Układ:** dwa mikrofony na każdej przekątnej kwadratu
- **Środowisko:** PlatformIO + Arduino framework
- **Oprogramowanie:** C/C++
