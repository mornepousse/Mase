# MaSe - Mouse Sensor Firmware

**MaSe** est un firmware de souris haute performance pour l'ESP32-S3, utilisant le capteur optique PMW3389 et l'interface TinyUSB pour une communication HID rapide.

## 🚀 Fonctionnalités

*   **Capteur PMW3389** : Support complet via SPI avec lecture en mode "Burst" pour une latence minimale.
*   **Polling Rate 1000Hz** : Boucle principale et tick FreeRTOS optimisés pour une réponse à 1ms.
*   **DPI Ajustable** : Cycle entre les niveaux de DPI (400, 800, 1600, 3200, 6400) via un bouton dédié.
*   **Molette de défilement** : Support d'encodeur rotatif.
*   **Boutons Programmables** : Clic gauche, droit, milieu, et touches de fonction (ex: Touche Windows).
*   **Indicateur d'État LED** : LED WS2812 (RGB) pour indiquer le mode de connexion (Vert = USB).
*   **Architecture Modulaire** : Code séparé pour le capteur, la gestion des LEDs et la logique principale.

## 🛠 Matériel Requis

*   **Microcontrôleur** : ESP32-S3 (Testé sur DevKitM-1 / DevKitC).
*   **Capteur** : PixArt PMW3389.
*   **Composants** : Switchs pour les boutons, Encodeur rotatif, LED WS2812.

### 🔌 Pinout (Configuration par défaut)

| Périphérique | Fonction | GPIO (ESP32-S3) |
| :--- | :--- | :--- |
| **PMW3389** | MISO | 7 |
| | MOSI | 6 |
| | SCLK | 5 |
| | CS | 15 |
| | MOTION | 4 |
| **Boutons** | Clic Gauche (LMB) | 21 |
| | Clic Droit (RMB) | 37 |
| | Clic Molette (Middle) | 38 |
| | DPI Switch | 44 |
| | Touche Windows | 35 |
| **Encodeur** | A | 2 |
| | B | 42 |
| | C (Commun/GND) | 41 |
| **LED** | Data (WS2812) | 48 |

## 📂 Structure du Projet

```
main/
├── main.c           # Point d'entrée, boucle principale (USB, lecture capteur)
├── pmw3389.c        # Driver du capteur (SPI, Burst read, SROM)
├── pmw3389.h        # Définitions et prototypes du capteur
├── led_status.c     # Gestion de la LED d'état (RMT)
├── led_status.h     # Prototypes LED
├── srom_pmw3389.h   # Firmware binaire du capteur
└── CMakeLists.txt   # Configuration de build
```

## 🔨 Compilation et Flash

Ce projet utilise le framework **ESP-IDF**.

1.  **Configurer la cible :**
    ```bash
    idf.py set-target esp32s3
    ```

2.  **Compiler :**
    ```bash
    idf.py build
    ```

3.  **Flasher et Monitorer :**
    ```bash
    idf.py -p /dev/ttyACM0 flash monitor
    ```

## 📝 Notes

*   Le firmware du capteur (SROM) est chargé au démarrage. Assurez-vous que le fichier `srom_pmw3389.h` est présent.
*   La fréquence FreeRTOS est configurée à 1000Hz pour assurer la fluidité du curseur.
