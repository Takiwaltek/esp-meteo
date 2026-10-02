# esp-meteo
Tableau de bord météo pour ESP32-S3 : heure, météo, alerte pluie et satellite Meteosat animé

## Utilisation
- ESP-IDF 6.1, cible `esp32s3` : `idf.py set-target esp32s3 && idf.py build flash monitor`
- Renseigner le Wi-Fi, les broches de l'écran ILI9341 et la LED dans `main/config.h`.
- BOOT (appui court) : images Meteosat IR de l'Europe (8 images sur 4 h, animées) ; appui long : retour à la météo.
- PNG décodés par `main/png.c` avec l'inflate de la ROM (`rom/miniz.h`), images stockées en PSRAM.
- LED d'état : jaune soleil, blanc couvert, bleu pluie, violet orage, cyan neige.
