# esp-meteo
Tableau de bord météo pour ESP32-S3 avec heure, météo actuelle et bandeau « PLUIE 24H »
(risque de pluie ou non sur les 24 prochaines heures). Le bouton BOOT fait défiler les pages :
météo → prévisions 7 jours (une carte par jour : température mini/maxi, condition, probabilité de
pluie) → satellite Meteosat animé en plein écran (RGB le jour, infrarouge IR108 la nuit) → météo.
Un appui long revient à l'accueil. Configurez le SSID et le mot de passe dans le menu Kconfig
« Horloge Wi-Fi ».
