# AOGmatic

AOGmatic (DIY) pour le contrôle des sections avec des Servomoteurs via AgopenGps (5.xx). Les modèles 3D présenté ici sont pour une utilisation avec un boitier HardySprayII ou EvrardEC2500.
Mais le code pourrait très bien servir à bien d'autre boitier ou autre utilisation.

Vidéo Démo:
https://www.youtube.com/watch?v=ULhPUGdXarA

# Hardware

Nous utilisons ici: Un arduino Nano, Un régulateur 5V, Une plaquette contrôle de relais de servomoteurs PCA9685, des servomoteurs SG90.

Schéma de branchement:
![Schéma](Pics/Schema.jpg)


Liens des fichiers 3D:
https://cults3d.com/fr/mod%C3%A8le-3d/outil/aogmatic-pour-hardy-sprayii


Vidéo d'explication du montage des pièces 3D:
https://www.youtube.com/watch?v=GPidbl-RtbM


Utilisé le code Shield UDP si vous voulez utiliser un shield enc28j60.

# Fonctionnalités

Les servomoteurs doivent être branchés dans l'ordre des sections sur la plaquette. Si moins de 16 sections sont utilisées, l'emplacement 16 sur la plaquette se comporte comme un maitre. Explication: il sera tout simplement actif si au moins une des sections est active.

Mode Démo : connecté A0 à la masse et avancé dans un champ sur AOG, admiré le résultat :-)



