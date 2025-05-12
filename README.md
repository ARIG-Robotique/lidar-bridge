# Bridge C++ avec les drivers Lidar
[![Build Status](https://travis-ci.org/ARIG-Robotique/rplidar-bridge.svg?branch=master)](https://travis-ci.org/ARIG-Robotique/rplidar-bridge)

## Description

Ce projet ouvre une socket TCP (inet ou unix) afin de recevoir des ordres
JSON pour piloter les lidars :
 - [RPLidar](http://www.slamtec.com/en/Lidar)
 - [LD19Lidar](https://www.gotronic.fr/art-capteur-de-distance-lidar-ld19-35884.htm)

## Attention le SDK 1.12 de RPLidar ne se compile pas avec >= C++11

Pour cela il y a deux switch a changer de la sorte :

Avant :
```c++
switch (_dataEvt.wait(timeout))
```

Apres :
```c++
switch (static_cast<int>(_dataEvt.wait(timeout)))
```

> Lors du download des librairies un patch est automatiquement appliqué pour résoudre cela.

## Messages JSON

### Récupérer les infos du device

* Query
```json
{
  "action": "DEVICE_INFO"
}
```

* Réponse
```json
{
  "status": "OK",
  "action": "DEVICE_INFO",
  "datas": {
    "driver": "rplidar",
    "firmwareVersion": "1.20",
    "hardwareVersion": 2,
    "serialNumber": "123456ABCDEF"
  },
  "errorMessage": null
}
```

### Récupérer l'état de santé du device

* Query
```json
{
  "action": "HEALTH_INFO"
}
```

* Réponse
```json
{
  "status": "OK",
  "action": "HEALTH_INFO",
  "datas": {
    "libelle": "OK",
    "value": 0,
    "errorCode": 0
  },
  "errorMessage": null
}
```

La données libélé peut prendre les valeurs suivante :

* OK
* WARNING
* ERROR

Les autres valeurs sont récupéré directement depuis le driver RPLidar

### Lancer le scan

* Query sans vitesse de rotation moteur
```json
{
  "action": "START_SCAN"
}
```
* Query avec vitesse de rotation moteur
```json
{
  "action": "START_SCAN",
  "datas": {
    "speed": 560
  }
}
```

La vitesse est comprise entre 0 et 1023

* Réponse
```json
{
  "status": "OK",
  "action": "START_SCAN",
  "errorMessage": null
}
```

### Arreter le scan

* Query
```json
{
  "action": "STOP_SCAN"
}
```

* Réponse
```json
{
  "status": "OK",
  "action": "STOP_SCAN",
  "errorMessage": null
}
```

### Changer la vitesse de rotation du moteur

* Query
```json
{
  "action": "SET_SPEED",
  "datas": {
    "speed": 560
  }
}
```

La vitesse est comprise entre 0 et 1023

* Réponse
```json
{
  "status": "OK",
  "action": "SET_SPEED",
  "errorMessage": null
}
```

### Récupérer les données du dernier scan

* Query

```json
{
  "action": "GRAB_DATA"
}
```

* Réponse
```json
{
  "status": "OK",
  "action": "GRAB_DATA",
  "errorMessage": null,
  "datas": {
    "ignored": 358,
    "scan": [
      {
        "angleDeg": 122.3,
        "distanceMm": 1243.45,
        "syncBit": true,
        "quality": 47
      },
      {
        "angleDeg": 245.6,
        "distanceMm": 234.12,
        "syncBit": true,
        "quality": 44
      }
    ]
  }
}
```
### Quitter le programme et arreter le lidar

* Query
```json
{
  "action": "EXIT"
}
```

* Réponse
```json
{
  "status": "OK",
  "action": "EXIT",
  "errorMessage": null
}
```
