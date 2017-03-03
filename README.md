# Bridge C++ avec le driver RPLidar
[![Build Status](https://travis-ci.org/ARIG-Robotique/rplidar-bridge.svg?branch=master)](https://travis-ci.org/ARIG-Robotique/rplidar-bridge)

## Description

Ce projet ouvre un socket TCP sur le port 50000 afin de recevoir des ordres
JSON pour piloter le [RPLidar](http://www.slamtec.com/en/Lidar)

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

### Lancer un scan

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
