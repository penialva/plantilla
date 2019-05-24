# Sistemas Operativos de Tiempo Real - Trabajo práctico final 

Curso de posgrado pensado para la Especialización en Sistemas Embebidos y la Maestría en Ingeniería Biomédica


## Drivers utilizados

- ILI9341
- f_stepper

## Descripción de funcionamiento

El slider permite ser configurado mediante el uso del teclado de la EDU-CIAA y el LCD. En primer lugar el menú permite elegir entre modo
Foto o Video.

A continuación se configura el tiempo de duración de la sesión y la distancia a recorrer del slider. En modo Foto se configura
además la cantidad de fotos a tomar.

Al finalizar la configuración se da inicio a la sesión, tras lo cual el slider realizará el "homming" o puesta a cero. En este
paso el carro se mueve en el sentifo de "home" hasta presionar un swith de fin de carrera, luego de lo cual inicia la 
sesión configurada.

## Diagrama de configuración en Modo Foto

![alt text](https://github.com/juanic/plantilla/blob/master/dia_1.png)

## Diagrama de configuración en Modo Video

![alt text](https://github.com/juanic/plantilla/blob/master/dia_2.png)


## Videos demostrativos
https://youtu.be/SDs7J-F429k
https://youtu.be/oxqPVLT6klE
https://drive.google.com/file/d/1--Ih3DCaDqLaV8Ka5u5lhqJuJlxRRIpR/view?usp=sharing

## Time-lapse
https://youtu.be/ESD5Wg6Tirk

## Backstage del time-lapse
https://drive.google.com/file/d/1-0l4XrU7XVb9XXkLCgqXGiK82HfRDBn_/view?usp=sharing
https://drive.google.com/file/d/1--wwBv05AdXQ_tr5hYrj2w9tjY-rjzaw/view?usp=sharing

