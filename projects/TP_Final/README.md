# Trabajo práctico final 

Este repositorio contiene los archivos necesarios para compilar proyectos en lenguaje C que se ejecuten en una placa CIAA. Es una versión simplifciada del firmware original, pensada para se utilizada en ambientes educativos.

## IMPORTANTE

**Este entorno esta en construccion!!!**

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

