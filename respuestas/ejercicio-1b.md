# Ejercicio 1.b — Configuración del proyecto en STM32CubeMX

## 1. Proyecto y herramientas

Se ha creado un proyecto en STM32CubeMX para el **STM32F411RET6**, correspondiente al microcontrolador seleccionado en el apartado 1.a. Se configura un bus SPI, un bus I²C y la interfaz de depuración necesaria para utilizar ST-LINK.

El proyecto se llama **exercise_1_2** y se encuentra en la carpeta [firmware/exercise_1_2](../firmware/exercise_1_2). Se utilizará este mismo proyecto para integrar la inicialización y lectura de los sensores en el ejercicio 2.b. La configuración se guarda en el archivo [exercise_1_2.ioc](../firmware/exercise_1_2/exercise_1_2.ioc), que permite abrirla y modificarla desde CubeMX.

Se ha utilizado **STM32CubeMX 6.18.1**, con el paquete **STM32Cube FW_F4 V1.28.3**, seleccionando **CMake** como opción de generación del proyecto. El código generado se ha abierto en VS Code con la extensión STM32CubeIDE y se ha compilado mediante GCC para ARM.

## 2. Interfaces y pines configurados

| Elemento | Pines | Configuración y función |
| --- | --- | --- |
| SPI1 | PA5: SCK; PA6: MISO; PA7: MOSI | Modo maestro, con líneas separadas para transmitir y recibir. Se utilizan transferencias de 8 bits, enviando primero el bit más significativo, y modo 0: CPOL bajo y CPHA en el primer flanco. El divisor de reloj es 16, por lo que la velocidad es de **6,25 Mbit/s**. |
| I²C1 | PB8: SCL; PB9: SDA | Modo estándar a **100 kHz**, con direcciones de 7 bits. Se configura para comunicar el microcontrolador con sensores que utilicen este bus. |
| Selección de sensores SPI | PA4: IMU_CS; PB0: HIGHG_CS | Dos salidas digitales independientes para seleccionar los sensores que compartirán SPI1. Se inicializan en nivel alto para mantenerlos deseleccionados, suponiendo señales CS activas en nivel bajo. |
| Depuración con ST-LINK | PA13: SWDIO; PA14: SWCLK | Se selecciona **Serial Wire** en la configuración de SYS, reservando estos pines para programación y depuración mediante SWD. |

En SPI1 se ha deshabilitado la selección automática por hardware, ya que el programa controlará cada señal CS mediante su salida digital. Ambas salidas se configuran sin resistencias internas y con velocidad de salida baja.

Los pines de I²C se configuran como salidas de drenador abierto, por lo que en el montaje físico el bus necesitará resistencias de subida a la alimentación, que pueden estar incorporadas en los módulos de sensores. El modo y la velocidad de SPI se comprobarán con los modelos elegidos en el ejercicio 2, antes de utilizarlos para leer medidas.

## 3. Configuración de relojes

Se utiliza el oscilador interno **HSI de 16 MHz** como origen del reloj. Mediante el PLL, que permite obtener una frecuencia superior a partir de este oscilador, se configura la CPU a **100 MHz**. Los valores utilizados son M = 16, N = 200 y P = 2:

**16 MHz / 16 × 200 / 2 = 100 MHz.**

El divisor AHB se mantiene en 1, mientras que APB1 se divide entre 2 y APB2 entre 1. De esta forma, AHB y la CPU trabajan a **100 MHz**, APB1 a **50 MHz** y APB2 a **100 MHz**, que son las frecuencias máximas de estos dos buses en el STM32F411. SPI1 recibe su reloj de APB2, por lo que su velocidad resulta de dividir 100 MHz entre 16.

Esta configuración permite utilizar la frecuencia máxima del microcontrolador seleccionado sin necesitar un oscilador externo para el reloj principal.

## 4. Código generado y comprobación

En [main.c](../firmware/exercise_1_2/Core/Src/main.c), el programa inicializa la biblioteca HAL de ST, configura los relojes y después inicializa las salidas digitales, I²C1 y SPI1. Tras ello, entra en el bucle principal, que queda vacío en este apartado. La inicialización y lectura de los sensores se incorporarán en el ejercicio 2.b.

Se ha comprobado la correspondencia entre la configuración de CubeMX y el código generado. Además, el proyecto se ha **compilado correctamente en configuración Debug**, generando el archivo `exercise_1_2.elf`. Esta comprobación confirma que el proyecto puede construirse con las herramientas seleccionadas, pero no demuestra su funcionamiento en hardware: al no disponer de una placa física, no se han probado la conexión con ST-LINK ni las comunicaciones con sensores.
