# Ejercicio 2.b — Inicialización y lectura de sensores

## 1. Integración en el proyecto

Se ha continuado el proyecto [exercise_1_2](../firmware/exercise_1_2) del ejercicio 1.b, incorporando el código necesario para inicializar y leer los sensores seleccionados en el [apartado 2.a](ejercicio-2a.md). Se mantienen el **STM32F411RET6**, el reloj de CPU a **100 MHz** y la interfaz de depuración SWD seleccionados en el primer ejercicio.

El código está escrito en **C**, utilizando la biblioteca HAL de ST. Las funciones propias se encuentran en los bloques `USER CODE` de [main.c](../firmware/exercise_1_2/Core/Src/main.c), y la configuración de los periféricos se guarda en [exercise_1_2.ioc](../firmware/exercise_1_2/exercise_1_2.ioc). Se utilizan los controladores oficiales de ST para [ISM330DHCX](../firmware/exercise_1_2/Drivers/ISM330DHCX) y [H3LIS331DL](../firmware/exercise_1_2/Drivers/H3LIS331DL), adaptando sus accesos a SPI y añadiéndolos a la compilación mediante CMake. Estas carpetas incluyen sus licencias y versiones.

## 2. Interfaces y configuración utilizada

| Elemento | Interfaz y pines | Configuración |
| --- | --- | --- |
| IMU ISM330DHCX | SPI1: PA5 (SCK), PA6 (MISO), PA7 (MOSI); PA4 (IMU_CS) | Aceleración a **±16 g** y velocidad angular a **±4000 °/s**, ambas a **208 Hz**. |
| Acelerómetro H3LIS331DL | Comparte SPI1; PB0 (HIGHG_CS) | Tres ejes a **±200 g** y **400 Hz**. |
| Barómetro AMS 5935-1500-A | I²C1: PB8 (SCL), PB9 (SDA) | **100 kHz**, dirección de 7 bits `0x28` y medición simple solicitada cada **20 ms**. |
| Receptor MAX-M10S | USART1: PA9 (TX), PA10 (RX) | **115200 bit/s**, 8 bits, sin paridad y un bit de parada; GPS + Galileo a **10 Hz**, con modelo Airborne <4g. |
| Alimentación y temperatura | ADC1: PC0 (canal 10), PC1 (canal 11), PC2 (canal 12), VREFINT y temperatura internos | Resolución de **12 bits**; alimentación cada **100 ms** y temperatura cada **1000 ms**. |

SPI1 se ajusta al **modo 3**, con CPOL alto y CPHA en el segundo flanco, porque ambos sensores inerciales describen una comunicación con el reloj en nivel alto durante el reposo y la captura de datos en el flanco ascendente, que en este modo es el segundo flanco. Se mantienen los **6,25 MHz**, por debajo del máximo de 10 MHz indicado en las hojas de datos del [ISM330DHCX](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf) y del [H3LIS331DL](https://www.st.com/resource/en/datasheet/h3lis331dl.pdf). Cada sensor se selecciona mediante su salida CS, que vuelve a nivel alto al terminar la transferencia, también si se produce un error.

Se añaden USART1 y ADC1 al proyecto inicial del ejercicio 1. USART1 utiliza recepción por **DMA circular**, con interrupciones de DMA y UART habilitadas. CubeMX la inicializa a 9600 bit/s y el programa cambia después a 115200 bit/s. TX del STM32 se conecta a RX del receptor y viceversa.

## 3. Inicialización y adquisición de las medidas

**Sensores inerciales.** `IMU_Init()` y `HIGHG_Init()` esperan el arranque y comprueban el registro `WHO_AM_I` antes de configurar cada dispositivo. La IMU se reinicia además con una espera limitada. Solo se habilitan las lecturas cuando la inicialización termina correctamente.

Se activa **Block Data Update (BDU)** porque cada medida de un eje ocupa dos bytes que se leen sucesivamente. Esta opción impide que sus registros se actualicen entre ambas lecturas, evitando combinar un byte de una medida con otro de la siguiente. Las funciones comprueban si hay datos nuevos y leen los tres ejes, convirtiéndolos a **g** y **grados por segundo**. Devuelven `1` con una muestra nueva, `0` si no hay otra disponible y `-1` si falla la comunicación.

**Barómetro.** `BARO_Update()` envía la orden de medición y deja que el bucle atienda los otros sensores durante la conversión. Pasados al menos 4 ms, consulta el estado del AMS 5935. Si sigue ocupado, vuelve a comprobarlo en llamadas posteriores, con un plazo de 10 ms para dejar de esperar.

Se reciben siete bytes: estado, presión y temperatura. Tras comprobar los indicadores de error, se convierten las medidas a **hPa** y **°C** mediante las ecuaciones del fabricante referenciadas en el apartado 2.a. Este sensor no requiere configurar rangos: la orden enviada inicia directamente la medición simple sin necesidad de una configuración anterior. Su temperatura corresponde a la del propio sensor.

**Receptor GNSS.** `GNSS_Configure()` prueba 9600 y 115200 bit/s, porque el receptor puede conservar la velocidad anterior si solo se reinicia el STM32. Los ajustes se aplican mediante órdenes UBX confirmadas y se guardan en la RAM del receptor. Tras cambiar la velocidad, se adapta la UART local y se verifica la comunicación con otra orden confirmada.

Se habilita el mensaje **UBX-NAV-PVT** y se desactiva la salida NMEA. El DMA recibe los bytes en un búfer circular de **512 bytes**; el bucle reconstruye cada mensaje completo y comprueba su longitud y checksum antes de interpretarlo. Se procesan como máximo 128 bytes por llamada para compartir el tiempo con los otros sensores.

Se extraen posición, altura respecto al nivel medio del mar, velocidades y estimaciones de error. La velocidad vertical se expresa positiva hacia arriba, haciendo un cambio de signo respecto a la que envía el receptor GNSS, que la marca positiva hacia abajo. Se comprueban los indicadores de validez, la solución 3D y los rangos de coordenadas; se retira la validez tras 500 ms sin procesar otro NAV-PVT. Las estimaciones de error se conservan para su posterior evaluación. Los campos y órdenes proceden del [protocolo u-blox M10](https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf).

**Alimentación y temperatura.** `ADC_Update()` lee sucesivamente VREFINT, la batería y las dos líneas reguladas. El ADC realiza una conversión por canal, iniciada por software, con reloj a 25 MHz y 480 ciclos de muestreo. La lectura de VREFINT y su calibración de fábrica permiten estimar **VDDA**, la alimentación analógica del microcontrolador, tal que para convertir las lecturas a voltios se emplea la siguiente expresión:

**Tensión de la línea = lectura ADC / 4095 × VDDA × factor del divisor resistivo.**

Los factores de ejemplo son **4 para la batería** y **2 para cada línea regulada**, y deben corresponder a los divisores resistivos del montaje. Se rechazan las lecturas en el máximo digital, donde no se puede distinguir el límite de medida de una saturación.

La temperatura interna se obtiene con las dos calibraciones de fábrica, realizadas con una alimentación analógica de **3,3 V**. Como el valor digital del ADC depende también de su alimentación, primero se utiliza VDDA para convertir la lectura al valor equivalente que se obtendría a 3,3 V. Así se puede comparar con las calibraciones sin confundir una variación de alimentación con un cambio de temperatura. Se habilitan previamente las entradas internas y se espera su estabilización antes de empezar a leer sus medidas. Las tensiones se actualizan juntas si todas se leen correctamente; la temperatura se evalúa por separado cuando VDDA está disponible.

## 4. Bucle principal y gestión de errores

Se configura primero el GNSS para completar sus esperas antes de iniciar las medidas inerciales. Después se inicializan estos sensores y las marcas de tiempo de las tareas periódicas.

En cada vuelta del bucle se atienden la IMU, el acelerómetro de alto rango, el barómetro, el GNSS y el ADC. No se añade una pausa general al final, ya que los sensores inerciales indican si tienen datos nuevos y las demás lecturas utilizan sus propios periodos. Las operaciones SPI, I²C y ADC emplean esperas limitadas, y la recepción GNSS se realiza mediante DMA.

Las últimas medidas quedan en variables de RAM, junto con sus estados y marcas de tiempo de lectura o procesamiento. Si una adquisición falla, se conserva la medida anterior con su marca de tiempo y se indica el error.

En el GNSS se comparan los totales de bytes recibidos y retirados para detectar que el búfer se ha llenado o ya se han sobrescrito bytes aún sin procesar. Con 512 bytes pendientes o más se detiene el procesamiento y se invalida la solución, incluyendo preventivamente el búfer justo lleno. Ante este problema o un error UART, se reinicia la recepción con intentos espaciados de un segundo. Se conservan contadores de mensajes rechazados e incidencias del búfer para poder analizarlos.

## 5. Comprobaciones realizadas

Se ha revisado la correspondencia entre la configuración de CubeMX, los pines y el código generado, así como la inicialización, las conversiones y la gestión de errores. El proyecto se ha **compilado correctamente en configuración Debug**, generando `exercise_1_2.elf`. Los pasos de compilación se encuentran en el [README](../README.md#proyecto-stm32).

Al no disponer de una placa y sensores físicos, no se ha podido comprobar la adquisición en hardware. Las frecuencias indicadas corresponden a los ajustes de los sensores o a los periodos solicitados por el programa; quedaría por medir el tiempo real del bucle y verificar que se recogen todas las muestras, especialmente las del acelerómetro a 400 Hz. Esta implementación conserva las últimas lecturas; la lógica posterior, como el registro completo del vuelo y la estimación del estado, queda fuera de este apartado, que solo se encarga de la inicialización y la lectura.
