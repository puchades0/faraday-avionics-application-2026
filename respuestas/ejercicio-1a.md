# Ejercicio 1.a — Selección del microcontrolador STM32

## 1. Misión de referencia y funciones del ordenador de vuelo

Para seleccionar justificadamente un microcontrolador STM32, primero se define la misión y el alcance del ordenador de vuelo. A partir de sus funciones y del hardware necesario se establecen los requisitos de memoria, capacidad de cálculo, consumo e interfaces, antes de comparar modelos concretos.

Se toma como referencia Origin, el cohete de dos etapas de Faraday que alcanzó un apogeo de **10 843 metros sobre el lanzamiento**. Para este ejercicio se adopta un límite de diseño de **15 km sobre el lanzamiento** y un número de Mach máximo de **2,5**, tomando este último del diseño publicado antes del vuelo. Son condiciones asumidas para dimensionar la aviónica, no los resultados del vuelo real, para el que el equipo publicó un valor de Mach 1,7. Fuentes: [Faraday, Origin](https://faraday.webs.upv.es/) y [descripción del diseño previo al vuelo en Dewesoft](https://dewesoft.com/blog/rocket-engine-static-fire-test).

Se considera que el ordenador está en la etapa superior y trabaja desde las comprobaciones previas al lanzamiento hasta la localización posterior al aterrizaje. Tras la separación, continúa gestionando esta etapa; la recuperación y el seguimiento de la primera quedan fuera del alcance. No se contempla regulación del apogeo mediante aerofrenos ni control activo de orientación durante el ascenso. Las funciones esenciales deben mantenerse de forma autónoma ante una pérdida de telemetría.

El ordenador deberá adquirir y validar medidas con sus marcas de tiempo; estimar posición, altitud, velocidad vertical y orientación; identificar las fases de vuelo y emitir las órdenes asignadas de separación y recuperación cuando se cumplan las condiciones previstas; registrar datos; transmitir telemetría y facilitar la localización; y supervisar fallos de sensores, alimentación y comunicaciones.

## 2. Requisitos iniciales de recursos

En este apartado se establecen las frecuencias de trabajo, el formato de los registros y las reservas de memoria que se utilizarán para comparar los microcontroladores. Son supuestos y presupuestos iniciales de diseño: los cálculos permiten obtener los recursos que requieren estas decisiones, mientras que las reservas para el software representan el espacio que se prevé dedicar a funciones todavía no implementadas. Su adecuación se comprobará según se especifiquen los componentes y se desarrolle el programa.

Se consideran los siguientes bloques, sin seleccionar todavía los modelos de sensores, que corresponden al ejercicio 2:

| Bloque o función | Necesidad prevista |
| --- | --- |
| IMU de seis ejes y acelerómetro de alto rango | Medir aceleraciones y velocidades angulares, con un canal adicional para aceleraciones que puedan superar el rango de la IMU. Se propone compartir un SPI. |
| Barómetro | Aportar una referencia para estimar altitud y velocidad vertical. Se propone I²C. |
| Receptor GNSS | Aportar posición y velocidad como información complementaria y facilitar la localización. Se reserva una UART, sin depender de su disponibilidad continua para las funciones esenciales. |
| Supervisión de alimentación y temperatura | Tres canales ADC para batería y dos líneas reguladas, y una medida de temperatura de la electrónica. |
| Almacenamiento externo y radio | Registrar el vuelo mediante un segundo SPI y transmitir telemetría mediante otra UART, suponiendo módulos compatibles. |
| Temporización, mecanismos y depuración | Temporizadores, señales digitales de selección, aviso, armado, orden y confirmación, y conexión SWD para ST-LINK. Los mecanismos requieren sus circuitos de actuación. |

Se presupuestan las siguientes frecuencias para los distintos bloques: datos inerciales a **200 Hz**, barómetro a **50 Hz**, GNSS y tensiones a **10 Hz**, temperatura a **1 Hz** y actualización del estado a **100 Hz**. De esta forma, se obtendrían medidas inerciales cada 5 ms y se actualizaría el estado cada 10 ms, mientras que las variables de supervisión tendrían un seguimiento más lento. Estas frecuencias se utilizarán para estimar los recursos necesarios.

Como referencia de lo que utilizan otras aviónicas, [Altus Metrum documenta 100 muestras/s durante el ascenso](https://altusmetrum.org/AltOS/doc/altusmetrum.html) y [Blue Raven registra datos inerciales a 500 Hz y barométricos a 50 Hz](https://www.featherweightaltimeters.com/blue-raven-altimeter.html). Estas fuentes permiten situar las frecuencias inerciales y barométricas propuestas en el contexto de sistemas existentes, pero no demuestran su suficiencia para nuestra misión. Además, se refieren a frecuencias de registro, que no tienen por qué coincidir con los ritmos internos de medida de los sensores o de cálculo de los algoritmos.

**Los datos que se van a registrar.** Para calcular cuánta memoria hace falta, se supone que cada lectura de un bloque se guarda como un registro con todos sus valores. Por ejemplo, las tres aceleraciones y las tres velocidades angulares de la IMU se agrupan en un único registro en el formato propuesto. Se reservan 4 bytes por valor y 8 bytes de cabecera: 4 para la marca de tiempo, 2 para identificar el tipo de registro y 2 para indicar su validez.

Por tanto, el número de bytes por registro queda como: **Bytes por registro = 4 × número de valores + 8.**

| Registro | Valores por registro | Bytes por registro | Registros/s | Bytes/s |
| --- | ---: | ---: | ---: | ---: |
| IMU | 6 | 32 | 200 | 6400 |
| Acelerómetro de alto rango | 3 | 20 | 200 | 4000 |
| Presión y temperatura barométricas | 2 | 16 | 50 | 800 |
| Posición y velocidad GNSS | 6 | 32 | 10 | 320 |
| Tres tensiones de alimentación | 3 | 20 | 10 | 200 |
| Temperatura de la electrónica | 1 | 12 | 1 | 12 |
| Estado estimado | Hasta 10 | 48 | 100 | 4800 |
| **Total** | | | **571** | **16 532** |

Estos tamaños corresponden al formato preparado para almacenar los datos, no necesariamente a los mensajes originales de los sensores. Adicionalmente, se añade un 25 % de margen de diseño para eventos y campos auxiliares que no se han considerado en los bloques establecidos:

**16 532 × 1,25 = 20 665 bytes/s.**

**RAM.** Se establece como objetivo poder acumular un segundo de datos mientras el almacenamiento externo no escribe. Es una tolerancia inicial elegida para el diseño, no una pausa medida de un dispositivo concreto ni un tiempo durante el que la CPU pueda quedarse bloqueada. El espacio necesario para los datos generados durante esa pausa es:

**20 665 bytes/s × 1 s = 20 665 bytes ≈ 20,2 KiB**, siendo 1 KiB = 1024 bytes.

Así que se reserva un **búfer de 24 KiB**, es decir, una zona de RAM donde guardar temporalmente los registros pendientes de escritura. Para soportar esa pausa debe quedar suficiente espacio libre al comenzar, y el almacenamiento debe poder vaciar lo acumulado al reanudar las escrituras. Las tareas esenciales deben continuar mientras tanto para no bloquear el sistema.

Además de este búfer se añaden dos bloques de memoria:

- **4 KiB para comunicaciones y últimas medidas:** 2 KiB para recibir e interpretar mensajes GNSS, 1 KiB para la radio y 0,5 KiB para conservar las últimas medidas, que se redondean a 4 KiB. Se prevé un total de hasta 512 bytes de mensajes GNSS por actualización, atendiendo su procesamiento al menos cada 100 ms, y paquetes de radio de hasta 128 bytes.
- **32 KiB para el resto del software:** una reserva conjunta para las variables y resultados temporales de todos los algoritmos, los datos de las bibliotecas, la gestión del sistema y la pila de ejecución, que guarda datos temporales de las funciones e interrupciones.

Por tanto, la RAM presupuestada es **24 + 4 + 32 = 60 KiB**.

**Flash.** Se presupuestan 32 KiB para arranque y periféricos, 32 KiB para sensores, comunicaciones y gestión de vuelo, 64 KiB para algoritmos y funciones matemáticas, 16 KiB para gestión de archivos y 8 KiB para constantes y valores iniciales. Se reserva más espacio para los algoritmos por ser la parte con mayor incertidumbre sobre su implementación y el espacio que ocupará. Los 152 KiB resultantes, con un 25 % de margen, dan 190 KiB, que se redondean a **192 KiB**. Los registros del vuelo se almacenarán externamente, por lo que no se suman al presupuesto de Flash interna; su almacenamiento temporal ya se ha incluido en el búfer de RAM.

**Capacidad de cálculo.** Se busca un microcontrolador con FPU de precisión simple, una unidad que realiza por hardware operaciones con números en coma flotante de 32 bits. Con la información disponible no se fija una frecuencia mínima de CPU justificada. Sí se establece que el programa debe atender la adquisición inercial cada 5 ms, actualizar el estado cada 10 ms y realizar las demás tareas dentro de sus plazos, evitando que las comunicaciones o el almacenamiento retrasen las funciones esenciales.

**Consumo.** Se utilizará el consumo activo publicado como criterio de comparación entre los modelos que cubran las necesidades anteriores, teniendo en cuenta la frecuencia, la tensión y las demás condiciones en las que se haya obtenido cada valor.

## 3. Comparación de candidatos

Se comparan tres modelos en sus versiones de 64 pines:

| Característica | STM32F411RE | STM32F446RE | STM32L476RG |
| --- | ---: | ---: | ---: |
| Frecuencia máxima de CPU | 100 MHz | 180 MHz | 80 MHz |
| SRAM principal | 128 KiB | 128 KiB | 128 KiB |
| Flash | 512 KiB | 512 KiB | 1024 KiB |
| FPU de precisión simple | Sí | Sí | Sí |
| Interfaces disponibles según la asignación de pines propuesta | Sí | Sí | Sí |
| Consumo típico publicado, en las condiciones indicadas | 11,6 mA a 100 MHz y 3,6 V | 32 mA a 180 MHz y 3,3 V | 10,2 mA a 80 MHz y 3,0 V |

Fuentes: hojas de datos de ST del [F411RE](https://www.st.com/resource/en/datasheet/stm32f411re.pdf), [F446RE](https://www.st.com/resource/en/datasheet/stm32f446re.pdf) y [L476RG](https://www.st.com/resource/en/datasheet/stm32l476rg.pdf); consumo en sus tablas 23, 23 y 27, respectivamente.

Los valores de consumo corresponden a ejecución desde Flash, con el acelerador de acceso a esa memoria (ART) habilitado, lectura anticipada deshabilitada, periféricos deshabilitados y 25 °C. Son valores típicos, no máximos garantizados. Las tensiones, frecuencias y procedimientos no son idénticos: sirven como referencia para comparar, pero no demuestran un porcentaje de ahorro ni representan el consumo del ordenador completo. El F446 también admite frecuencias inferiores; a 90 MHz publica 14 mA típicos bajo las condiciones indicadas.

Los tres cubren los presupuestos de memoria y permiten utilizar a la vez las interfaces previstas, incluyendo señales auxiliares y depuración, según la [asignación de pines propuesta en el anexo](../docs/ejercicio-1a-asignacion-pines.md). Se han considerado los pines disponibles en cada versión, no solo el número de interfaces anunciado para la familia. Esta comprobación respalda la disponibilidad de conexiones, pero las velocidades y la compatibilidad con los componentes concretos deberán verificarse al seleccionarlos.

## 4. Selección y comprobaciones posteriores

**Selección del microcontrolador.** Entre los tres candidatos considerados, se selecciona el **STM32F411RE** por su combinación de memoria, interfaces, capacidad de cálculo disponible y consumo activo publicado. Sus 128 KiB de RAM y 512 KiB de Flash superan los presupuestos, dejando diferencias de **68 KiB respecto a los 60 KiB de RAM previstos** y **320 KiB respecto a los 192 KiB de Flash**. Dispone de las interfaces previstas y FPU, y su consumo activo publicado permite considerarlo una opción razonable. Respecto a los otros dos candidatos, por ahora no se ha identificado una necesidad que haga imprescindible la frecuencia adicional del F446 ni la Flash adicional del L476. El L476 se considera también una alternativa válida; sin embargo, no se ha definido un reparto de tiempos entre actividad y reposo del ordenador que permita demostrar una ventaja energética decisiva a su favor, por lo que no se le da prioridad en esta selección.

Aun así, el **STM32F446RE** es también una opción muy adecuada y queda como alternativa preferente. De hecho, pasaría a ser el candidato principal si se decide priorizar un mayor margen de cálculo o si los algoritmos y la carga conjunta del programa muestran que el F411, incluso a 100 MHz, no cumple los plazos por falta de capacidad de procesamiento. Sus 180 MHz amplían este posible margen, aunque aumentar la frecuencia no resuelve necesariamente las esperas de sensores, almacenamiento o comunicaciones.

La elección queda justificada como diseño preliminar; la capacidad real de este microcontrolador para la misión deberá verificarse mediante la configuración, la ocupación de Flash y RAM, el uso máximo de pila, los tiempos de ejecución y el consumo. El proyecto básico del apartado 1.b permitirá comprobar la configuración solicitada, pero no validará las funciones del programa de vuelo que todavía no estén implementadas.
