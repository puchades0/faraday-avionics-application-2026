# Ejercicio 2.a — Selección de sensores

Se mantiene la misión de referencia y la arquitectura previstas en el [ejercicio 1.a](ejercicio-1a.md). A partir de estas se seleccionan los modelos de sensores para integrarlos en el proyecto del ejercicio 1.b. Los rangos y frecuencias indicados son las configuraciones propuestas para esta misión de diseño.

## 1. IMU de seis ejes

Se incluye una IMU formada por un acelerómetro y un giróscopo de tres ejes. Sus medidas permiten estudiar el movimiento del cohete, estimar la evolución de su orientación y aportar información para identificar las fases del vuelo. El giróscopo mide velocidad angular; la orientación se obtiene mediante el procesamiento de las medidas.

Se comparan dos modelos compatibles con alimentación y señales de 3,3 V, con comunicación SPI y controladores oficiales de ST en C:

| Característica | LSM6DSO | ISM330DHCX |
| --- | --- | --- |
| Rangos de aceleración seleccionables | ±2, ±4, ±8 y ±16 g | ±2, ±4, ±8 y ±16 g |
| Rango máximo de velocidad angular | ±2000 °/s | ±4000 °/s |
| Frecuencia de datos próxima a la prevista | 208 Hz | 208 Hz |
| Reloj SPI máximo | 10 MHz | 10 MHz |
| Temperatura de funcionamiento | −40 a +85 °C | −40 a +105 °C |
| Consumo típico conjunto en alto rendimiento, a 1,8 V y 25 °C | 0,55 mA | 1,2 mA |

Fuentes: hojas de datos del [LSM6DSO](https://www.st.com/resource/en/datasheet/lsm6dso.pdf) y del [ISM330DHCX](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf). Los consumos se comparan bajo las condiciones indicadas, no como medidas de nuestro montaje a 3,3 V.

Se selecciona el **ISM330DHCX**, principalmente por su mayor rango de velocidad angular. Como no se conoce el máximo de giro del vehículo y no se dispone de otro giróscopo que cubra una saturación, es preferible disponer de esta capacidad adicional. El **LSM6DSO** sería preferible si se prioriza reducir el consumo y se justifica que ±2000 °/s cubren las rotaciones previstas.

Se propone configurar el acelerómetro a **±16 g** y el giróscopo a **±4000 °/s**, con datos a **208 Hz**. El acelerómetro adicional de alto rango que se elige a continuación permitirá medir aceleraciones que superen el rango de la IMU. Según la tabla 2 de la hoja de datos del ISM330DHCX, a ±4000 °/s cada unidad de su lectura digital equivale nominalmente a 0,14 °/s, frente a 0,07 °/s a ±2000 °/s. Por tanto, ampliar el rango supone distinguir incrementos mayores de velocidad angular; estas cifras describen la resolución digital, no el error de la medida.

La conexión se realizará por **SPI1**, con **PA4 como IMU_CS**. En el siguiente apartado 2.b se ajustará el bus al **modo SPI 3** documentado por el fabricante. Los 208 Hz son la frecuencia nominal disponible en este modelo más próxima a los 200 Hz utilizados en el presupuesto inicial.

Para la integración se dispone del [controlador oficial de ST para ISM330DHCX](https://github.com/STMicroelectronics/ism330dhcx-pid). Sus funciones de acceso se adaptarán a SPI del STM32 y se comprobarán las opciones utilizadas para inicializar el sensor y convertir sus lecturas a unidades físicas.

## 2. Acelerómetro de alto rango

Se incluye un acelerómetro de tres ejes con mayor rango para seguir obteniendo medidas cuando se superen los ±16 g del acelerómetro de la IMU. Se comparan los siguientes modelos:

| Característica | ADXL375 | H3LIS331DL |
| --- | --- | --- |
| Rango nominal | ±200 g | Seleccionable: ±100, ±200 y ±400 g |
| Incremento nominal de la salida a ±200 g | 0,049 g | 0,098 g |
| Frecuencia de datos propuesta | 200 Hz | 400 Hz |
| Reloj SPI máximo | 5 MHz | 10 MHz |
| Alimentación y señales compatibles con 3,3 V | Sí | Sí |

Fuentes: hojas de datos del [ADXL375](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.pdf) y del [H3LIS331DL](https://www.st.com/resource/en/datasheet/h3lis331dl.pdf). Los incrementos indican la resolución digital de la salida, no la exactitud de las medidas.

Se selecciona el **H3LIS331DL**, configurado inicialmente a **±200 g**, por la posibilidad de ajustar el rango a ±100 o ±400 g si la información posterior del vuelo lo aconseja. Se prioriza esta posibilidad de ajustar el rango, aceptando a cambio una resolución digital menor que la del ADXL375 a ±200 g. La IMU aporta medidas más finas dentro de su rango; cuando este se supera, la medida depende del acelerómetro de alto rango y de su resolución.

Se utilizará a **400 Hz**, la menor de sus frecuencias disponibles en funcionamiento normal (50, 100, 400 y 1000 Hz) que no queda por debajo de los 200 Hz previstos inicialmente. Se prevé adquirir y registrar todas las muestras del sensor, por lo que esta elección aumenta el volumen de datos respecto al presupuesto inicial. Compartirá **SPI1** con la IMU, utilizando **PB0 como HIGHG_CS**. Se mantiene el reloj SPI de **6,25 MHz** del proyecto, que resulta de dividir los 100 MHz de APB2 entre 16 y queda por debajo del máximo de 10 MHz tanto del **H3LIS331DL** como de la **IMU ISM330DHCX**.

El **ADXL375** sería preferible si se prioriza una resolución digital más fina a ±200 g y trabajar directamente a 200 Hz. Su límite de reloj SPI no impide utilizarlo: bastaría con cambiar el divisor de SPI1 de 16 a 32 para obtener 3,125 MHz, por debajo de su máximo de 5 MHz. Sin embargo, su fabricante advierte de posibles interferencias al compartir SPI con otros dispositivos y propone una puerta lógica de protección cuando no se pueda garantizar que estas condiciones no ocurran. Esta particularidad se tendría que resolver en caso de integrarlo, pues compartiría SPI con la IMU; esto se describe en el apartado «Preventing Bus Traffic Errors», página 15 de su hoja de datos.

## 3. Barómetro

Se incluye un sensor de presión absoluta para estimar la altitud y aportar información sobre el ascenso y el descenso. El sensor mide presión; la altitud se calcula mediante un modelo atmosférico, tomando también una referencia antes del lanzamiento.

A 15 km sobre el nivel del mar, el [modelo atmosférico de la NASA](https://www.grc.nasa.gov/www/k-12/BGP/atmosmet.html) da aproximadamente **121 hPa**. Como la misión considera 15 km sobre el lanzamiento, hay que sumar la elevación del lugar para estimar la presión mínima y dejar margen para las variaciones atmosféricas. No se adopta, por tanto, 121 hPa como límite inferior del sensor. En estas unidades, 1 hPa equivale a 1 mbar.

| Característica | MS5607-02BA03 | AMS 5935-1500-A |
| --- | --- | --- |
| Rango de presión | 10–2000 hPa, incluyendo el rango extendido | 0–1500 hPa |
| Cobertura de la presión prevista cerca del apogeo | En el rango extendido, sin límites de error publicados para esa presión | En el rango calibrado con error especificado |
| Alimentación | 1,8–3,6 V | 1,7–3,6 V |
| Interfaces | I²C y SPI | I²C y SPI |
| Temperatura de funcionamiento | −40 a +85 °C | −25 a +85 °C |

Fuentes: [hoja de datos del MS5607](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocFormat=pdf&DocLang=English&DocNm=MS5607-02BA03&DocType=Data+Sheet&PartCntxt=MS560702BA03-50) y [características y variantes del AMS 5935](https://www.analog-micro.com/en/products/pressure-sensors/board-mount-pressure-sensors/ams5935/).

Se selecciona el **AMS 5935-1500-A** porque permite justificar el error de presión también en la parte más alta del vuelo. Además, el límite superior de 1500 hPa deja margen respecto a la presión atmosférica del lanzamiento. Se acepta a cambio su mayor tamaño y se requiere mantener el sensor dentro de su intervalo de temperatura. El **MS5607** sería una alternativa si se priorizan el tamaño reducido o el funcionamiento a temperaturas inferiores a −25 °C, siempre que se pueda caracterizar su error a bajas presiones. Su intervalo principal de precisión especificada es de 300 a 1100 hPa; no se puede extender esta precisión al apogeo aunque se sigan obteniendo lecturas.

Para el AMS, el error total especificado entre −25 y +85 °C es **±0,25 % del intervalo de medida**, equivalente a **±3,75 hPa** en la variante de 1500 hPa. A unos 121 hPa, al propagar ese error mediante el modelo de la NASA, resulta aproximadamente **−194 a +200 m de error en la altitud calculada**. Esta estimación solo recoge el error del sensor; las diferencias respecto a la atmósfera del modelo y la toma de presión introducen otros errores. Medir la temperatura exterior puede ayudar a corregir el modelo atmosférico, pero no elimina este error de presión del sensor. Este intervalo describe el posible error de altitud respecto al modelo, no cuánto variarán dos lecturas consecutivas. Se utilizará junto a los demás sensores para estimar el estado del cohete.

Se conectará a **I²C1, a 100 kHz y 3,3 V**, manteniendo las **50 muestras/s** previstas, es decir, una muestra cada 20 ms. Se propone el modo de medición simple, cuyo tiempo de conversión indicado es de 4 ms, dejando margen para leer el resultado antes de iniciar la siguiente medida. El programa iniciará la conversión y atenderá otras tareas hasta que el sensor indique que ha terminado. La presión y la temperatura se reciben compensadas y se convierten a unidades físicas mediante las ecuaciones del fabricante. Véase la [hoja de datos del AMS 5935, páginas 4–6, 10–11 y 15](https://www.analog-micro.com/products/pressure-sensors/board-mount-pressure-sensors/ams5935/ams5935-datasheet.pdf).

El AMS utiliza una medida interna de temperatura para compensar su lectura de presión. Esta lectura puede verse afectada por el calentamiento de la electrónica, por lo que no representa necesariamente la temperatura del aire exterior.

La altitud se estimará mediante un modelo atmosférico por capas y la referencia tomada antes del lanzamiento. Una sonda de temperatura exterior podría ayudar a corregir las diferencias entre la atmósfera real y el modelo. Sin embargo, la relación entre presión y altura depende de la temperatura media de la capa de aire atravesada, no solo de la temperatura en el punto donde está el cohete. Por ello, sus medidas tendrían que incorporarse a un modelo o utilizarse para estimar cómo cambia la temperatura durante el ascenso. Para esta propuesta se mantiene el cálculo con el modelo atmosférico, sin añadir una sonda exterior, aceptando el error asociado a esa aproximación. Esta dependencia se explica en la [relación entre presión, altura y temperatura del National Weather Service](https://www.weather.gov/source/zhu/ZHU_Training_Page/Miscellaneous/Heights_Thicknesses/thickness_temperature.htm).

Se mantiene un único barómetro para todo el vuelo, ya que su rango cubre las presiones previstas. Para los requisitos establecidos no se considera necesario añadir otro especializado en bajas presiones. Esta opción se reconsideraría en caso de exigirse una menor incertidumbre de la altitud barométrica cerca del apogeo.

## 4. Receptor GNSS

Se incluye un receptor GNSS para obtener posición y velocidad de forma independiente de los sensores inerciales y del barómetro. Su función principal es facilitar la localización de la etapa superior durante el descenso y después del aterrizaje, aunque también se aprovecharán sus datos en otros tramos del vuelo cuando sean válidos.

| Característica | MAX-M10S | NEO-M9N |
| --- | --- | --- |
| Altitud máxima publicada | 80 km | 80 km |
| Velocidad máxima publicada | 500 m/s | 500 m/s |
| Límite dinámico publicado | ≤4 g | ≤4 g |
| Frecuencia de navegación | Admite los 10 Hz previstos con GPS + Galileo sin activar alto rendimiento | Hasta 25 Hz, también con cuatro constelaciones |
| Alimentación y señales compatibles con 3,3 V | Sí | Sí |
| Interfaces | UART e I²C | UART, I²C, SPI y USB |
| Consumo típico orientativo con GPS, seguimiento continuo, 1 Hz y 3 V | 9,6 mA, sumando VCC y V_IO | 28 mA |

Fuentes: hojas de datos del [MAX-M10S](https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf) y del [NEO-M9N](https://content.u-blox.com/sites/default/files/NEO-M9N-00B_DataSheet_UBX-19014285.pdf). Los límites operativos corresponden al modelo de navegación Airborne 4g y los consumos son orientativos bajo las condiciones indicadas.

Se selecciona el **MAX-M10S** porque permite mantener los 10 Hz previstos y presenta un consumo orientativo menor. El **NEO-M9N** sería preferible si se necesitase una mayor frecuencia con varias constelaciones simultáneas, pero no resolvería los límites de velocidad y aceleración.

La altura prevista está dentro del límite publicado de ambos receptores. Sin embargo, Mach 2,5 supera los 500 m/s en las condiciones atmosféricas previstas y durante la propulsión podrían superarse los 4 g. Por ello, no se cuenta con disponer de medidas GNSS válidas durante todo el ascenso. Cerca del apogeo, la reducción de velocidad puede favorecer su funcionamiento, pero que la velocidad vertical sea nula no basta, ya que también deben cumplirse los límites de velocidad total y aceleración. Además, al volver a estas condiciones, el receptor puede necesitar tiempo para recuperar una posición válida.

Se mantendrá el GNSS funcionando durante el vuelo y se utilizarán únicamente soluciones válidas y recientes. El ordenador deberá continuar sus funciones esenciales cuando esos datos no estén disponibles. Se comprobarán la validez, la antigüedad y las estimaciones de error recibidas antes de utilizar una posición o velocidad. El [manual de integración del MAX-M10S](https://content.u-blox.com/sites/default/files/MAX-M10S_IntegrationManual_UBX-20053088.pdf) describe los modelos de navegación y la comprobación de validez de las soluciones.

En cuanto a la medida del apogeo, para mejorar su exactitud se pueden analizar los datos GNSS válidos alrededor del máximo y contrastarlos con los registros barométricos e inerciales, utilizando una misma referencia de altura respecto al lanzamiento.

Se propone **GPS + Galileo a 10 Hz**, con el modelo **Airborne <4g**, y conexión a **USART1 utilizada como UART**, mediante PA9 (TX) y PA10 (RX), según la asignación del ejercicio 1.a. Se utilizarán alimentación y señales de 3,3 V. El receptor necesitará una antena GNSS adecuada; las coordenadas llegarán al STM32 y será la radio de telemetría la que permita transmitirlas a tierra.

## 5. Supervisión de alimentación y temperatura

Para estas medidas se aprovechan el **ADC de 12 bits y el sensor interno de temperatura del STM32F411RE**, sin añadir un sensor específico externo.

| Medida | Recurso utilizado | Frecuencia de lectura y registro |
| --- | --- | --- |
| Tensión de batería | ADC1, entrada PC0 | 10 Hz |
| Primera línea regulada | ADC1, entrada PC1 | 10 Hz |
| Segunda línea regulada | ADC1, entrada PC2 | 10 Hz |
| Temperatura del microcontrolador | Sensor interno, leído mediante ADC1 | 1 Hz |

Las tensiones se medirán sucesivamente con el mismo ADC, y se utilizarán divisores resistivos para adaptar las señales a su rango de entrada. Sus proporciones se fijarán según la tensión máxima de cada línea, incluida la batería completamente cargada, dejando margen respecto al límite admitido. El programa reconstruirá las tensiones originales a partir de las lecturas y de esas proporciones.

También se leerá la tensión de referencia interna (VREFINT) a 10 Hz. Esta lectura, junto con su calibración de fábrica, permite estimar la alimentación analógica real y corregir la conversión de las lecturas a voltios. No requiere un pin externo y se utilizará como dato auxiliar para calcular las tres tensiones.

Estas medidas se utilizarán para comprobar si la tensión de la batería y las líneas reguladas se mantienen dentro de los valores previstos. No permiten conocer por sí solas la carga restante de la batería ni detectar todos los fallos de alimentación de cada componente. Al tomar una medida cada 100 ms, pueden pasar inadvertidas caídas más breves; la protección frente a estas debe resolverse en el diseño de la alimentación.

La temperatura se calculará utilizando los datos de calibración de fábrica del STM32 y se empleará para observar el calentamiento del propio microcontrolador. Esta lectura no representa la temperatura de toda la placa o del ambiente. La lectura de temperatura del barómetro permitirá también comprobar si éste se mantiene dentro de su intervalo de funcionamiento. Si se necesitase una medida precisa en la batería o en otro punto concreto, habría que añadir un sensor situado en dicho punto.

Fuente: [hoja de datos del STM32F411RE](https://www.st.com/resource/en/datasheet/stm32f411re.pdf), apartados 3.29, 3.30, 6.3.21 y 6.3.23, y tabla de funciones de los pines.

## 6. Actualización del presupuesto del ejercicio 1.a

Se mantiene el formato de los registros del ejercicio 1.a. Las frecuencias que cambian son las de la IMU, de 200 a **208 Hz**, y del acelerómetro de alto rango, de 200 a **400 Hz**. Sus registros siguen ocupando 32 y 20 bytes, respectivamente. Por tanto, el volumen previsto pasa a ser:

**16 532 + (208 − 200) × 32 + (400 − 200) × 20 = 20 788 bytes/s.**

Se mantienen las demás frecuencias, incluido el estado estimado a 100 Hz. Además, la temperatura del barómetro ya estaba incluida en su registro y la del STM32 ocupa el registro de temperatura de la electrónica. VREFINT se utiliza para calcular las tensiones, sin añadir un registro independiente.

Así que, con el mismo margen del 25 % para eventos y campos auxiliares:

**20 788 × 1,25 = 25 985 bytes/s.**

Para acumular un segundo de datos se necesitan **25 985 / 1024 ≈ 25,4 KiB**, por lo que el búfer inicial de 24 KiB se amplía a **32 KiB**. Se mantienen las reservas de 4 KiB para comunicaciones y últimas medidas y de 32 KiB para el resto del software:

**RAM presupuestada = 32 + 4 + 32 = 68 KiB.**

El STM32F411RE dispone de 128 KiB, dejando **60 KiB respecto al presupuesto actualizado**. Se mantiene la reserva inicial de Flash de 192 KiB; la ocupación efectiva se comprobará al integrar y compilar el programa.

Debido al cambio de frecuencia, la adquisición del acelerómetro de alto rango debe atender ahora muestras cada **2,5 ms**, aunque la estimación del estado siga realizándose cada 10 ms. Esta selección mantiene la viabilidad en cuanto a memoria e interfaces; el cumplimiento de los tiempos se comprobará durante la implementación. En el siguiente apartado 2.b se ajustará SPI1 al modo 3 y se incorporarán USART1 y ADC1 al proyecto existente, junto con la inicialización y lectura de los sensores seleccionados.
