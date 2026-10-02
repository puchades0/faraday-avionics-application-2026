# Ejercicio 1.a — Comprobación de la asignación de pines

Para comprobar que los microcontroladores considerados permiten conectar los elementos previstos, se propone la siguiente distribución. Los pines indicados están disponibles en las versiones de 64 pines del **STM32F411RE, STM32F446RE y STM32L476RG**, y permiten utilizar las interfaces de la tabla sin repetir pines.

Los nombres como PA5 indican el pin 5 del puerto A del microcontrolador.

| Uso previsto | Interfaz o función | Pines asignados |
| --- | --- | --- |
| IMU y acelerómetro de alto rango | SPI1 | SCK: PA5; MISO: PA6; MOSI: PA7 |
| Selección de los dos sensores SPI | Salidas digitales independientes | IMU_CS: PA4; HIGHG_CS: PB0 |
| Almacenamiento externo | SPI2 | SCK: PB13; MISO: PB14; MOSI: PB15 |
| Selección del almacenamiento | Salida digital | PB12 |
| Barómetro | I²C1 | SCL: PB8; SDA: PB9 |
| Receptor GNSS | USART1 utilizada como UART | TX: PA9; RX: PA10 |
| Radio | USART2 utilizada como UART | TX: PA2; RX: PA3 |
| Batería y dos líneas reguladas | Tres entradas de ADC1 | PC0, PC1 y PC2 |
| Dos avisos de datos disponibles, si los sensores los proporcionan | Entradas digitales | PC4 y PC5 |
| Dos señales de orden para mecanismos | Salidas digitales | PC6 y PC7 |
| Dos señales de confirmación de mecanismos, si existen | Entradas digitales | PC8 y PC9 |
| Señal de armado y una señal adicional de control | Una entrada y una salida digital | PA0 y PA1, respectivamente |
| Programación y depuración con ST-LINK | SWD | SWDIO: PA13; SWCLK: PA14 |

La propuesta ocupa **28 pines de señales distintos**. TX es la salida que transmite datos y RX es la entrada que los recibe. En la tabla se indican desde el punto de vista del microcontrolador: su TX se conecta al RX del módulo, y su RX al TX del módulo.

Las señales de armado, orden y confirmación son una reserva inicial; su número definitivo dependerá de los mecanismos.

Esta distribución respalda la disponibilidad de pines indicada en la [respuesta del ejercicio 1.a](../respuestas/ejercicio-1a.md). De las conexiones previstas en la tabla, en el proyecto del ejercicio 1.b se han configurado SPI1, I²C1, las dos salidas CS de sensores y SWD; las demás quedan como previsión del diseño.

## Fuentes

Se han utilizado los diagramas de pines de las versiones de 64 pines y las tablas de funciones de las hojas de datos de ST:

- [STM32F411RE: figura 11 y tablas 8–9](https://www.st.com/resource/en/datasheet/stm32f411re.pdf).
- [STM32F446RE: figura 10 y tablas 10–11](https://www.st.com/resource/en/datasheet/stm32f446re.pdf).
- [STM32L476RG: figura 16 y tablas 16–17](https://www.st.com/resource/en/datasheet/stm32l476rg.pdf).
