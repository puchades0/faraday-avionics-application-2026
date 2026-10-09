# faraday-avionics-application-2026
Prueba de acceso a Faraday Rocketry UPV (Equipo de Aviónica Software, 2026-27).

## Respuestas

- [Ejercicio 1.a: respuesta integrada — selección del STM32F411RE](respuestas/ejercicio-1a.md).
- [Ejercicio 1.b: configuración del proyecto en STM32CubeMX](respuestas/ejercicio-1b.md).
- [Ejercicio 2.a: selección de sensores y actualización del presupuesto](respuestas/ejercicio-2a.md).
- [Ejercicio 2.b: inicialización y lectura de sensores](respuestas/ejercicio-2b.md).

## Anexo

- [Ejercicio 1.a: comprobación de la asignación de pines](docs/ejercicio-1a-asignacion-pines.md).

## Proyecto STM32

El proyecto [exercise_1_2](firmware/exercise_1_2) contiene la configuración del STM32F411RET6 iniciada en el ejercicio 1.b y la integración de sensores del ejercicio 2.b: IMU, acelerómetro de alto rango, barómetro, receptor GNSS y medidas de alimentación y temperatura.

- [Configuración de STM32CubeMX](firmware/exercise_1_2/exercise_1_2.ioc).
- [Código principal](firmware/exercise_1_2/Core/Src/main.c).

Para revisar o modificar los periféricos, abrir el archivo `.ioc` con STM32CubeMX. Para compilar, abrir la carpeta `firmware/exercise_1_2` en VS Code con la extensión STM32CubeIDE, configurar el proyecto con GCC y seleccionar el preset **Debug** de CMake. Después, ejecutar **CMake: Build**.

Se ha comprobado la compilación del proyecto con los sensores integrados. No se ha probado su ejecución en una placa física. Las herramientas y la configuración inicial se describen en la respuesta del ejercicio 1.b; los ajustes y el funcionamiento de la adquisición se explican en el ejercicio 2.b.
