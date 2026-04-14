**# Objetivo**

Simular, diseñar y construir un robot tipo brazo con 6 grados de libertad (6DOF), controlado por un microcontrolador. 

Capaz de:

- Moverse de manera precisa, con trayectorias definidas
- Coger objetos pequeños, de 1 kilogramo máximo con el brazo estirado completamente

El sistema se basara en:

- Un modelo dinamico simplificado que estimara el movimiento
- Control de lazo cerrado para mejorar la precision
- Diseño modular y parametrico para facilitar iteraciones y modificacion


**# Alcance**

- Diseño mecánico completo
- Simulación dinámica en Python o Matlab
- Implementación de control en STM32
- Validación experimental


**# Restricciones**

- Fabricación:
    - Piezas impresas en 3D (excepto: tornillería, ejes, rodamientos, electrónica, motores)
  
- Presupuesto:
  - Máximo: 300€
  
- Hardware:
    - Controlador: STM32
  
- Manufactura:
    - Volumen máximo: 220 × 220 × 250 mm (Ender 3 V1)
  
- Geometría:
    - Longitud máxima de piezas: ~300 mm (con impresión diagonal)