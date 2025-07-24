 

# Informe de Proyecto Final
# Actualización del Laboratorio CIM


Alumno: Jorge Nicolas Franco
DNI: 30353371
Nombre de la Institución: Laboratorio CIM. UNLZ-FI
Tutor institucional y académico: Martin González
Año: 2025









## Resumen

El propósito de este proyecto surgió ante la necesidad de reemplazar el PLC Sysmac CQM1 de Omron, que había quedado obsoleto y ya no garantizaba el funcionamiento adecuado de la cinta transportadora de cuatro estaciones. Para abordar esta actualización, se optó por implementar un Arduino Mega como controlador principal del sistema.
Un paso fundamental fue realizar un análisis detallado del equipo existente mediante ingeniería inversa. Esto incluyó el estudio de los sensores presentes en cada estación y el sistema de cableado eléctrico. Esta evaluación permitió adaptar y optimizar el nuevo controlador, asegurando una operación eficiente con tecnologías más modernas.
El sistema utiliza sensores magnéticos para identificar los pallets y topes electromecánicos para detenerlos. Como parte de las pruebas iniciales, el Arduino fue programado para detectar los pallets y detenerlos en una de las cuatro estaciones, donde se realiza la lectura de su identificación.















## Esquema general del Proyecto
 <img width="1148" height="858" alt="Imagen1" src="https://github.com/user-attachments/assets/b73b17f1-f5ef-4ee6-a914-bf25a0694494" />



## Estado Inicial del CIM

<img width="252" height="188" alt="Imagen2" src="https://github.com/user-attachments/assets/96634f45-844d-43e2-8286-f9549ec18edc" />
<img width="106" height="188" alt="Imagen3" src="https://github.com/user-attachments/assets/8b622130-829d-4d0d-924d-c326ae1fb664" />
<img width="141" height="189" alt="Imagen4" src="https://github.com/user-attachments/assets/ceb174ef-03bb-4f5f-9114-a18302cf1a0f" />
<img width="252" height="189" alt="Imagen5" src="https://github.com/user-attachments/assets/6ef75674-3057-4c62-b69d-21a1a6fe0c42" />
<img width="256" height="192" alt="Imagen6" src="https://github.com/user-attachments/assets/8af3aded-8c90-4d99-bf92-f4fe0cc72a02" />
<img width="250" height="189" alt="Imagen7" src="https://github.com/user-attachments/assets/14270f3d-b168-44f2-9cb5-aad3529a0132" />
<img width="142" height="189" alt="Imagen8" src="https://github.com/user-attachments/assets/8684da7b-b0ca-42e3-b57c-7b6cdba6bff3" />
<img width="142" height="189" alt="Imagen9" src="https://github.com/user-attachments/assets/5d6b4dd0-713b-4440-821a-b48e5b9fe1f8" />
<img width="141" height="189" alt="Imagen10" src="https://github.com/user-attachments/assets/e10d025c-7986-4208-b16a-58c9d283f85b" />
<img width="107" height="189" alt="Imagen11" src="https://github.com/user-attachments/assets/b1d98ba2-a145-4db8-a528-e4c1d1d7ad4a" />


## Componentes instalados durante la actualización:
<img width="142" height="189" alt="Imagen12" src="https://github.com/user-attachments/assets/7af92823-831b-4e85-8f94-3cdcb10fef54" />
<img width="240" height="189" alt="Imagen13" src="https://github.com/user-attachments/assets/4c9d4bbf-890f-4013-b882-8ecc2404e692" />
<img width="107" height="189" alt="Imagen14" src="https://github.com/user-attachments/assets/85f72073-d242-4c15-8951-a89376b4b4f4" />


## Componentes

Cinta Transportadora:
	Posee 4 estaciones de trabajo.
	Cada estación cuenta con 5 sensores mecánicos, 2 topes electromecánicos y un semáforo de señalización (verde: start, rojo: stop).
<img width="189" height="142" alt="Imagen15" src="https://github.com/user-attachments/assets/b0be33f3-b0c0-406f-a98c-97d5e9565183" />



  

Pallets:
	Los pallets transportan los materiales, cada uno posee 5 imanes.
	4 imanes indican la identificación o número de pallet con notación binaria.
	El 5to imán es el “In place”, indica que el Pallet está posicionado en la estación.
 ![Imagen16](https://github.com/user-attachments/assets/55099c4a-9313-4ae5-b25f-61cd8dfc5202)


Arduino Mega:
	Reemplaza el antiguo PLC.
	Lee las señales de los sensores de la cinta.
	Acciona los topes electromecánicos para leer el número de Pallet.
	Acciona las señales luminosas, rojo “stop” y verde “run”.
	Se comunica con el Manager (PC) a través del protocolo RS-232.
 <img width="553" height="268" alt="Imagen17" src="https://github.com/user-attachments/assets/f99d512d-5441-4ddb-9024-5e83008a552a" />



Placa de Optoacopladores:
	Los optoacopladores adaptan las señales de los sensores magnéticos al nivel de voltaje requerido por el Arduino Mega (de 24V a 5V) y protegen al sistema contra picos y fluctuaciones eléctricas.
 ![Imagen18](https://github.com/user-attachments/assets/2296ff7c-899a-40b4-bd98-6bfe64f51fbe)


Sensores Magnéticos:
	Detectan los imanes que poseen pallets y envían la señal amplificada a la placa de optoacopladores, a su vez esta última se comunica con el Arduino mega.
 ![Imagen19](https://github.com/user-attachments/assets/73156121-e3c1-43f0-bbfe-7b99947faf82)


Topes Electromecánicos:
	Cada estación posee 2 topes que actúan en simultaneo, el primero detiene el Pallet a leer y el segundo asegura un gap con el Pallet siguiente para evitar colisiones entre estos.
	Se activan cuando el primer imán del Pallet pasa por el primer sensor magnético (ID3 “Irriving”) de una estación.
	Detienen el pallet para leer su identificación.
 ![Imagen20](https://github.com/user-attachments/assets/603f1d7b-4bb1-4206-92d5-c67b4663780e)




Comunicación con la PC:
	El Arduino envía el número de pallet a la PC vía RS-232
	El operador de la PC decide si se realiza una operación o si el Pallet continúa avanzando hacia la siguiente estación.


Relés:
	Energizan y des energizan los topes electromecánicos para detener los Pallets en cada estación.}
 <img width="497" height="371" alt="Imagen21" src="https://github.com/user-attachments/assets/bdf2c28a-a469-46d0-a503-537c25878312" />


## ingeniería Inversa 
## Identificación de terminales del PLC
Antes de avanzar con el diseño del circuito electrónico se realizó un relevamiento de las conexiones eléctricas del PLC a fin de identificar los sensores magnéticos y demás componentes de entrada y salida del sistema.
Dicho PLC posee 3 módulos de conexiones donde cada módulo tiene 16 puntos de conexión.
	Modulo 1 – “OCH” (En general salidas de los sensores magnéticos)
	Modulo 2 – “ID212” (“INPLACE”)
	Modulo 3 – “OC222” (“Salidas Topes Electromecánicos y semáforos)
 ![Imagen22](https://github.com/user-attachments/assets/12c294c4-930e-4793-9953-f224dbb19fa9)



## Identificación de los pines de las 4 estaciones
 <img width="425" height="151" alt="Imagen23" src="https://github.com/user-attachments/assets/27d5edc2-90b1-432d-9b46-ec21e369cddd" />




## Esquema eléctrico en Proteus
<img width="1227" height="529" alt="Imagen24" src="https://github.com/user-attachments/assets/188d666a-a193-488a-8ad3-958bc8319ba1" />

## Diagrama eléctrico general
	Subcircuito: Placa interface PLC
	Subcircuitos: Optoacopladores x 4
	Arduino Mega
	Subcircuito: Relés



## Subcircuito: Placa interfaz PLC

 <img width="1120" height="1003" alt="Imagen25" src="https://github.com/user-attachments/assets/ecf12756-44f9-4b7e-9649-2d50ca1b794b" />



## Subcircuito: Optoacopladores x 4

 <img width="673" height="1573" alt="Imagen26" src="https://github.com/user-attachments/assets/6a51c7a3-40cb-4dbe-8081-2bd105293a10" />



## Arduino Mega

 <img width="1216" height="936" alt="Imagen27" src="https://github.com/user-attachments/assets/cbdf1a75-17d1-4fe2-af59-fb18c3797e39" />




## Subcircuito: Relés x 8
 <img width="603" height="1731" alt="Imagen28" src="https://github.com/user-attachments/assets/03907012-ab65-4ccc-8dc4-15afe0baff04" />

# Programación
El código del Arduino debe:
	Leer señales de los sensores magnéticos.
	Cuando el primer imán de un pallet es detectado por el sensor ID3 (arriving) se debe accionar el tope electromecánico de la estación correcta.
	Cuando el sensor IN PLACE detecte un imán, se deberá tomar lectura del pallet para identificarlo.
	Se debe transmitir el número de Pallet y esperar la instrucción del Manager para continuar.
	Si el Manager indica que el pallet debe continuar, poner en 0 la salida del relé que energiza el tope (freno del pallet).
	Tope hacia arriba = semáforo en rojo, tope desactivado = semáforo en verde.
	Se utilizarán interrupciones para detener el pallet y luego leer el nro. de identificación.

## Primer código. Permite leer la identificación de los Pallets que pasan por cualquiera de las 4 estaciones de la cinta transportadora. Imprime el número de estación e identificación del Pallet.
#include <LiquidCrystal.h>

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  lcd.begin(16, 2);
  
  pinMode(26, INPUT);
  pinMode(33, INPUT);
  pinMode(A4, INPUT);
  pinMode(A12, INPUT);
  pinMode(13, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(8, OUTPUT);

  Serial.begin(9600);
  
}
void loop() 
{
  
  int i = digitalRead(26);  // estacion a
  int j = digitalRead(33);  // estacion b
  int k = digitalRead(A4);  // estacion c
  int l = digitalRead(A12);  // estacion d
  
if(digitalRead(26)==1)
     { 
      i=PINC;
      i=PINC&0b00001111;
      digitalWrite(13, HIGH); 
      Serial.println("Estacion 1 ON");   
      Serial.print("carro nro "); Serial.println(i,HEX); 
     }
     else{
      digitalWrite(13,LOW);
      Serial.println("Estacion 1 OFF"); 
     }

if(digitalRead(33)==1)
     { 
      j=PINC;
      j=PINC&0b00001111;
      digitalWrite(9, HIGH);
      Serial.println("Estacion 2 ON");  
      Serial.print("carro nro "); Serial.println(j,HEX);  

  }
     else{
      digitalWrite(9,LOW);
      Serial.println("Estacion 2 OFF");
     }

if(digitalRead(A4)==1)
     { 
      k=PINF;
      k=PINF&0b00001111;
      digitalWrite(10, HIGH);   
      Serial.println("Estacion 3 ON");  
      Serial.print("carro nro "); Serial.println(k,HEX);
     }
     else{
      digitalWrite(10,LOW);
      Serial.println("Estacion 3 OFF"); 
     }

if(digitalRead(A12)==1)
     { 
      l=PINK;
      l=PINK&0b00001111;
      digitalWrite(8, HIGH);  
      Serial.println("Estacion 4 ON");
      Serial.print("carro nro "); Serial.println(l,HEX); 
     }
     else{
      digitalWrite(8,LOW);
      Serial.println("Estacion 4 OFF");
     }
       //lcd.noDisplay();
       //delay(500);
       lcd.display();
       delay(300);
       lcd.print("a");
       lcd.print(i);
       //if(i==0b00000001)
     //{ 
      //lcd.print("p1_c1");
     //}
       delay(500);
       lcd.print("b");
       lcd.print(j); 
       delay(500); 
       lcd.print("c");
       lcd.print(k);    
       delay(500); 
       lcd.print("d");
       lcd.print(l);  
}

## Código Arduino mejorado para las 4 Estaciónes:

#include <avr/interrupt.h>
#include <Arduino.h>

// =================================================================
// 1. DEFINICIONES DE HARDWARE (ASIGNACIÓN DE PINES)
// =================================================================
// --- Estación A ---
#define SENSOR_A_ID0      PA0   // Pin 22
#define SENSOR_A_ID1      PA1   // Pin 23
#define SENSOR_A_ID2      PA2   // Pin 24
#define SENSOR_A_ID3      PD0   // Pin 21 (INT0) Arriving
#define SENSOR_A_INPLACE  PA3   // Pin 25
#define ACTUADOR_A_PIN    PF0   // Asignado al Puerto F

// --- Estación B ---
#define SENSOR_B_ID0      PA4   // Pin 26
#define SENSOR_B_ID1      PA5   // Pin 27
#define SENSOR_B_ID2      PA6   // Pin 28
#define SENSOR_B_ID3      PD1   // Pin 20 (INT1) Arriving
#define SENSOR_B_INPLACE  PA7   // Pin 29
#define ACTUADOR_B_PIN    PF1   // Asignado al Puerto F

// --- Estación C ---
#define SENSOR_C_ID0      PC0   // Pin 37
#define SENSOR_C_ID1      PC1   // Pin 36
#define SENSOR_C_ID2      PC2   // Pin 35
#define SENSOR_C_ID3      PD2   // Pin 19 (INT2) Arriving
#define SENSOR_C_INPLACE  PC3   // Pin 34
#define ACTUADOR_C_PIN    PF2   // Asignado al Puerto F

// --- Estación D ---
#define SENSOR_D_ID0      PC4   // Pin 33
#define SENSOR_D_ID1      PC5   // Pin 32
#define SENSOR_D_ID2      PC6   // Pin 31
#define SENSOR_D_ID3      PD3   // Pin 18 (INT3) Arriving
#define SENSOR_D_INPLACE  PC7   // Pin 30
#define ACTUADOR_D_PIN    PF3   // Asignado al Puerto F

// =================================================================
// 2. PARÁMETROS DE TIEMPO (AJUSTABLES EN MILISEGUNDOS)
// =================================================================
#define TIEMPO_FRENADO_MS 500  // Tiempo que el actuador permanece activado antes de esperar la señal "inplace".
#define TIEMPO_LECTURA_MS 20  // Pausa adicional después de leer el ID y antes de liberar el pallet.

// =================================================================
// 3. ESTRUCTURAS Y VARIABLES GLOBALES
// =================================================================

#define NUM_ESTACIONES 4
const char NOMBRES_ESTACION[NUM_ESTACIONES] = {'A', 'B', 'C', 'D'};

// El desbordamiento del Timer2 con preescalador 1024 ocurre cada ~16 ms.
// Calculamos cuántos desbordamientos se necesitan para alcanzar el tiempo de frenado.
const int OVERFLOWS_PARA_FRENADO = TIEMPO_FRENADO_MS / 16;

struct EstadoEstacion {
  volatile bool palletDetectado = false;
  volatile bool tiempoDeFrenadoCumplido = false;
  volatile int contadorOverflows = 0;
};

EstadoEstacion estados[NUM_ESTACIONES];

// Prototipos de funciones auxiliares
void gestionarEstacion(int i);
uint8_t leerID(int estacionIndex);
void imprimirID(int estacionIndex, uint8_t id);
bool leerSensorInplace(int estacionIndex);
void controlarActuador(int estacionIndex, bool activar);
void configurarInterrupcionesExternas();
void configurarTimer2();

// =================================================================
// 4. CONFIGURACIÓN INICIAL (setup)
// =================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Sistema de estaciones iniciado.");
  Serial.print("Tiempo de frenado configurado a ");
  Serial.print(TIEMPO_FRENADO_MS);
  Serial.println(" ms.");
  Serial.print("Tiempo de lectura configurado a ");
  Serial.print(TIEMPO_LECTURA_MS);
  Serial.println(" ms.");

  // --- Configuración de Pines de Entrada (Sensores) ---
  DDRA &= ~((1 << SENSOR_A_ID0) | (1 << SENSOR_A_ID1) | (1 << SENSOR_A_ID2) | (1 << SENSOR_A_INPLACE) | (1 << SENSOR_B_ID0) | (1 << SENSOR_B_ID1) | (1 << SENSOR_B_ID2) | (1 << SENSOR_B_INPLACE));
  PORTA |= (1 << SENSOR_A_ID0) | (1 << SENSOR_A_ID1) | (1 << SENSOR_A_ID2) | (1 << SENSOR_A_INPLACE) | (1 << SENSOR_B_ID0) | (1 << SENSOR_B_ID1) | (1 << SENSOR_B_ID2) | (1 << SENSOR_B_INPLACE);

  DDRC &= ~((1 << SENSOR_C_ID0) | (1 << SENSOR_C_ID1) | (1 << SENSOR_C_ID2) | (1 << SENSOR_C_INPLACE) | (1 << SENSOR_D_ID0) | (1 << SENSOR_D_ID1) | (1 << SENSOR_D_ID2) | (1 << SENSOR_D_INPLACE));
  PORTC |= (1 << SENSOR_C_ID0) | (1 << SENSOR_C_ID1) | (1 << SENSOR_C_ID2) | (1 << SENSOR_C_INPLACE) | (1 << SENSOR_D_ID0) | (1 << SENSOR_D_ID1) | (1 << SENSOR_D_ID2) | (1 << SENSOR_D_INPLACE);

  DDRD &= ~((1 << SENSOR_A_ID3) | (1 << SENSOR_B_ID3) | (1 << SENSOR_C_ID3) | (1 << SENSOR_D_ID3));
  PORTD |= (1 << SENSOR_A_ID3) | (1 << SENSOR_B_ID3) | (1 << SENSOR_C_ID3) | (1 << SENSOR_D_ID3);

  // --- Configuración de Pines de Salida (Actuadores) ---
  DDRF |= (1 << ACTUADOR_A_PIN) | (1 << ACTUADOR_B_PIN) | (1 << ACTUADOR_C_PIN) | (1 << ACTUADOR_D_PIN);
  PORTF |= (1 << ACTUADOR_A_PIN) | (1 << ACTUADOR_B_PIN) | (1 << ACTUADOR_C_PIN) | (1 << ACTUADOR_D_PIN);

  // --- Configuración de Interrupciones ---
  configurarInterrupcionesExternas();
  configurarTimer2();

  sei();
}

// =================================================================
// 5. BUCLE PRINCIPAL (loop)
// =================================================================
void loop() {
  for (int i = 0; i < NUM_ESTACIONES; i++) {
    gestionarEstacion(i);
  }
}

// =================================================================
// 6. LÓGICA PRINCIPAL POR ESTACIÓN (CORREGIDA)
// =================================================================
void gestionarEstacion(int i) {
  if (!estados[i].palletDetectado) {
    return;
  }

  // --- INICIO DE LA CORRECCIÓN PARA EL PALLET 8 ---
  // Si el sensor 'inplace' está activo justo cuando llega el pallet,
  // omitimos el temporizador de frenado para evitar la condición de carrera.
  if (leerSensorInplace(i) && !estados[i].tiempoDeFrenadoCumplido) {
    estados[i].tiempoDeFrenadoCumplido = true;
  }
  // --- FIN DE LA CORRECCIÓN ---

  // PASO 1: Pallet detectado, se activa el freno hasta que se cumpla el tiempo.
  if (estados[i].tiempoDeFrenadoCumplido == false) {
    controlarActuador(i, true);  // Activa el actuador
  }
  // PASO 2: Tiempo cumplido (o saltado), ahora se espera la posición final.
  else {
    if (leerSensorInplace(i)) {  // Si el sensor de posición está activado...

// --- PROCESO COMPLETADO PARA ESTA ESTACIÓN ---
      uint8_t id = leerID(i);
      imprimirID(i, id);
      
delay(TIEMPO_LECTURA_MS);

controlarActuador(i, false);  // Desactiva el actuador

// --- REINICIO DE ESTADO PARA LA PRÓXIMA LECTURA ---
      estados[i].palletDetectado = false;
      estados[i].tiempoDeFrenadoCumplido = false;
      estados[i].contadorOverflows = 0;

EIFR |= (1 << (INTF0 + i));
EIMSK |= (1 << (INT0 + i));
    }
  }
}

// =================================================================
// 7. RUTINAS DE SERVICIO DE INTERRUPCIÓN (ISR)
// =================================================================

ISR(INT0_vect) {
  estados[0].palletDetectado = true;
  EIMSK &= ~(1 << INT0);
}
ISR(INT1_vect) {
  estados[1].palletDetectado = true;
  EIMSK &= ~(1 << INT1);
}
ISR(INT2_vect) {
  estados[2].palletDetectado = true;
  EIMSK &= ~(1 << INT2);
}
ISR(INT3_vect) {
  estados[3].palletDetectado = true;
  EIMSK &= ~(1 << INT3);
}

// --- ISR para el Desbordamiento del Timer2 ---
ISR(TIMER2_OVF_vect) {
  for (int i = 0; i < NUM_ESTACIONES; i++) {
    if (estados[i].palletDetectado && !estados[i].tiempoDeFrenadoCumplido) {
      estados[i].contadorOverflows++;
      if (estados[i].contadorOverflows >= OVERFLOWS_PARA_FRENADO) {
        estados[i].tiempoDeFrenadoCumplido = true;
        estados[i].contadorOverflows = 0;
      }
    }
  }
}

// =================================================================
// 8. FUNCIONES AUXILIARES
// =================================================================

void configurarInterrupcionesExternas() {
  EICRA = (1 << ISC01) | (1 << ISC11);
  EICRB = (1 << ISC21) | (1 << ISC31);
  EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);
}

void configurarTimer2() {
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << TOIE2);
}

uint8_t leerID(int estacionIndex) {
  uint8_t pina = PINA;
  uint8_t pinc = PINC;
  uint8_t pind = PIND;

  switch (estacionIndex) {
    case 0:  // Estación A
      return ((!(pina & (1 << SENSOR_A_ID2))) ? 0b0100 : 0) | ((!(pina & (1 << SENSOR_A_ID1))) ? 0b0010 : 0) | ((!(pina & (1 << SENSOR_A_ID0))) ? 0b0001 : 0) | ((!(pind & (1 << SENSOR_A_ID3))) ? 0b1000 : 0);
    case 1:  // Estación B
      return ((!(pina & (1 << SENSOR_B_ID2))) ? 0b0100 : 0) | ((!(pina & (1 << SENSOR_B_ID1))) ? 0b0010 : 0) | ((!(pina & (1 << SENSOR_B_ID0))) ? 0b0001 : 0) | ((!(pind & (1 << SENSOR_B_ID3))) ? 0b1000 : 0);
    case 2:  // Estación C
      return ((!(pinc & (1 << SENSOR_C_ID2))) ? 0b0100 : 0) | ((!(pinc & (1 << SENSOR_C_ID1))) ? 0b0010 : 0) | ((!(pinc & (1 << SENSOR_C_ID0))) ? 0b0001 : 0) | ((!(pind & (1 << SENSOR_C_ID3))) ? 0b1000 : 0);
    case 3:  // Estación D
      return ((!(pinc & (1 << SENSOR_D_ID2))) ? 0b0100 : 0) | ((!(pinc & (1 << SENSOR_D_ID1))) ? 0b0010 : 0) | ((!(pinc & (1 << SENSOR_D_ID0))) ? 0b0001 : 0) | ((!(pind & (1 << SENSOR_D_ID3))) ? 0b1000 : 0);
  }
  return 0;
}

bool leerSensorInplace(int estacionIndex) {
  switch (estacionIndex) {
    case 0: return (PINA & (1 << SENSOR_A_INPLACE)) == 0;
    case 1: return (PINA & (1 << SENSOR_B_INPLACE)) == 0;
    case 2: return (PINC & (1 << SENSOR_C_INPLACE)) == 0;
    case 3: return (PINC & (1 << SENSOR_D_INPLACE)) == 0;
  }
  return false;
}

void controlarActuador(int estacionIndex, bool activar) {
  uint8_t pinActuador;
  switch (estacionIndex) {
    case 0: pinActuador = ACTUADOR_A_PIN; break;
    case 1: pinActuador = ACTUADOR_B_PIN; break;
    case 2: pinActuador = ACTUADOR_C_PIN; break;
    case 3: pinActuador = ACTUADOR_D_PIN; break;
    default: return;
  }

  if (activar) {
    PORTF &= ~(1 << pinActuador);
  } else {
    PORTF |= (1 << pinActuador);
  }
}

void imprimirID(int estacionIndex, uint8_t id) {
  Serial.print("Estacion ");
  Serial.print(NOMBRES_ESTACION[estacionIndex]);
  Serial.print(". ID leido: ");
  for (int8_t b = 3; b >= 0; b--) {
    Serial.print((id >> b) & 1);
  }
  Serial.print(" (Decimal: ");
  Serial.print(id);
  Serial.println(")");
}























# Conclusiones
	La implementación del Arduino Mega como reemplazo del PLC en el sistema de control de la cinta transportadora demostró ser una solución eficiente y versátil. A través del diseño de esquemas eléctricos, la incorporación de componentes óptimos como optoacopladores y sensores magnéticos, y el desarrollo de un código funcional para la comunicación y control, se logró cumplir con los objetivos del proyecto: modernizar el sistema y optimizar su funcionamiento.
	Este enfoque no solo permitió la automatización precisa de las cuatro estaciones de trabajo, sino que también facilitó una interacción efectiva entre el Arduino y la PC mediante comunicación USB - RS232. Las pruebas realizadas garantizaron la fiabilidad del sistema, ajustando tiempos y lógica para evitar colisiones entre pallets y mejorar la productividad.












# ANEXO 1
## Placa Optoacopladores – diseño y calculo
La corriente típica del LED del PC817A es de 10 mA, pero puede funcionar correctamente con corrientes más bajas, como 5 mA. de esta forma optimizamos el consumo y temperaturas elevadas en la resistencia de entrada.

Datos conocidos:
Tensión de la fuente: Vfuente= Vin=24V
Tensión directa del led del optoacoplador: Vled=1.2V
Corriente deseada a través del led: iled=5mA=0.005ª

Paso 1: Calculo de la Resistencia de Entrada
Corriente de 5 mA para disminuir la disipación de potencia y el calor generado:

If=5mA=0.005 A
La nueva resistencia se calcula como:
R1=(Vin-Vled)/iled
Sustituimos los valores:
R1=(24V-1.2V)/0.005A
R1=4560Ω
Utilizamos la resistencia estándar de 4.7k Ω

Paso 2: Verificar la Potencia Disipada
La potencia disipada por la resistencia se calcula como:

Pr1=〖iled〗^2  x R1
Donde:
iled=0.005A
R1=4700 Ω
Calculamos:
Pr1=〖(0.005A)〗^2  x 4700Ω
Pr1=0.1175W
Una resistencia de 1/4 de vatio (0.25 W) debería ser suficiente, ya que la potencia disipada es mucho menor que la capacidad de la resistencia.

1. Diodo de protección en la entrada:
Colocamos un diodo en polaridad inversa en la entrada para proteger el optoacoplador en caso de conexión inversa de la fuente de 24V.
2. LED en la salida:
Colocamos un LED con una resistencia en la salida del optoacoplador para visualizar el estado de la señal de 5V.
Esquema del Circuito:
 
<img width="1030" height="670" alt="Imagen29" src="https://github.com/user-attachments/assets/fabfe3ff-78b2-44f4-bc44-59535f2be3f7" />


Lista de Componentes:
	R1: 4.7kΩ (resistor de entrada)
	R2: 2kΩ (resistor de pull-up)
	R3: 330Ω (resistor limitador de corriente para el LED de salida)
	D1: Diodo de protección en polaridad inversa (1N4007 o similar)
	LED (OUT): LED para indicar la salida de 5V
Detalles del Circuito:
	Resistor de entrada (R1):
	Valor: 4.7kΩ
	Función: Limitar la corriente a través del LED del optoacoplador.
	Diodo de protección (D1):
	Tipo: 1N4007 o similar.
	Conexión: En polaridad inversa entre la entrada de 24V y GND.
	Función: Proteger el optoacoplador en caso de conexión inversa de la fuente.
	Optoacoplador (PC817A):
	LED interno: Conectado en serie con R1.
	Transistor interno: Colector a GND a través de D1, emisor a +5V a través de R2.
	Resistor de salida (R2):
	Valor: 2kΩ
	Función: Actuar como un resistor de pull-up para la salida del optoacoplador.
	Resistor limitador de corriente para el LED de salida (R3):
	Valor: 330Ω (calculado para una corriente de aproximadamente 15 mA para el LED).
	Función: Limitar la corriente a través del LED de salida.
	LED de salida (LED OUT):
	Función: Indicar visualmente la presencia de la señal de 5V.

