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
#define SEMAFORO_A_VERDE  PF4

// --- Estación B ---
#define SENSOR_B_ID0      PA4   // Pin 26
#define SENSOR_B_ID1      PA5   // Pin 27
#define SENSOR_B_ID2      PA6   // Pin 28
#define SENSOR_B_ID3      PD1   // Pin 20 (INT1) Arriving
#define SENSOR_B_INPLACE  PA7   // Pin 29
#define ACTUADOR_B_PIN    PF1   // Asignado al Puerto F
#define SEMAFORO_B_VERDE  PF5

// --- Estación C ---
#define SENSOR_C_ID0      PC0   // Pin 37
#define SENSOR_C_ID1      PC1   // Pin 36
#define SENSOR_C_ID2      PC2   // Pin 35
#define SENSOR_C_ID3      PD2   // Pin 19 (INT2) Arriving
#define SENSOR_C_INPLACE  PC3   // Pin 34
#define ACTUADOR_C_PIN    PF2   // Asignado al Puerto F
#define SEMAFORO_C_VERDE  PF6

// --- Estación D ---
#define SENSOR_D_ID0      PC4   // Pin 33
#define SENSOR_D_ID1      PC5   // Pin 32
#define SENSOR_D_ID2      PC6   // Pin 31
#define SENSOR_D_ID3      PD3   // Pin 18 (INT3) Arriving
#define SENSOR_D_INPLACE  PC7   // Pin 30
#define ACTUADOR_D_PIN    PF3   // Asignado al Puerto F
#define SEMAFORO_D_VERDE  PF7

// =================================================================
// 2. PARÁMETROS DE TIEMPO (AJUSTABLES EN MILISEGUNDOS)
// =================================================================
#define TIEMPO_FRENADO_MS 1000  // Tiempo que el actuador permanece activado antes de esperar la señal "inplace".
#define TIEMPO_LECTURA_MS 20    // Pausa adicional después de leer el ID y antes de liberar el pallet.

// =================================================================
// 3. ESTRUCTURAS Y VARIABLES GLOBALES
// =================================================================

#define NUM_ESTACIONES 4
const char NOMBRES_ESTACION[NUM_ESTACIONES] = {'A', 'B', 'C', 'D'};

const int OVERFLOWS_PARA_FRENADO = TIEMPO_FRENADO_MS / 16;

struct EstadoEstacion {
  volatile bool palletDetectado = false;
  volatile bool tiempoDeFrenadoCumplido = false;
  volatile int contadorOverflows = 0;
};

EstadoEstacion estados[NUM_ESTACIONES];

// *** NUEVO: Variables para el control por comandos ***
int palletARetener = -1; // ID del pallet a retener (-1 significa ninguno)
char estacionDeRetencion = ' '; // Estación donde se retendrá el pallet
unsigned long tiempoDeRetencion = 0; // Tiempo en milisegundos para retener el pallet
bool palletRetenido = false; // Indica si un pallet está actualmente retenido

// Prototipos de funciones auxiliares
void gestionarEstacion(int i);
uint8_t leerID(int estacionIndex);
void imprimirID(int estacionIndex, uint8_t id);
bool leerSensorInplace(int estacionIndex);
void controlarActuador(int estacionIndex, bool activar);
void configurarInterrupcionesExternas();
void configurarTimer2();
void procesarComandosSerial(); // *** NUEVO ***
void esperarLiberacion(int estacionIndex, uint8_t id); // *** NUEVO ***

// =================================================================
// 4. CONFIGURACIÓN INICIAL (setup)
// =================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Sistema de estaciones iniciado.");
  Serial.println("Usa el formato: P<ID>,S<LETRA>,T<milisegundos> para retener un pallet.");
  Serial.println("Ejemplo: P5,SC,T30000 (Retener pallet 5 en estación C por 30 segundos)");


  // --- Configuración de Pines de Entrada (Sensores) ---
  DDRA &= ~((1 << SENSOR_A_ID0) | (1 << SENSOR_A_ID1) | (1 << SENSOR_A_ID2) | (1 << SENSOR_A_INPLACE) | (1 << SENSOR_B_ID0) | (1 << SENSOR_B_ID1) | (1 << SENSOR_B_ID2) | (1 << SENSOR_B_INPLACE));
  PORTA |= (1 << SENSOR_A_ID0) | (1 << SENSOR_A_ID1) | (1 << SENSOR_A_ID2) | (1 << SENSOR_A_INPLACE) | (1 << SENSOR_B_ID0) | (1 << SENSOR_B_ID1) | (1 << SENSOR_B_ID2) | (1 << SENSOR_B_INPLACE);

  DDRC &= ~((1 << SENSOR_C_ID0) | (1 << SENSOR_C_ID1) | (1 << SENSOR_C_ID2) | (1 << SENSOR_C_INPLACE) | (1 << SENSOR_D_ID0) | (1 << SENSOR_D_ID1) | (1 << SENSOR_D_ID2) | (1 << SENSOR_D_INPLACE));
  PORTC |= (1 << SENSOR_C_ID0) | (1 << SENSOR_C_ID1) | (1 << SENSOR_C_ID2) | (1 << SENSOR_C_INPLACE) | (1 << SENSOR_D_ID0) | (1 << SENSOR_D_ID1) | (1 << SENSOR_D_ID2) | (1 << SENSOR_D_INPLACE);

  DDRD &= ~((1 << SENSOR_A_ID3) | (1 << SENSOR_B_ID3) | (1 << SENSOR_C_ID3) | (1 << SENSOR_D_ID3));
  PORTD |= (1 << SENSOR_A_ID3) | (1 << SENSOR_B_ID3) | (1 << SENSOR_C_ID3) | (1 << SENSOR_D_ID3);

  // --- Configuración de Pines de Salida (Actuadores) ---
  DDRF |= (1 << ACTUADOR_A_PIN) | (1 << ACTUADOR_B_PIN) | (1 << ACTUADOR_C_PIN) | (1 << ACTUADOR_D_PIN) | (1  << SEMAFORO_A_VERDE) | (1  << SEMAFORO_B_VERDE) | (1  << SEMAFORO_C_VERDE) | (1  << SEMAFORO_D_VERDE);
  PORTF |= (1 << ACTUADOR_A_PIN) | (1 << ACTUADOR_B_PIN) | (1 << ACTUADOR_C_PIN) | (1 << ACTUADOR_D_PIN) | (1  << SEMAFORO_A_VERDE) | (1  << SEMAFORO_B_VERDE) | (1  << SEMAFORO_C_VERDE) | (1  << SEMAFORO_D_VERDE);


  // --- Configuración de Interrupciones ---
  configurarInterrupcionesExternas();
  configurarTimer2();

  sei();
}

// =================================================================
// 5. BUCLE PRINCIPAL (loop)
// =================================================================
void loop() {
  procesarComandosSerial(); // Revisa si hay nuevos comandos
  for (int i = 0; i < NUM_ESTACIONES; i++) {
    gestionarEstacion(i);
  }
}

// =================================================================
// 6. LÓGICA PRINCIPAL POR ESTACIÓN
// =================================================================
void gestionarEstacion(int i) {
  if (!estados[i].palletDetectado) {
    return;
  }

  if (estados[i].tiempoDeFrenadoCumplido == false) {
    controlarActuador(i, true);
  } else {
    if (leerSensorInplace(i)) {
      uint8_t id = leerID(i);
      imprimirID(i, id);

      // *** NUEVO: Lógica de retención ***
      if (id == palletARetener && NOMBRES_ESTACION[i] == estacionDeRetencion && !palletRetenido) {
        palletRetenido = true;
        esperarLiberacion(i, id);
      } else {
        delay(TIEMPO_LECTURA_MS);
        controlarActuador(i, false);
      }

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

// *** NUEVA FUNCIÓN ***
void procesarComandosSerial() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    
    int p_index = comando.indexOf('P');
    int s_index = comando.indexOf('S');
    int t_index = comando.indexOf('T');

    if (p_index != -1 && s_index != -1 && t_index != -1) {
      palletARetener = comando.substring(p_index + 1, s_index).toInt();
      estacionDeRetencion = comando.charAt(s_index + 1);
      tiempoDeRetencion = comando.substring(t_index + 1).toInt();
      
      Serial.print("Comando recibido: Retener pallet ID ");
      Serial.print(palletARetener);
      Serial.print(" en la estacion ");
      Serial.print(estacionDeRetencion);
      Serial.print(" por ");
      Serial.print(tiempoDeRetencion);
      Serial.println(" ms.");
      palletRetenido = false; // Resetea el estado de retención
    } else {
      Serial.println("Comando no valido. Formato: P<ID>,S<LETRA>,T<milisegundos>");
    }
  }
}

// *** NUEVA FUNCIÓN ***
void esperarLiberacion(int estacionIndex, uint8_t id) {
  Serial.print("Pallet ID ");
  Serial.print(id);
  Serial.print(" retenido en la estacion ");
  Serial.print(NOMBRES_ESTACION[estacionIndex]);
  Serial.print(" por ");
  Serial.print(tiempoDeRetencion);
  Serial.println(" ms.");

  delay(tiempoDeRetencion); // Espera el tiempo especificado

  Serial.print("Tiempo de retencion finalizado. Liberando pallet ID ");
  Serial.println(id);
  
  controlarActuador(estacionIndex, false);
  
  // Reinicia las variables de control
  palletARetener = -1;
  estacionDeRetencion = ' ';
  tiempoDeRetencion = 0;
  palletRetenido = false;
}

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
  uint8_t pinLuz;

  switch (estacionIndex) {
    case 0:
      pinActuador = ACTUADOR_A_PIN;
      pinLuz = SEMAFORO_A_VERDE;
      break;
    case 1:
      pinActuador = ACTUADOR_B_PIN;
      pinLuz = SEMAFORO_B_VERDE;
      break;
    case 2:
      pinActuador = ACTUADOR_C_PIN;
      pinLuz = SEMAFORO_C_VERDE;
      break;
    case 3:
      pinActuador = ACTUADOR_D_PIN;
      pinLuz = SEMAFORO_D_VERDE;
      break;
    default:
      return;
  }

  if (activar) {
    PORTF &= ~((1 << pinActuador) | (1 << pinLuz));
  } else {
    PORTF |= ((1 << pinActuador) | (1 << pinLuz));
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
