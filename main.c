#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// A5/SCL = PC5
// A4/SDA = PC4
// A1/Temp = PC1
// A0/Gas = PC0
// D2/Button1 = PD2
// D3/Button2 = PD3
// D9/Servo = PB1 (OC1A)
// D13/LED = PB5

// I2C LCD address
#define LCD_ADDR 0x27   
// Menu states
#define MAIN_MENU 0
#define SENSOR_READINGS 1
#define SET_THRESHOLDS 2
#define STATUS_DISPLAY 3

// LCD commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// LCD flags
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01

#define LCD_RS_BIT 0x01        // P0
#define LCD_RW_BIT 0x02        // P1
#define LCD_E_BIT 0x04         // P2
#define LCD_BL_BIT 0x08        // P3

// Variabile globale
volatile uint32_t milliseconds = 0;  // Contor global pentru milisecunde

uint8_t menuState = MAIN_MENU;
int tempThreshold = 35;    // Default threshold (Celsius)
int gasThreshold = 900;    // Default threshold (sensor value)
uint8_t currentMenuOption = 0;
uint8_t editingValue = 0;
uint8_t alarmActive = 0;
uint8_t servoPosition = 0;
uint8_t _backlightState = LCD_BL_BIT; // Backlight on by default

// Button debounce variables
uint16_t debounceDelay = 50; // ms

// Function prototypes
void uart_init(void);
void uart_transmit(uint8_t data);
void uart_print_string(const char* str);
void uart_print_int(int value);
void uart_print_float(float value, int decimal_places);
void delay_ms(uint16_t ms);
void delay_us(uint16_t us);
void timer0_init(void);
uint32_t millis(void);
void twi_init(void);
void twi_wait(void);
uint8_t twi_start(void);
void twi_stop(void);
uint8_t twi_write(uint8_t data);
uint8_t pcf8574_write(uint8_t data);
void lcd_write_4bits(uint8_t data, uint8_t control);
void lcd_send_command(uint8_t command);
void lcd_write_char(uint8_t data);
void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* str);
void lcd_print_int(int value);
void lcd_print_float(float value, int decimal_places);
void lcd_backlight(uint8_t on);
void timer1_pwm_init(void);
void servo_write(uint8_t angle);
void setup_ports(void);
void adc_init(void);
uint16_t adc_read(uint8_t channel);
void scan_i2c_devices(void);
void handleButton1(void);
void handleButton2(void);
void checkButtons(void);
void displayMainMenu(void);
void displaySensorReadings(float temp, int gas);
void displayThresholdSettings(void);
void displayStatus(void);
float readTemperature(void);
uint16_t readGasLevel(void);
void checkThresholds(float temp, int gas);
void updateDisplay(float temperature, int gasLevel);

// Utility functions
int max_val(int a, int b) {
  return (a > b) ? a : b;
}

int min_val(int a, int b) {
  return (a < b) ? a : b;
}

float max_float(float a, float b) {
  return (a > b) ? a : b;
}

// UART initialization with baudrate 9600
void uart_init(void) {
  #define F_CPU 16000000UL  // Frecvența procesorului (16MHz pentru Arduino standard)
  #define BAUD 9600
  #define UBRR_VALUE ((F_CPU / 16 / BAUD) - 1)
  
  // Set baud rate
  UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
  UBRR0L = (uint8_t)UBRR_VALUE;
  
  // Enable transmitter
  UCSR0B = (1 << TXEN0);
  
  // Set frame format: 8 data bits, 1 stop bit, no parity
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Transmit single byte via UART
void uart_transmit(uint8_t data) {
  // Wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)));
  
  // Put data into buffer
  UDR0 = data;
}

// Print string via UART
void uart_print_string(const char* str) {
  while (*str) {
    uart_transmit(*str++);
  }
}

// Print integer via UART
void uart_print_int(int value) {
  char buffer[16];
  sprintf(buffer, "%d", value);
  uart_print_string(buffer);
}

// Print float via UART
void uart_print_float(float value, int decimal_places) {
  int int_part = (int)value;
  int decimal_part = abs((int)((value - int_part) * pow(10, decimal_places)));
  
  uart_print_int(int_part);
  uart_transmit('.');
  uart_print_int(decimal_part);
}

// Initialize timer for millisecond tracking
void timer0_init(void) {
  // Configurare Timer0 pentru întrerupere la fiecare 1ms
  // Mod CTC (Clear Timer on Compare Match)
  TCCR0A = (1 << WGM01);
  
  // Prescaler 64
  TCCR0B = (1 << CS01) | (1 << CS00);
  
  // Valoare pentru compare match (pentru 1ms la 16MHz cu prescaler 64)
  // 16000000 / 64 / 1000 = 250
  OCR0A = 249;
  
  // Activare intrerupere la compare match A
  TIMSK0 = (1 << OCIE0A);
}

// Timer0 compare match ISR
ISR(TIMER0_COMPA_vect) {
  milliseconds++;
}

// Get current milliseconds
uint32_t millis(void) {
  uint32_t ms;
  
  // Disable interrupts to ensure atomic read
  cli();
  ms = milliseconds;
  sei();
  
  return ms;
}

// Delay for the specified number of milliseconds
void delay_ms(uint16_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms);
}

// Delay for specified microseconds
void delay_us(uint16_t us) {
  // La 16 MHz, fiecare ciclu durează 0.0625 microsecunde
  // Așadar, 16 cicluri = 1 microsecundă
  
  // Factorul 4 este aproximativ pentru overhead-ul de apelare
  us *= 4;
  
  // Folosim asamblare pentru cicluri precise
  __asm__ __volatile__ (
    "1: sbiw %0,1" "\n\t"
    "brne 1b"
    : "=w" (us)
    : "0" (us)
  );
}

void twi_init(void) {
  // Seteaza frecvența TWI la ~100kHz pentru un clock de 16MHz
  // Formula: SCL frequency = CPU Clock / (16 + 2 * TWBR * prescaler)
  
  TWBR = 72;  // Valoare pentru ~100kHz la 16MHz
  TWSR = 0;   // Prescaler 1

  TWCR = (1 << TWEN);
  
  PORTC |= (1 << PC4) | (1 << PC5);
}

// Așteaptă finalizarea operațiunii TWI
void twi_wait(void) {
  // Așteaptă până când flag-ul TWINT este setat
  while (!(TWCR & (1 << TWINT)));
}

// Trimite o condiție de START pe bus
uint8_t twi_start(void) {
  // Trimite START
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  twi_wait();
  
  // Verifică dacă START a fost trimis cu succes (status code 0x08)
  if ((TWSR & 0xF8) != 0x08) {
    return 0;  // Eroare la trimiterea START
  }
  
  return 1;  // START trimis cu succes
}

// Trimite o condiție de STOP pe bus
void twi_stop(void) {
  // Trimite STOP
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
  
  // Așteaptă până când STOP este procesat
  // Pentru STOP nu se setează TWINT, așa că așteptăm să se șteargă bitul TWSTO
  while (TWCR & (1 << TWSTO));
}

// Trimite un byte pe bus și așteaptă ACK/NACK
uint8_t twi_write(uint8_t data) {
  // Încarcă data în registrul TWDR
  TWDR = data;
  
  // Trimite data
  TWCR = (1 << TWINT) | (1 << TWEN);
  twi_wait();
  
  // Verifică dacă byte-ul a fost primit și ACK-ul a fost returnat (status code 0x28)
  if ((TWSR & 0xF8) != 0x28 && (TWSR & 0xF8) != 0x18) {
    return 0;  // Nu s-a primit ACK
  }
  
  return 1;  // S-a primit ACK
}


uint8_t pcf8574_write(uint8_t data) {
  uint8_t retry_count = 3;
  uint8_t status_code;
  
  while (retry_count > 0) {
    // Trimite START
    if (!twi_start()) {
      uart_print_string("Eroare la TWI START\r\n");
      twi_stop();
      retry_count--;
      delay_ms(5);
      continue;
    }
    
    // Trimite adresa slave + Write bit (0)
    TWDR = (LCD_ADDR << 1) | 0;  // Adresă slave + Write bit
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();
    
    // Verifică dacă SLA+W a fost trimis și ACK primit
    status_code = (TWSR & 0xF8);
    if (status_code != 0x18) {
      if (status_code == 0x20) {
        uart_print_string("PCF8574: NACK la adresa\r\n");
      } else {
        uart_print_string("PCF8574: Eroare la adresa: 0x");
        // Afișare cod de eroare hex
        char hex_str[5];
        sprintf(hex_str, "%02X", status_code);
        uart_print_string(hex_str);
        uart_print_string("\r\n");
      }
      twi_stop();
      retry_count--;
      delay_ms(10);  // Așteptăm mai mult între încercări
      continue;
    }
    
    // Trimite data
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();
    
    // Verifică dacă data a fost trimisă și ACK primit
    status_code = (TWSR & 0xF8);
    if (status_code != 0x28) {
      uart_print_string("PCF8574: Eroare la transmitere date: 0x");
      char hex_str[5];
      sprintf(hex_str, "%02X", status_code);
      uart_print_string(hex_str);
      uart_print_string("\r\n");
      
      twi_stop();
      retry_count--;
      delay_ms(10);
      continue;
    }
    
    // Trimite STOP
    twi_stop();
    
    // Succes, ieșim din buclă
    return 1;
  }
  
  uart_print_string("PCF8574: Comunicatie esuata dupa 3 incercari\r\n");
  return 0;  // Eșec după toate încercările
}

void lcd_write_4bits(uint8_t data, uint8_t control) {
  // Combinăm data (D4-D7) cu biții de control (RS, RW, E, BL)
  uint8_t value = (data & 0xF0) | control;
  
  // Setăm enable, trimitem, resetăm enable
  pcf8574_write(value | LCD_E_BIT);  // Enable HIGH
  delay_us(1);                       // Puls enable minim (timpul de setare)
  pcf8574_write(value);              // Enable LOW
  delay_us(100);                     // Delay pentru procesare (mai lung pentru siguranță)
}

void lcd_send_command(uint8_t command) {
  // Comandă: RS=0, RW=0, BL=starea backlightului
  uint8_t control = _backlightState; // Folosim doar backlight, RS=0, RW=0
  
  // Trimitem high nibble (4 biți superiori)
  lcd_write_4bits(command & 0xF0, control);
  delay_us(50);  // Timp de așteptare între nibbli
  
  // Trimitem low nibble (4 biți inferiori)
  lcd_write_4bits((command << 4) & 0xF0, control);
  
  // Mai mult timp pentru comenzi care necesită mai mult timp
  if (command == LCD_CLEARDISPLAY || command == LCD_RETURNHOME) {
    delay_ms(2);
  } else {
    delay_us(50);  // Timp minim pentru restul comenzilor
  }
}

void lcd_write_char(uint8_t data) {
  // Date: RS=1, RW=0, BL=starea backlightului
  uint8_t control = LCD_RS_BIT | _backlightState; // RS=1 pentru date
  
  // Trimitem high nibble (4 biți superiori)
  lcd_write_4bits(data & 0xF0, control);
  delay_us(50);  // Timp de așteptare între nibbli
  
  // Trimitem low nibble (4 biți inferiori)
  lcd_write_4bits((data << 4) & 0xF0, control);
  
  delay_us(100);  // Timp de așteptare pentru procesarea caracterului
}

void lcd_init(void) {
  uart_print_string("Inițializare LCD\r\n");
  
  // Inițializare TWI hardware
  twi_init();
  delay_ms(100);
  
  // Verificare prezență dispozitiv
  if (!twi_start()) {
    uart_print_string("Eroare la TWI START în timpul inițializării\r\n");
    return;
  }
  
  TWDR = (LCD_ADDR << 1) | 0;  // Adresă + Write bit
  TWCR = (1 << TWINT) | (1 << TWEN);
  twi_wait();
  
  if ((TWSR & 0xF8) != 0x18) {
    uart_print_string("ERROR: PCF8574 nu a fost găsit la adresa specificată!\r\n");
    twi_stop();
    return;
  }
  
  twi_stop();
  
  _backlightState = LCD_BL_BIT;
  pcf8574_write(_backlightState);
  delay_ms(100);
  

  lcd_write_4bits(0x30, _backlightState);
  delay_ms(5); // Trebuie minim 4.1ms
  
  lcd_write_4bits(0x30, _backlightState);
  delay_ms(5);
  
  lcd_write_4bits(0x30, _backlightState);
  delay_ms(5);
  
  lcd_write_4bits(0x20, _backlightState); // Setează modul 4-bit
  delay_ms(5);
  
  lcd_send_command(LCD_FUNCTIONSET | 0x08);
  delay_ms(5);
  
  lcd_send_command(LCD_DISPLAYCONTROL | LCD_DISPLAYON);
  delay_ms(5);
  
  lcd_send_command(LCD_CLEARDISPLAY);
  delay_ms(5);
  
  lcd_send_command(LCD_ENTRYMODESET | LCD_ENTRYLEFT);
  delay_ms(5);
  
  uart_print_string("Inițializare LCD completă!\r\n");
}

void lcd_clear(void) {
  lcd_send_command(LCD_CLEARDISPLAY);
  delay_ms(2);
}

void lcd_home(void) {
  lcd_send_command(LCD_RETURNHOME);
  delay_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
  const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  lcd_send_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_print(const char* str) {
  while (*str) {
    lcd_write_char(*str++);
  }
}

void lcd_print_int(int value) {
  char buffer[16];
  sprintf(buffer, "%d", value);
  lcd_print(buffer);
}

void lcd_print_float(float value, int decimal_places) {
  char buffer[16];
  int int_part = (int)value;
  int decimal_part = abs((int)((value - int_part) * pow(10, decimal_places)));
  sprintf(buffer, "%d.%d", int_part, decimal_part);
  lcd_print(buffer);
}

void lcd_backlight(uint8_t on) {
  _backlightState = on ? LCD_BL_BIT : 0;
  pcf8574_write(_backlightState);
}

// ADC initialization
void adc_init(void) {
  ADMUX = (1 << REFS0);
  
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  (void)ADC; // Ignoră rezultatul
  
  delay_ms(1); // Timp de stabilizare suplimentar
}

// Read ADC value from specified channel (0-7)
uint16_t adc_read(uint8_t channel) {
  uint16_t result = 0;
  
  // Selectare canal (0-7) păstrând restul setărilor (referință AVCC)
  ADMUX = (ADMUX & 0xF0) | (channel & 0x07);
  
  delay_us(10); // Timp pentru stabilizarea MUX
  
  // Execută mai multe citiri pentru a avea o valoare mai stabilă
  for (uint8_t i = 0; i < 4; i++) {
    // Start conversie
    ADCSRA |= (1 << ADSC);
    
    // Așteptare finalizare conversie
    while (ADCSRA & (1 << ADSC));
    
    // Acumulare rezultat
    result += ADC;
  }
  
  // Calcularea mediei (împărțire la 4)
  return result >> 2;
}


void timer1_pwm_init(void) {
  // Configurăm pinul servo ca ieșire
  DDRB |= (1 << PB1);  // PB1 = Digital Pin 9
  
  TCCR1A = 0;
  TCCR1B = 0;
  
  TCCR1B = (1 << CS11);
  
  TCNT1 = 0;
}

void servo_write(uint8_t angle) {
  if (angle > 180) angle = 180;
  
  servoPosition = angle;
  
  
  uint16_t pulseWidth = 1000 + (angle * 1000 / 180);
  
  for (uint8_t i = 0; i < 5; i++) {
    PORTB |= (1 << PB1);
    
    delay_us(pulseWidth);
    
    PORTB &= ~(1 << PB1);
    
    
    delay_us(20000 - pulseWidth);
  }
}

void setup_ports(void) {
  
  DDRB |= (1 << PB5) | (1 << PB1);
  
 
  PORTB &= ~(1 << PB5);
  
 
  DDRC |= (1 << PC5) | (1 << PC4);    // SCL/SDA outputs
  DDRC &= ~((1 << PC1) | (1 << PC0)); // Senzori inputs
  
  PORTC |= (1 << PC5) | (1 << PC4);
  
  
  DDRD &= ~((1 << PD2) | (1 << PD3)); // Butoane inputs
  PORTD |= (1 << PD2) | (1 << PD3);   // Activare pull-ups pentru butoane
  
  uart_print_string("Stare initiala butoane: ");
  uart_print_int(PIND & (1 << PD2) ? 1 : 0);
  uart_transmit(',');
  uart_print_int(PIND & (1 << PD3) ? 1 : 0);
  uart_print_string("\r\n");
}

void checkButtons(void) {
  uint8_t button1State = (PIND & (1 << PD2)) ? 1 : 0;  // PD2 = D2
  uint8_t button2State = (PIND & (1 << PD3)) ? 1 : 0;  // PD3 = D3
  
  static uint8_t button1StateStable = 1;  // Stare stabilă buton 1 (1 = inactiv, 0 = apăsat)
  static uint8_t button2StateStable = 1;  // Stare stabilă buton 2 (1 = inactiv, 0 = apăsat)
  static uint32_t lastTime = 0;           // Timpul ultimei schimbări detectate
  
  uint32_t currentTime = millis();
  
  if (currentTime - lastTime >= debounceDelay) {
    // Verificare dacă butonul 1 a fost apăsat (tranziție HIGH->LOW)
    if (button1State == 0 && button1StateStable == 1) {
      button1StateStable = 0;
      lastTime = currentTime;
      
      // Apelare funcție de tratare
      handleButton1();
      uart_print_string("Buton 1 apasat\r\n");
    }
    // Verificare dacă butonul 1 a fost eliberat (tranziție LOW->HIGH)
    else if (button1State == 1 && button1StateStable == 0) {
      button1StateStable = 1;
      lastTime = currentTime;
    }
    
    // Verificare dacă butonul 2 a fost apăsat (tranziție HIGH->LOW)
    if (button2State == 0 && button2StateStable == 1) {
      button2StateStable = 0;
      lastTime = currentTime;
      
      // Apelare funcție de tratare
      handleButton2();
      uart_print_string("Buton 2 apasat\r\n");
    }
    // Verificare dacă butonul 2 a fost eliberat (tranziție LOW->HIGH)
    else if (button2State == 1 && button2StateStable == 0) {
      button2StateStable = 1;
      lastTime = currentTime;
    }
  }
}
void displayStatus(void) {
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print("Alarm: ");
  
  // Afișăm starea alarmei (ON/OFF)
  if (alarmActive) {
    lcd_print("ON");
  } else {
    lcd_print("OFF");
  }
  
  lcd_set_cursor(0, 1);
  lcd_print("Vent: ");
  
  int ventPercent = (servoPosition * 100) / 45;
  lcd_print_int(ventPercent);
  lcd_print("%");
}
void handleButton1(void) {
  if (!editingValue) {
    currentMenuOption = (currentMenuOption + 1) % 3;
  } else {
    if (menuState == SET_THRESHOLDS) {
      if (currentMenuOption == 0) {
        tempThreshold++;
        if (tempThreshold > 50) tempThreshold = 50;
      } else {
        gasThreshold += 10;
        if (gasThreshold > 1000) gasThreshold = 1000;
      }
    }
  }
}

void handleButton2(void) {
  if (menuState == MAIN_MENU) {
    if (currentMenuOption == 0) {
      menuState = SENSOR_READINGS;
    } else if (currentMenuOption == 1) {
      menuState = SET_THRESHOLDS;
      currentMenuOption = 0;
    } else if (currentMenuOption == 2) {
      menuState = STATUS_DISPLAY;
    }
  } else if (menuState == SET_THRESHOLDS) {
    if (editingValue) {
      editingValue = 0;
      currentMenuOption = (currentMenuOption + 1) % 2;
    } else {
      editingValue = 1;
    }
  } else {
    menuState = MAIN_MENU;
    currentMenuOption = 0;
  }
}

void updateDisplay(float temperature, int gasLevel) {
  switch (menuState) {
    case MAIN_MENU:
      displayMainMenu();
      break;
    case SENSOR_READINGS:
      displaySensorReadings(temperature, gasLevel);
      break;
    case SET_THRESHOLDS:
      displayThresholdSettings();
      break;
    case STATUS_DISPLAY:
      displayStatus();
      break;
  }
}

void displayMainMenu(void) {
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print("Main Menu");
  lcd_set_cursor(0, 1);
  
  switch (currentMenuOption) {
    case 0:
      lcd_print("> Sensor Readings");
      break;
    case 1:
      lcd_print("> Set Thresholds");
      break;
    case 2:
      lcd_print("> Status");
      break;
  }
}

void displaySensorReadings(float temp, int gas) {
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print("Temp: ");
  lcd_print_float(temp, 1);
  lcd_print("C");
  lcd_set_cursor(0, 1);
  lcd_print("Gas: ");
  lcd_print_int(gas);
}

void displayThresholdSettings(void) {
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print("Thresholds:");
  
  if (currentMenuOption == 0) {
    lcd_set_cursor(0, 1);
    lcd_print("Temp: ");
    lcd_print_int(tempThreshold);
    lcd_print("C");
    if (editingValue) {
      lcd_print(" *");
    }
  } else {
    lcd_set_cursor(0, 1);
    lcd_print("Gas: ");
    lcd_print_int(gasThreshold);
    if (editingValue) {
      lcd_print(" *");
    }
  }
}

// Sensor reading functions
float readTemperature(void) {
  uint16_t sensorValue = adc_read(1); // Canal A1 / PC1
  float voltage = sensorValue * (5.0 / 1023.0);
  float temperatureC = (voltage - 0.5) * 100.0;
  return temperatureC;
}

uint16_t readGasLevel(void) {
  return adc_read(0); // Canal A0 / PC0
}

void checkThresholds(float temp, int gas) {
  int shouldTriggerAlarm = (temp >= tempThreshold) || (gas >= gasThreshold);
  
  if (shouldTriggerAlarm) {
    if (!alarmActive) {
      alarmActive = 1;
      PORTB |= (1 << PB5); // Set LED pin HIGH (D13/PB5)
    }
    
    float tempExcess = max_float(0.0, temp - tempThreshold);
    int gasExcess = max_val(0, gas - gasThreshold) / 10;
    float excess = max_float(tempExcess * 2.0, (float)gasExcess);
    
    int servoPos = min_val(45, (int)(excess * 5));
    servo_write(servoPos);
    
  } else if (alarmActive) {
    // Deactivate alarm
    alarmActive = 0;
    PORTB &= ~(1 << PB5); // Set LED pin LOW (D13/PB5)
    servo_write(0);
  }
}

void scan_i2c_devices(void) {
  uint8_t device_count = 0;
  
  uart_print_string("Scanare dispozitive I2C:\r\n");
  
  for (uint8_t addr = 1; addr < 127; addr++) {
    // Trimite START
    if (!twi_start()) {
      uart_print_string("Eroare la TWI START in timpul scanarii\r\n");
      continue;
    }
    
    // Trimite adresa slave + Write bit (0)
    TWDR = (addr << 1) | 0;
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();
    
    // Verifică dacă SLA+W a fost trimis și ACK primit (status code 0x18)
    if ((TWSR & 0xF8) == 0x18) {
      device_count++;
      
      uart_print_string("Dispozitiv gasit la adresa: 0x");
      char hex_str[5];
      sprintf(hex_str, "%02X", addr);
      uart_print_string(hex_str);
      uart_print_string("\r\n");
    }
    
    // Trimite STOP
    twi_stop();
    delay_ms(5);
  }
  
  uart_print_string("Scanare completa. Dispozitive gasite: ");
  uart_print_int(device_count);
  uart_print_string("\r\n");
}

int main(void) {
  uart_init();
  uart_print_string("\r\nInitializare sistem control sera...\r\n");
  
  cli(); 
  setup_ports();
  
  timer0_init();
  
  adc_init();
  
  timer1_pwm_init();
  
  sei(); 
  
  uart_print_string("Hardware initializat.\r\n");
  
  uart_print_string("Test comunicatie seriala: OK\r\n");
  
  servo_write(0);
  uart_print_string("Servo initializat la pozitia 0\r\n");
  
  uint16_t testAdc = adc_read(0);
  uart_print_string("Test ADC canal 0: ");
  uart_print_int(testAdc);
  uart_print_string("\r\n");
  
  twi_init();
  uart_print_string("TWI initializat\r\n");
  
  scan_i2c_devices();
  
  uart_print_string("Incepere initializare LCD...\r\n");
  
  lcd_init();
  
  lcd_clear();
  uart_print_string("LCD clear trimis\r\n");
  
  lcd_set_cursor(0, 0);
  lcd_print("Greenhouse");
  
  lcd_set_cursor(0, 1);
  lcd_print("Control System");
  uart_print_string("Mesaj de bun-venit afisat\r\n");
  
  delay_ms(2000);
  
  lcd_backlight(0);
  uart_print_string("Backlight OFF\r\n");
  delay_ms(500);
  
  lcd_backlight(1);
  uart_print_string("Backlight ON\r\n");
  delay_ms(500);
  
  displayMainMenu();
  
  uart_print_string("Initializare finalizata!\r\n");
  
  uint32_t lastDisplayUpdate = 0;
  uint32_t displayUpdateInterval = 500; // ms
  
  while (1) {
    // Verificare butoane la fiecare iterație
    checkButtons();
    
    uint32_t currentTime = millis();
    if (currentTime - lastDisplayUpdate >= displayUpdateInterval) {
      float temperature = readTemperature();
      uint16_t gasLevel = readGasLevel();
      
      uart_print_string("Temperature: ");
      uart_print_float(temperature, 1);
      uart_print_string("C, Gas: ");
      uart_print_int(gasLevel);
      uart_print_string("\r\n");
      
      checkThresholds(temperature, gasLevel);
      updateDisplay(temperature, gasLevel);
      
      lastDisplayUpdate = currentTime;
    }
    
    // O scurtă pauză pentru a preveni citiri prea rapide
    delay_ms(10);
  }
  
  return 0; // Nu se va ajunge niciodată aici
}