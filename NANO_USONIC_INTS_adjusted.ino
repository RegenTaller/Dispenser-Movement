#include <SimpleKalmanFilter.h>



#define trig 3  //6 на меге
#define echo 2  //3 на меге

#define STEP 5
#define DIR 4

//PWM 5,6,9,11

#define CS 10
#define MOSI 11
#define MISO 12
#define SCK 13

#define EN 6

#include "TMC2130Stepper.h"
TMC2130Stepper driver = TMC2130Stepper(EN, DIR, STEP, CS);

//TMC2130Stepper TMC2130 = TMC2130Stepper();

const int N = 159;  //N - число для сброса таймера. 159+1 = 160. 160*62.5 = 10000 нс = 10 мкс.

float MM_na_oborot = 31.141592;         //Миллиметров будет пройдено за один оборот
int microsteps = 16;

float MM_na_Shag = MM_na_oborot / (200*microsteps);  //Миллиметров будет пройдено за один шаг
float speedMMs = 20;                 //Скорость в мм/с

float Freq_MOT = speedMMs / MM_na_Shag;  //шагов в секунду при данной скорости

float period_MOT = 1000000.0 / Freq_MOT;  //Микросекунд на шаг при данной скорости

float MOT_ISR_Tact = (period_MOT / ((N + 1) * 0.0625)) / 2;  //Сколько тактов нужно для данной конфигурации таймера 519 мкс для 150 мм/с, значит 51,9 тактов для N = 159

volatile int MOT_ISR_N = round(MOT_ISR_Tact); //ОКРУГЛЁННОЕ ЗНАЧЕНИЕ ШАГОВ ДЛЯ ЗАПИСИ В РЕГИСТР

//volatile int MOT_counter = 0; //СЧЁТЧИК ДЛЯ ОТСЛЕЖИВАНИЯ СМЕНЫ СИГНАЛА С 1 на 0

//volatile bool flagMove = 1;




bool en = 1; //ОТКЛЮЧЕНИЕ ЛОГИКИ ДРАЙВЕРА


volatile uint16_t counterSteps = 0;  //Реальное количество сделанных шагов

int direction = 0;      //Направление
long controlPos = 0;     //Необходимая позиция в мм. Пропорциональна количеству шагов
long controlStepPos = 0; //Необходимая позиция в шагах, зависящая от позиции в миллиметрах

long MAXSteps = 8899;//ОГРАНИЧИТЕЛЬ ШАГОВ
int MINSteps = 0; //НИЖНИЙ ОГРАНИЧИТЕЛЬ ШАГОВ
int delta = 0; //ОТСТУП ДЛЯ КАЛИБРОВКИ


volatile int32_t countrising = 0, counterfalling = 0;


const float timicros = 0.0625;                           //Время такта в микросекундах
const float speed = 0.343;                               //Скорость звука в мм/мкс
const float coefficient = 8.00000000 * speed * timicros / 2.0000000;  //коеффициент умножения для получения расстояния

volatile int32_t count = 0, count2 = 0;
uint16_t cnt_ovf = 0;


uint32_t t1 = 0, t2 = 0;

bool flag;

float dist = 0;

volatile uint16_t counternew = 0;

const uint16_t per = 999;  //Период подачи сигнала на TRIG датчика

SimpleKalmanFilter Filter(2, 1, 0.03);  //1-ый коэффициент - амплитуда разлёта показаний от реального
float filtered = 0;
int NULLFLAG = 0;

void setup() {

  Serial.begin(115200);
  if (NULLFLAG == 0) {

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT_PULLUP);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);

  //Serial.begin(9600);
	while(!Serial);
	Serial.println("Start...");
	driver.begin(); 			// Initiate pins and registeries
	driver.rms_current(500); 	// Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
	driver.stealthChop(1); 	// Enable extremely quiet stepping
  driver.microsteps(microsteps);
	
	digitalWrite(EN, LOW);

	Serial.print("DRV_STATUS=0b");
	Serial.println(driver.DRV_STATUS(), BIN);

  Serial.println(MOT_ISR_N);

  //Таймер ECHO для установления длительности сигнала.
  TCCR1A = TCCR1B = TCNT1 = cnt_ovf = 0;  //5 таймер на подсчёте прерываний
  TIFR1 = (1 << TOV1);                    //обработка прерывания
  TIMSK1 = (1 << TOIE1);                  //Включение прерывания по переполнению
  TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10);                   //делитель тактирования 1 16 МГЦ

  // EIMSK &= ~(1 << INT0);                 //External Interrupt Mask Register - EIMSK - is for enabling INT[6;3:0] interrupts, INT0 is disabled to avoid false interrupts when mainuplating EICRA
  // EICRA |= (1 << ISC01) | (1 << ISC00);  //External Interrupt Control Register A - EICRA - defines the interrupt edge profile, here configured to trigger on rising edge
  // EIFR &= ~(1 << INTF0);                 //External Interrupt Flag Register - EIFR controls interrupt flags on INT[6;3:0], here it is cleared
  // EIMSK |= (1 << INT0);                  //Enable INT0
  //Enable global interrupts

  ////////////////////////////////////////////////////////////////////////////

  //Таймер TRIG Триггера и Моторов:
  TCCR2A = 0;  // set entire TCCR4A register to 0
  TCCR2B = 0;  // same for TCCR4B
  TCNT2 = 0;   // initialize counter value to 0

  TIFR2 = (1 << TOV2);
  TCCR2A |= (0 << WGM20) | (1 << WGM21);
  // turn on CTC mode
  TCCR2B |= (0 << WGM22);  //страница 145 док. CTC mode 2560 //с 130 для 328P
  // Set  bits for 1 prescaler
  TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  OCR2A = N;  //159+1 = 160; 160*62.5 = 10000 нс = 10 мкс. Вызов функции каждые 10 мкс.

  /////////////////////////////////////////////////////////////////////////////////////////
  //OCR4A = 65535;

  count2 = 0;
  attachInterrupt(digitalPinToInterrupt(echo), AISR_CHANGE, CHANGE);

  sei();

  // Serial.println("Frequency: " + String(Freq_MOT));
  // Serial.println("Period: " + String(period_MOT));
  // Serial.println("Tacts: " + String(MOT_ISR_N));
  

  NULLFLAG++;
  //attachInterrupt(digitalPinToInterrupt(echo), BISR_Falling, FALLING);

  }

  Serial.println("Start");

  // put your setup code here, to run once:
}

void loop() {

  if (millis() - t1 >= 10) {

    dist = (float)count * coefficient;
    filtered = Filter.updateEstimate(dist);
    controlPos = constrain((filtered + delta), 0, 82);  //Получение желаемой позиции в мм
    controlStepPos = controlPos/MM_na_Shag;
    t1 = millis();
  }

   if (millis() - t2 >= 700) {

    PrintData();
    t2 = millis();

   }
  ///PrintData();
  //Serial.println(count);

  if (counterSteps < controlStepPos && counterSteps <= MAXSteps) {  //Движение вниз

    direction = 1;
    en = 0;
    digitalWrite(EN, en);
    digitalWrite(DIR, direction);

  } else if (counterSteps > controlStepPos && counterSteps >= MINSteps) {  //Движение вверх

    direction = 0;
    en = 0;
    digitalWrite(EN, en);
    digitalWrite(DIR, direction);

  } else if (counterSteps >= controlStepPos && counterSteps <= controlStepPos) {  //Остановка, если в зоне интереса

    en == 1;
    digitalWrite(EN, en);
    digitalWrite(DIR, 0);
  }
  // PORTH |= 0b00001000;  // PH3 pin 6
  // delayMicroseconds(10);
  // PORTH &= ~(1 << 3);
  // delayMicroseconds(10);
  // put your main code here, to run repeatedly:
}

//ISR(TIMER2_OVF_vect) {

  //PORTD |= 0b00100000;  //PD5 Pin5 Step
    //digitalWrite(STEP, HIGH);
  //PORTD &= ~(1 << 5);

//}

// ISR(TIMER4_COMPA_vect) {  //Старая версия со скважностью 50% и очень высокой частотой на датчик

//   if (flag == 1) {

//     PORTH |= 0b00001000;
//     flag = 0;

//   } else {

//     PORTH &= ~(1 << 3);
//     flag = 1;
//   }
// }

ISR(TIMER2_COMPA_vect) {  //Прямоугольный сигнал излучателя TRIG и сигнал на мотор

  TRIGGER();  //подача сигнала на датчик

  static int MOT_counter;
  static bool flagMove;
  
  MOT_counter++;  //Отсчёт тактов.

  if (MOT_counter == MOT_ISR_N) {  //Если настало время, обнуляется счётчик

    MOT_counter = 0;
    flagMove = 1;

  }

  if (en == 0 && flagMove == 1 && counterSteps < MAXSteps && counterSteps != controlStepPos) {

      PORTD |= 0b00100000;  //PD5 Pin5 Step
      PORTD &= ~(1 << 5);
      flagMove = 0;

      if (direction == 1) {

        counterSteps++;

      } else if (direction == 0) {

        counterSteps--;
      }

  }

  flagMove == 0;

}


void AISR_CHANGE() {  //Внешнее прерывание считывания ECHO
  //TCNT5 = 0;
  //flag = 0;

  if (!(PIND & 0b00000100) == 0) {

    // TIFR5 = (1 << TOV5);    //обработка прерывания
    // TIMSK5 = (1 << TOIE5);  //Включение прерывания по переполнению
    //cnt_ovf = 0;
    TCCR1A = TCCR1B = 0;                                //Обнуление таймеров и их отключение
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);  //делитель тактирования 1 16 МГЦ 4 мс переполнение делитель 2, на 8, 32 мс переполнение. Запуск таймеров
    //flag = 1;
    TCNT1 = 0x0000;  // Обнуление счётчика
    //countrising = micros();

  } else if (!(PIND & 0b00000100) == 1) {

    count = TCNT1;  //Запись значения счётчика

    TCCR1A = TCCR1B = 0;  //Обнуление счётчика и выключение таймеров

    //counterfalling = micros() - countrising;

    //flag = 0;
    //count = TCNT5;
  }
}

// void ISR(TIMER3_COMPA_vect) {  //Частота зависит от делителя и желаемой скорости

//   static bool flagMove;

//   if (en == 0 && flagMove == 1 && counterSteps < MAXSteps) {

//     digitalWrite(STEP, HIGH);

//     flagMove = 0;

//     if (direction == 1) {

//       counterSteps++;

//     } else {
//       counterSteps--;
//     }


//   } else if (en == 0 && flag == 0) {

//     digitalWrite(STEP, LOW);

//     flagMove = 1;
//   }
// }

void PrintData() {

  Serial.println();
  Serial.print(dist, 1);
  Serial.print(",");

  Serial.print(filtered, 1);
  Serial.print(",");
  // Serial.print(NULLFLAG, 3);
  // Serial.print(",");
  Serial.print(controlPos);
  Serial.print(",");
  Serial.print(controlStepPos);
  Serial.print(",");
  Serial.print(counterSteps);
  Serial.print(",");
  //Serial.print(direction);
  //Serial.print(",");


  // Serial.print(MOT_ISR_N);
  // Serial.print(",");
  // Serial.print(MOT_ISR_N);
  // Serial.print(",");
  // Serial.print(MOT_counter);
  // Serial.print(",");
}

void TRIGGER() { //Сигнал на триггер с частотой 100000/(per + 1)

  if (counternew <= 5) {  //Один отсчёт - это 10 мкс. 5 - 50 мкс. 50 мкс - высокий сигнал

    PORTD |= 0b00001000;

  } else {

    PORTD &= ~(1 << 3);
  }

  if (counternew >= per) {  // 16000000/OCR4B = 100 кГц исходной, Новая частота:100000/(число + 1). 999 => 100 Гц

    counternew = 0;  //Сброс каждые 160*62.5*per. То есть каждые 10 мс времени для per = 1000. итого 50 мкс высокий уровень для 10 мс периода.
  }

  counternew++;
}

void MOTOR() {
}
