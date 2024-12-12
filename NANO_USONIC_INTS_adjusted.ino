#include <SimpleKalmanFilter.h>
#include <EEPROM.h>

uint16_t controlSteps_Saved = 0;  //Сохранённая позиция моторов. Нужна для отсутствия сбития положения при отключении контроллера.

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

uint8_t input = 0;

#include "TMC2130Stepper.h"
TMC2130Stepper driver = TMC2130Stepper(EN, DIR, STEP, CS);

//TMC2130Stepper TMC2130 = TMC2130Stepper();

const int N = 159;  // N - число-отсчёт сброса таймера. 159+1 = 160. 160*62.5 = 10000 нс = 10 мкс.

float MM_na_oborot = 31.141592;  // Миллиметров будет пройдено за один оборот. Диаметр шестерни 10 мм.
int microsteps = 64;             // количество микрошагов двигателя

float MM_na_Shag = MM_na_oborot / (200 * microsteps);  // Миллиметров будет пройдено за один шаг. 200 - полных шагов двигателя на оборот
float speedMMs = 20;                                   // Скорость в мм/с

float Freq_MOT = speedMMs / MM_na_Shag;                      // Шагов в секунду при данной скорости
float period_MOT = 1000000.0 / Freq_MOT;                     // Микросекунд на шаг при данной скорости
float MOT_ISR_Tact = (period_MOT / ((N + 1) * 0.0625)) / 2;  // Сколько тактов нужно для данной конфигурации таймера 519 мкс для 150 мм/с, значит 51,9 тактов для N = 159

volatile int MOT_ISR_N = round(MOT_ISR_Tact);  // ОКРУГЛЁННОЕ ЗНАЧЕНИЕ ШАГОВ. Определяет скорость вращения мотора.

bool en = 1;  //ОТКЛЮЧЕНИЕ ЛОГИКИ ДРАЙВЕРА

volatile uint16_t counterSteps = 0;  //Реальное количество поданных импульсов/сделанных шагов, получаемое в ISR. Без учёта потерь

int direction = 0;                //Направление
float controlPos = 0;             //Необходимая позиция в мм. Пропорциональна количеству шагов
unsigned int controlStepPos = 0;  //Необходимая позиция в шагах, зависящая от позиции в миллиметрах

unsigned int MAXSteps = 40000;  //ОГРАНИЧИТЕЛЬ ШАГОВ
int MINSteps = 0;               //НИЖНИЙ ОГРАНИЧИТЕЛЬ ШАГОВ
int shift = 0;                  //ОТСТУП ДЛЯ КАЛИБРОВКИ

///////////////////////////////ДАТЧИК//////////////////////////////////////////////////////////////////////

const float timicros = 0.0625;                           //Время такта в микросекундах
const float speed = 0.343;                               //Скорость звука в мм/мкс
const float coefficient = 8.0 * speed * timicros / 2.0;  //коеффициент умножения для получения расстояния

volatile uint16_t count = 0, count2 = 0;
uint16_t cnt_ovf = 0;


uint32_t t1 = 0, t2 = 0;

bool flag;

float dist = 0;

volatile uint16_t counternew = 0;

const uint16_t per = 999;  //Период подачи сигнала на TRIG датчика // 1000-1

SimpleKalmanFilter Filter(2, 1, 0.03);  //1-ый коэффициент - амплитуда разлёта показаний от реального, третий - скорость изменения показаний
float filtered = 0;

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(1);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT_PULLUP);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);

  //Serial.begin(9600);
  while (!Serial)
    ;

  SPI.begin();
  Serial.println("Start...");
  driver.begin();           // Initiate pins and registeries
  driver.rms_current(500);  // Set stepper current to 500mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.stealthChop(1);    // Enable extremely quiet stepping
  driver.microsteps(microsteps);

  digitalWrite(EN, LOW);


  Serial.println(MOT_ISR_N);

  //Таймер ECHO для установления длительности сигнала.
  TCCR1A = TCCR1B = TCNT1 = cnt_ovf = 0;              //1 таймер на подсчёте прерываний
  TIFR1 = (1 << TOV1);                                //обработка прерывания
  TIMSK1 = (1 << TOIE1);                              //Включение прерывания по переполнению
  TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10);  //делитель тактирования 1 16 МГЦ

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

  count2 = 0;
  attachInterrupt(digitalPinToInterrupt(echo), AISR_CHANGE_ECHO, CHANGE);

  sei();

  EEPROM.get(0, controlSteps_Saved);  //Получение, записанного перед выключением, положения мотора

  if (controlSteps_Saved > MAXSteps) {

    counterSteps = 0;

  } else {

    counterSteps = controlSteps_Saved;
  }

  Serial.println("Start");
}

void loop() {

  if (millis() - t1 >= 10) {

    dist = (float)count * coefficient;
    filtered = Filter.updateEstimate(dist);  //Фильтрация значения дистанции

    if (filtered >= 82) {  //Ограничение максимальной дистанции

      controlPos = 82 - shift;

    } else {

      controlPos = filtered - shift;
    }

    controlStepPos = controlPos / MM_na_Shag;  //Получение желаемой позиции в шагах

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

    } else if (counterSteps >= controlStepPos && counterSteps <= controlStepPos) {  //Остановка, если в зоне равновесия

      en == 1;
      digitalWrite(EN, en);
      digitalWrite(DIR, 0);
    }

    t1 = millis();
  }

  inputData();

  if (millis() - t2 >= 100) {

    t2 = millis();
    //PrintData();
    EEPROM.put(0, counterSteps);  //Обновление сохранённого значения позиции моторов
    PrintData();

    //EEPROM.get(0, controlSteps_Saved);
  }
}

//ISR(TIMER2_OVF_vect) {

//PORTD |= 0b00100000;  //PD5 Pin5 Step
//digitalWrite(STEP, HIGH);
//PORTD &= ~(1 << 5);

//}

// ISR(TIMER4_COMPA_vect) {  //Старая версия со скважностью 50% ивысокой частотой подачи сигнала на датчик

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

  static uint16_t MOT_counter;
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


void AISR_CHANGE_ECHO() {  //Внешнее прерывание считывания ECHO
  //TCNT5 = 0;
  //flag = 0;

  if (!(PIND & 0b00000100) == 0) {

    TCCR1A = TCCR1B = 0;                                //Обнуление таймеров и их отключение
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);  //делитель тактирования 1 16 МГЦ 4 мс переполнение делитель 2, на 8, 32 мс переполнение. Запуск таймеров
    //flag = 1;
    TCNT1 = 0x0000;  // Обнуление счётчика

  } else if (!(PIND & 0b00000100) == 1) {

    count = TCNT1;  //Запись значения счётчика

    TCCR1A = TCCR1B = 0;  //Обнуление счётчика и выключение таймеров
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
  // Serial.print(dist, 1);
  // Serial.print(",");

  Serial.print(filtered, 1);
  Serial.print(",");
  // Serial.print(NULLFLAG, 3);
  // Serial.print(",");
  // Serial.print(controlPos);
  // Serial.print(",");

  Serial.print(controlStepPos);
  Serial.print(",");
  Serial.print(counterSteps);
  Serial.print(",");

  // Serial.println(controlSteps_Saved);
  // Serial.print(",");
  //Serial.print(direction);
  //Serial.print(",");
}

void TRIGGER() {  //Сигнал на триггер с частотой 100000/(per + 1)

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

void inputData() {

  if (Serial.available() > 0) {  //если есть доступные данные

    char buffer[] = { "" };
    String dannie = "";

    while (Serial.available()) {

      if (Serial.readBytesUntil(10, buffer, 1)) {
        //Serial.print("I received: ");
        //Serial.println(buffer[0]);
        dannie = dannie + buffer[0];
      }
    }

    dannie.trim();
    Serial.println(dannie);

    if (dannie.indexOf("sp") != -1) {

      uint8_t sppos = dannie.indexOf("sp") + 2;
      String speedrec = dannie.substring(sppos, sppos + 3);
      int receivedspeed = constrain(speedrec.toInt(), 1, 100);
      Serial.println("received speed: ");
      Serial.println(receivedspeed);

      recalculation(1, receivedspeed);
    }

    if (dannie.indexOf("sh") != -1) {

      uint8_t shpos = dannie.indexOf("sh") + 2;
      String Shiftrec = dannie.substring(shpos, shpos + 2);
      //Serial.println(Shiftrec);
      int receivedshift = constrain(Shiftrec.toInt(), 0, 82);
      Serial.println("received shift: ");
      Serial.println(receivedshift);

      recalculation(2, receivedshift);

    }

    

    
    //Serial.println(dannie);
  }
}

void recalculation(int opID, float val) {

  switch (opID) {

    case 1:

      Freq_MOT = val / MM_na_Shag;                      // Шагов в секунду при данной скорости
      period_MOT = 1000000.0 / Freq_MOT;                     // Микросекунд на шаг при данной скорости
      MOT_ISR_Tact = (period_MOT / ((N + 1) * 0.0625)) / 2;  // Сколько тактов нужно для данной конфигурации таймера 519 мкс для 150 мм/с, значит 51,9 тактов для N = 159

      MOT_ISR_N = round(MOT_ISR_Tact);  // ОКРУГЛЁННОЕ ЗНАЧЕНИЕ ШАГОВ. Определяет скорость вращения мотора.

      break;

    case 2:

      shift = val;

      break;
  }

}
