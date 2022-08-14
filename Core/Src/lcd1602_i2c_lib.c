#include "lcd1602_i2c_lib.h"

#define adress_lcd (0x27<<1)
extern I2C_HandleTypeDef hi2c1;
bool backlight = 1;
char tx_buffer_lcd[40] = {0, };
uint8_t global_buffer = 0;

/// Функция предназначена для отправки байта данных по шине i2c. Содержит в себе Delay. Без него инициализация дисплея не проходит.
/// \param *init_Data - байт, например 0x25, где 2 (0010) это DB7-DB4 или DB3-DB0, а 5(0101) это сигналы LED, E, RW, RS соответственно
static void lcd1602_Send_init_Data(uint8_t *init_Data) {
	if (backlight) {
		*init_Data |= 0x08; //Включить подсветку
	} else {
		*init_Data &= ~0x08; //Выключить подсветку
	}
	*init_Data |= 0x04; // Устанавливаем стробирующий сигнал E в 1
	HAL_I2C_Master_Transmit(&hi2c1, adress_lcd, init_Data, 1, 10);
	HAL_Delay(5);
	*init_Data &= ~0x04; // Устанавливаем стробирующий сигнал E в 0
	HAL_I2C_Master_Transmit(&hi2c1, adress_lcd, init_Data, 1, 10);
	HAL_Delay(5);
}

/// Функция отправки байта информации на дисплей
/// \param Data - Байт данныйх
static void lcd1602_Write_byte(uint8_t Data) {
	HAL_I2C_Master_Transmit(&hi2c1, adress_lcd, &Data, 1, 10);
}

static void lcd1602_Send_cmd(uint8_t Data) {
	Data <<= 4;
	lcd1602_Write_byte(global_buffer |= 0x04); // Устанавливаем стробирующий сигнал E в 1
	lcd1602_Write_byte(global_buffer | Data); // Отправляем в дисплей полученный и сдвинутый байт
	lcd1602_Write_byte(global_buffer &= ~0x04);	// Устанавливаем стробирующий сигнал E в 0.
}

/// Функция отправки байта данных на дисплей
/// \param Data - байт данных
/// \param mode - отправка команды. 1 - RW = 1(отправка данных). 0 - RW = 0(отправка команды).
static void lcd1602_Send_data_symbol(uint8_t Data, uint8_t mode) {
	if (mode == 0) {
		lcd1602_Write_byte(global_buffer &= ~0x01); // RS = 0
	} else {
		lcd1602_Write_byte(global_buffer |= 0x01); // RS = 1
	}
	uint8_t MSB_Data = 0;
	MSB_Data = Data >> 4; // Сдвигаем полученный байт на 4 позичии и записываем в переменную
	lcd1602_Send_cmd(MSB_Data);	// Отправляем первые 4 бита полученного байта
	lcd1602_Send_cmd(Data);	   // Отправляем последние 4 бита полученного байта
}

/// Функция предназначена для отправки байта данных по шине i2c
/// \param *init_Data - байт, например 0x25, где 2 (0010) это DB7-DB4 или DB3-DB0, а 5(0101) это сигналы LED, E, RW, RS соответственно
static void lcd1602_Send_data(uint8_t *Data) {

	if (backlight) {
		*Data |= 0x08;
	} else {
		*Data &= ~0x08;
	}
	*Data |= 0x04; // устанавливаем стробирующий сигнал E в 1
	HAL_I2C_Master_Transmit(&hi2c1, adress_lcd, Data, 1, 10);
	*Data &= ~0x04; // устанавливаем стробирующий сигнал E в 0
	HAL_I2C_Master_Transmit(&hi2c1, adress_lcd, Data, 1, 10);
}

void lcd1602_Init(void) {
	/*========Power on========*/
	uint8_t tx_buffer = 0x30;
	/*========Wait for more than 15 ms after Vcc rises to 4.5V========*/
	HAL_Delay(15);
	/*========BF can not be checked before this instruction.========*/
	/*========Function set ( Interface is 8 bits long.========*/
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Wait for more 4.1 ms========*/
	HAL_Delay(5);
	/*========BF can not be checked before this instruction.========*/
	/*========Function set ( Interface is 8 bits long.========*/
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Wait for more 100 microsec========*/
	HAL_Delay(1);
	/*========BF can not be checked before this instruction.========*/
	/*========Function set ( Interface is 8 bits long.========*/
	lcd1602_Send_init_Data(&tx_buffer);

	/*========Включаем 4х-битный интерфейс========*/
	tx_buffer = 0x20;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Включаем 4х-битный интерфейс========*/

	/*======2 строки, шрифт 5х8======*/
	tx_buffer = 0x20;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x80;
	lcd1602_Send_init_Data(&tx_buffer);
	/*======2 строки, шрифт 5х8======*/

	/*========Выключить дисплей========*/
	tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x80;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Выключить дисплей========*/

	/*========Очистить дисплей========*/
	tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x10;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Очистить дисплей========*/

	/*========Режим сдвига курсора========*/
	tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x30;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Режим сдвига курсора========*/

	/*========Инициализация завершена. Включить дисплей========*/
	tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0xC0;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Инициализация завершена. Включить дисплей========*/
}

/// Функция вывода символа на дисплей
/// \param* symbol - символ в кодировке utf-8
void lcd1602_Print_symbol(uint8_t symbol) {
	uint8_t command;
	command = ((symbol & 0xf0) | 0x09); //Формирование верхнего полубайта в команду для дисплея
	lcd1602_Send_data(&command);
	command = ((symbol & 0x0f) << 4) | 0x09; //Формирование нижнего полубайта в команду для дисплея
	lcd1602_Send_data(&command);
}

/// Функция вывода символа на дисплей
/// \param *message - массив, который отправляем на дисплей.
/// Максимальная длина сообщения - 40 символов.
void lcd1602_Print_text(char *message) {
	for (int i = 0; i < strlen(message); i++) {
		lcd1602_Print_symbol(message[i]);
	}
}

/// Функция установки курсора для вывода текста на дисплей
/// \param x - координата по оси x. от 0 до 39.
/// \param y - координата по оси y. от 0 до 3.
/// Видимая область:
/// Для дисплеев 1602 max x = 15, max y = 1.
/// Для дисплеев 2004 max x = 19, max y = 3.
void lcd1602_SetCursor(uint8_t x, uint8_t y) {
	uint8_t command, adr;
	if (y > 3)
		y = 3;
	if (x > 39)
		x = 39;
	if (y == 0) {
		adr = x;
	}
	if (y == 1) {
		adr = x + 0x40;
	}
	if (y == 2) {
		adr = x + 0x14;
	}
	if (y == 3) {
		adr = x + 0x54;
	}
	command = ((adr & 0xf0) | 0x80);
	lcd1602_Send_data(&command);

	command = (adr << 4);
	lcd1602_Send_data(&command);

}

/// Функция перемещения текста влево
/// Если ее повторять с периодичностью, получится бегущая строка
void lcd1602_Move_to_the_left(void) {
	uint8_t command;
	command = 0x18;
	lcd1602_Send_data(&command);

	command = 0x88;
	lcd1602_Send_data(&command);
}

/// Функция перемещения текста вправо
/// Если ее повторять с периодичностью, получится бегущая строка
void lcd1602_Move_to_the_right(void) {
	uint8_t command;
	command = 0x18;
	lcd1602_Send_data(&command);

	command = 0xC8;
	lcd1602_Send_data(&command);
}

/// Булевая функция включения/выключения подсветки
/// \param state - состояние подсветки.
/// 1 - вкл. 0 - выкл.
void lcd1602_Backlight(bool state) {
	if (state) {
		backlight = true;
	} else {
		backlight = false;
	}
}

/// Функция создания своего собственного символа и запись его в память.
/// \param *my_Symbol - массив с символом
/// \param memory_adress - номер ячейки: от 1 до 8. Всего 8 ячеек.
void lcd1602_Create_symbol(uint8_t *my_Symbol, uint8_t memory_adress) {
	lcd1602_Send_data_symbol(((memory_adress * 8) | 0x40), 0);
	for (uint8_t i = 0; i < 8; i++) {
		lcd1602_Send_data_symbol(my_Symbol[i], 1); // Записываем данные побайтово в память
	}
}

void lcd1602_Clean(void) {
/// Аппаратная функция очистки дисплея.
/// Удаляет весь текст, возвращает курсор в начальное положение.
	uint8_t tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x10;
	lcd1602_Send_init_Data(&tx_buffer);

}

void lcd1602_Clean_Text(void) {
/// Альтернативная функция очистки дисплея
/// Заполняет все поле памяти пробелами
/// Работает быстрее, чем lcd1602_Clean, но в отличии от нее не возвращает курсор в начальное положение
	lcd1602_SetCursor(0, 0);
	lcd1602_Print_text("                                        ");
	lcd1602_SetCursor(0, 1);
	lcd1602_Print_text("                                        ");
}
