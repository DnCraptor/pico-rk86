# RK8266 port for RP2040
Эмулятор Радио-86РК изначально на ESP8266 (см. https://github.com/klad-me/RK8266).<br/>
А тут порт на ZX Murmulator: https://github.com/AlexEkb4ever/MURMULATOR_classical_scheme

# Железо
Raspberry Pi Pico: эмулирует процессор и переферию РК86, генерирует видеосигнал стандарта VGA, обрабатывает PS/2 клавиатуру.

# Исходники
Во всех исходниках кодировка UTF-8.<br/>
Проект в стадии разработки.<br/>

# Эмулятор i8080
Используется эмулятор https://github.com/begoon/i8080-core<br/>
При работе RP2040 на 366 МГц, частота эмуляции i8080 1.7 МГц (базовая) Разгон переключается клавишей Scroll Lock!<br/>

# Переферия
На данный момент реализовано:
<ul>
<li>Процессор (ВМ80)</li>
<li>Экран (ВГ75, ИК57) - вывод композитного PAL-сигнала (только текст) на VGA разъём в формате 640*480 60Гц</li>
<li>Клавиатура (ВВ55) - PS/2 клавиатура с автоматической перекодировкой русских букв и спец.символов</li>
<li>Загрузка файлов с SD-card, из папки /rk86</li>
</ul>

# Джойстик (Dendy-8bit или Wii)
<ul>
<li>право-влево-вверх-вниз - понятно</li>
<li>А - ввод</li>
<li>В - пробел</li>
<li>SELECT - Esc</li>
<li>START - F12</li>
</ul>

# Клавиатура
<ul>
<li>Ctrl+Alt+Del - перезагрузка RP2040 (с выходом в М-ОС, если запуск был из неё)</li>
</ul>
