# ROS Clever (PX4) Telegram Bot

## Описание.
Что можно делать с помощью бота, без использования qGroundControl:
1. Активировать стандартные режимы arm, disarm, land, takeoff
2. Просматривать image, distance данные с топиков ROS.
3. Проводить калибровку level, gyro, accel.
4. Вызывать selfcheck из пакета клевер и получать в телеграм его отчет.
5. Работа с профайлами, можно использовать сразу до 3х профайлов, для сохранения разных конфигураций FCU и быстрого переключения между ними, просмотра отличий в параметрах и сохранения.
6. Прямой доступ к консоли mavlink полетного контроллера, позволит легко получить или изменить параметры полетного контроллера не предусмотренные ботом.
7. Предполетный набор проверок, можно просмотреть hz парметры основной камеры, получить данные get_telemetry, а так же вторичный набор диагностических комманд commander status, sensors status, commander check, mavlink status и тому подобное.

### Разделы system и navigate 

Раздел system предусмотрен для скриптов выполняемых в распбери, по умолчанию там заложены следующие скрипты:

1. undistortion - Запускает ноду для отображения картинки с камеры с учетом коррекции линзы, как ее будет видеть opencv, после запуска надо перейти в раздел images и выбрать отображение undistortion

2. reboot_rasp - Перезапускает полетный компьютер.

3. halt - Корректно останавливает полетный компьютер, что бы не допустить повреждения файловой системы.

4. reboot_clever - Перезапускает клевер.

5. Для экономии ресурсов, данные ноды можно отключить из основного конфига клевера и активировать их, когда это необходимо.
web_server - Запускает веб сервер.
aruco - Запускает aruco - ноду.


Раздел navigation предусмотрен для произвольных скриптов и полетных программ, которые можно активировать по необходимости.

1. get_telemetry.sh - Пример вызова из телеграмм комманды

```
rosservice call /get_telemetry "{frame_id: ''}"
```

2. navigate_z0.5.sh -  Примеры вызовов navigate из телеграмм

```
rosservice call /navigate "{x: 0.0, y: 0.0, z: 0.5, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body', auto_arm: true}"
```

3. navigate_z1.0.sh

```
rosservice call /navigate "{z: 1.0, frame_id: 'body', auto_arm: true}"
```

4. navigate_z1.5.sh

```
rosservice call /navigate "{z: 1.5, frame_id: 'body', auto_arm: true}"
```


### Набор быстрых комманд

/start                    Инициирует основное меню

/selfcheck                Активирует запуск clever - selfcheck ноды

/list xxx                 Запускает поиск параметров FCU в названии которых встречается ххх, для проверки можно использовать cbr например.

/set px4_param => value   Изменяет параметр FCU выставляя значение value. Стоит учитывать что валидность значения тут проверена быть не может, поэтому нужно точно знать что можно и нужно выставлять.

/save                     Сохраняет все параметры FCU в профайл0 

/diff                     Отображает список измененых параметров в FCU относительно профайл0

/upload                   Выгружает локальный профайл0 в полетный контроллер.

### Для чего нельзя использовать этого бота:

Для начальной настройки полетного контроллера и клевера их в любом случае надо настраивать предварительно.

Для полной замены пульта, стоит учитывать, какой бы надежной не казалась связь, в любой момент интернет может быть прерван от нескольких секунд до неопределенного времени.

Поэтому, пульт с настроеным kill switch всегда должен быть под рукой.

Для реальных полетов вне закрытых пространств, бот всего лишь средство упрощающее отладку определенных проблем, но не более того.

Ну и разумеется все что вы делаете, делаете на свой страх и риск, если ваши действия вызовут, поломки, аварии, катастрофы, цунами, эпидемии, я предупреждал.

### Устновка:

Устанавливать из под пользователя pi. Не из под root.

```
pip install certifi --user
```

```
git clone https://github.com/tech0x/clever_telegram.git /home/pi/catkin_ws/src/clever_telegram/
```
    
```
git clone https://github.com/python-telegram-bot/python-telegram-bot.git /tmp/python-telegram-bot && mv /tmp/python-telegram-bot/telegram /home/pi/catkin_ws/src/clever_telegram/scripts
```

или установить локальную копию
    
```
cd /home/pi/catkin_ws/src/clever_telegram/scripts/ && tar -xzpvf ./telegram.tar.gz 
```

Далее необходимо создать телеграмм бота, 

как описано здесь: https://core.telegram.org/bots#6-botfather
    
После создания телеграмм даст ключ от бота (примерно такого вида 110201543:AAHdqTcvCH1vGWJxfSeofSAs0K5PALDsaw), его нужно вписать в файле вместо поля BOT_TOKEN

/home/pi/catkin_ws/src/clever_telegram/config/token.yaml

После делаем первый запуск и фиксируем наш идентификатор телеграм, что бы никто посторонний не мог отправлять комманды.

```
roslaunch clever_telegram clever_telegram.launch
```
    
Через меню поиск находим своего бота и отправляем ему текстовое сообщение /start
    
В ответ подгружается основная панель управления.
    
И на любой последующий клик должно выдаваться сообщение 
    
Incorrect chatid, please update chatid to 442000126 in token.yaml, and restart node
    
Вписывыаем в файл token.yaml, вместо chatid номер который выдался вместо 442000126.
    
Перезапускаем ноду и пользуемся.

Для того что бы прописать телеграмм в загрузку надо выполнить комманду из под рута.

```
systemctl enable /home/pi/catkin_ws/src/clever_telegram/systemd/clever_telegram.service 
```

<img src="https://raw.githubusercontent.com/tech0x/clever_telegram/master/images/undistortion.png?raw=true"/><br>
<img src="https://raw.githubusercontent.com/tech0x/clever_telegram/master/images/calibrate.png?raw=true"/><br>
<img src="https://raw.githubusercontent.com/tech0x/clever_telegram/master/images/get_telemetry.png?raw=true"/><br>
<img src="https://raw.githubusercontent.com/tech0x/clever_telegram/master/images/list_filter.png?raw=true"/><br>
<img src="https://raw.githubusercontent.com/tech0x/clever_telegram/master/images/profiles.png?raw=true"/><br>
<img src="https://raw.githubusercontent.com/tech0x/clever_telegram/master/images/range.png?raw=true"/><br>
<img src="https://raw.githubusercontent.com/tech0x/clever_telegram/master/images/set_value.png?raw=true"/><br>
<img src="https://raw.githubusercontent.com/tech0x/clever_telegram/master/images/main_menu.png?raw=true"/><br>

<br>
<br>
