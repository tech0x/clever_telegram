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

1. get_telemetry - Пример вызова из телеграмм комманды

```
rosservice call /get_telemetry "{frame_id: ''}"
```

2. navigate -  Примеры вызовов navigate из телеграмм

```
rosservice call /navigate "{x: 0.0, y: 0.0, z: 0.5, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body', auto_arm: true}"
```

```
rosservice call /navigate "{z: 1.0, frame_id: 'body', auto_arm: true}"
```

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

Для полной замены пульта, стоит учитывать, как бы не казалась надежной связь, в любой момент интернет может быть прерван от нескольких секунд до неопределенного времени.

Поэтому, пульт с настроеным кил свичем всегда должен быть под рукой.

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

<br>
<img src="data:jpg;base64,
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">
