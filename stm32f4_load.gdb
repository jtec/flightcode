target remote loacalhost:61234
monitor reset

load bin/main.elf
symbol-file bin/main.elf
tbreak main
continue