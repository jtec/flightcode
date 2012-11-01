target remote localhost:61234
monitor reset

load bin/main.elf
symbol-file bin/main.elf
tbreak main
continue