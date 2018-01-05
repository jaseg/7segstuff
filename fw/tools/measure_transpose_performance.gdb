target remote localhost:3333
file main.elf
load
set print pretty on
set pagination off
cont
print cvr/50.0F
cont
print cvr/50.0F
cont
print cvr/50.0F
cont
print cvr/50.0F
cont
print cvr/50.0F
cont
quit
