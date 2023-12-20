#!/usr/bin/env python3
from ev3dev.ev3 import *
from time       import sleep

m1 = LargeMotor('outB')
m2 = LargeMotor('outC')
assert m1.connected, "Connecter un large motor sur outB"

m1.run_forever(speed_sp = 200)

sleep(2)
m1.run_forever(speed_sp = 0)
m2.run_forever(speed_sp = 200)
m2.run_forever(speed_sp = 0)

sleep(1)