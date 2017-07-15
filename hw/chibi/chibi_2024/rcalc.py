#!/usr/bin/env python3
"""
    MBI5026 current set resistor calculations

    The MBI5026's output current is set by a current set via a single resistor
    connected to its R_ext pin.

    To get a larger inter-frame dynamic range Megumin can switch between
    four different current ranges. The ratio between one current range and the
    next smaller one is r=1:8 (eq. -lg(r)=3 bit). This means at b=12bit BCM range we
    get a minimum of

        bmin = b+lg(r) = 8bit @ r=1:16, b=12bit
        
    worst-case in the intermediate ranges using a static current setting.
    Megumin uses BC847 small-signal NPN transistors to switch between three
    current ranges:

                        ┌─────────┐
                        │ MBI5026 │
                        │         │     
                        │    Rext─┼──┬──┤R1├───────────GND
                        │         │  │                 
                        └─────────┘  ├──┤R2├──┤BC847├──GND
                                     │                 
                                     ├──┤R3├──┤BC847├──GND
                                     │                 
                                     └──┤R4├──┤BC847├──GND

    The transistors are used to select either or none of {R2, R3, R4}. This means
    the R_ext pin sees either R1, R1||R2, R1||R3 or R1||R4. We don't do a full
    R-2R or similar DAC configuration as we only have to maintain the ratio r
    between ranges.

    Megumin's smallest BCM period is tb=250ns resulting in a base BCM rate of
    4MHz minus control overhead. This results in a BCM period and frame rate of

        Tm = tb*(2**b) = 1.024ms @ tb=250ns, b=12bit. 
        fm = 1/Tm ≈ 1kHz
    
    Now, if we want to modulate the display at a current range in between two
    of the preset ranges, we can switch between both ranges with a ratio of
    sqrt(r)=1:4 and still get a frame rate of
    
        f = fm/sqrt(r) = 250Hz @ fm=1kHz, r=1:16
        
    Normalized to the larger of the two ranges (here r1=1) we get the following
    equation for the ratio of the resulting modulated range:

        r_im1    = sqrt(r)*r1 = 0.25               @ r=1:16, r1=1
        r_im_tot = r_im1 + (1-sqrt(r))*r2 = 0.297  @ r2=r*r1

    Including the 2 bit gained by inter-frame modulation this results in the
    following basic ranges at framerate f=250Hz with a slight mid-range
    discontinuity at the mixed ranges:

         Range max │ Total bits
        ───────────┼──────────────────────
             1.000 │ 14
             0.297 | 16 (14 at mid-range)
             0.250 | 14

    The resistances of the resistors R1, R2, R3, R4 used are calculated in this
    script.
"""

prefixes = {' ': 1, 'k': 1e3, 'M': 1e6, 'm': 1e-3, 'μ': 1e-6, 'n': 1e-9}
def format_unit(val):
    for prefix, magnitude in prefixes.items():
        if 1.0 <= val/magnitude < 1000.0:
            return val/magnitude, prefix
    else:
        if val<1:
            return val/10e-9, 'n'
        else:
            return val/10e6, 'M'

def print_var(name, val, unit, **kwargs):
    scaled, prefix = format_unit(val)
    print('{} = {: >7.3f}{}{}'.format(name, scaled, prefix, unit), **kwargs)

r = 1/16
stages = 3
mod_r = 1/8
I_max_led = 0.01
n_boards = 20
n_digits_per_board = 8*4
n_leds = n_boards*n_digits_per_board*8
V_fw = 1.9 # V

print('r = 1:{:.0f}'.format(1/r))

I_min_led = I_max_led*(r**(stages-1)) # A
I_max_mod = I_max_led/mod_r
I_min_mod = I_min_led/mod_r
print_var('I_max_led', I_max_led, 'A')
print_var('I_max_mod', I_max_mod, 'A')
print_var('I_min_mod', I_min_mod, 'A')
if (I_max_mod > 0.09):
    print('\033[91mError: The MBI5026 has a maximum output current of 90mA!\033[0m')

Vrext = 1.26 # V
# Iout = 15 * Vrext/Rext | acc. to MBI5026 datasheet
R1 = 15*Vrext/I_min_mod
Itot_1 = n_leds * mod_r * I_min_mod
Ptot_1 = Itot_1 * V_fw
print_var('R1', R1, 'Ω', end='\t')
print_var('I1', I_min_mod, 'A', end='\t')
print_var('Itot_1', Itot_1, 'A', end='\t')
print_var('Ptot_1', Ptot_1, 'W')

for i in range(stages-2, -1, -1):
    # Rpar = 15*Vrext/(I_max_mod*r)
    # R1||R2 = 1/(1/R1 + 1/R2) =!= Rpar = 15*Vrext/(I_max_mod*r)
    # ⇒ 1/R1 + 1/R2 = 1/(15*Vrext/(I_max_mod*r))
    # ⇒ 1/R2 = 1/(15*Vrext/(I_max_mod*r)) - 1/R1
    # ⇒ R2 = 1/((I_max_mod*r)/(15*Vrext) - 1/R1)
    In = I_max_mod*(r**i)
    Rn = 1/(In/(15*Vrext) - 1/R1)
    Itot_n = n_leds * mod_r * In
    Ptot_n = Itot_n * V_fw
    scaled, prefix = format_unit(Rn)
    print_var('R{}'.format(stages-i), Rn, 'Ω', end='\t')
    print_var('I{}'.format(stages-i), In, 'A', end='\t')
    print_var('Itot_{}'.format(stages-i), Itot_n, 'A', end='\t')
    print_var('Ptot_{}'.format(stages-i), Ptot_n, 'W')

l = [ (1,     [1/2,    1/4,    1/8,    1/16,
               1/32,   1/64,   1/128,  1/256,
               1/512,  1/1024, 1/2048, 1/4096]),
      (1/16,  [1/2,    1/4,    1/8,    1/16,
               1/32,   1/64,   1/128,  1/256,
               1/512,  1/1024, 1/2048, 1/4096]),
      (1/256, [1/2,    1/4,    1/8,    1/16,
               1/32,   1/64,   1/128,  1/256,
               1/512,  1/1024, 1/2048, 1/4096])
    ]
for v, ls in l:
    for e in ls:
        print('{:> 12.10f} {:.0f}'.format(e*v, 0.5/(e*v)))

print('\033[93m---\033[0m')
l = [ (1/2**0,  [1/2,    1/4,    1/8,    1/16,
                 1/32,   1/64,   1/128,  1/256,
                 1/512,  1/1024, 1/2048, 1/4096]),
      (1/2**7,  [1/32,   1/64,   1/128,  1/256,
                 1/512,  1/1024, 1/2048, 1/4096]),
      (1/2**14, [1/32,   1/64,   1/128,  1/256,
                 1/512,  1/1024, 1/2048, 1/4096])
    ]
for v, ls in l:
    for e in ls:
        print('{:> 5.0f} {:> 12.10f} {:.0f}'.format(0.5/e, e*v, 0.5/(e*v)))
plain = sum(l[0][1])
optimized = sum([e for v, ls in l for e in ls])
overhead_percent = (optimized/plain-1)*100
print(plain, optimized, overhead_percent)
