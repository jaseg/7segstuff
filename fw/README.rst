Note on the LED modulation 
==========================

To control LED brightness, this project uses a modulation technique known as
"Binary Code Modulation" (BCM) or "Bit Angle Modulation" (BAM). The base idea is
to clock out all outputs in parallel bit-by-bit, then modulate this with a
precisely timed output enable signal. In contrast to PWM this allows almost
arbitrarily high channel counts and configurable modulation resolution at low
CPU overhead and high frame rates. In this project that is needed due to the
large number of channels (32) and the medium oversampling rate (1:8).

A good article explaining BCM can be found on batsocks.co.uk_ and a nice video
explaining has been published by mikeselectricstuff_. A possible optimization
for even smoother brightness fading (probably mostly in unmultiplexed
applications) has been discussed in the forums at picbasic.co.uk_.

.. _mikeselectricstuff: https://www.youtube.com/watch?v=Sq8SxVDO5wE
.. _`picbasic.co.uk`: http://www.picbasic.co.uk/forum/showthread.php?t=7393
.. _batsocks.co.uk: http://www.batsocks.co.uk/readme/art_bcm_1.htm

