# Lessons Learned
This file is for thoughts about improvements, things to investigate, and lessons learned.

---
Investigation:

I'd like to test to see what the jitter looks like between a sync0 signal and a master process data cycle. I suspect that the master jitter sometimes interferes with the sync0 signal?
To test you'd have to splice some kind of wireshark monitoring between the first slave and the next? Since you can't see sync signals from the master. Can't see the sync packets, but why would the master need them? Also, the clocks sync fine anyway, but they will be within 1,000 ns at 1ms cycle but randomly fall out of PLL lock when error is >40,000ns. This just generates a quick warning, but this shouldn't be happening...

---
Lesson Learned:

Turns out decreasing the cycle period acutally helps things stay in sync. Sounds counter intuitive, but this allows the distributed clocks to sync together better since there are more samples per second. They can make more corrections.

---
Investigation:

I'm really curious why there is such a wide range of jitter for the master. Sometimes it will measure under 1,000ns of jitter, then spike to ~±30,000ns of jitter. I suspect the linux networking stack. From what I've read, its not meant for realtime communications? Looks like there are some patches for this?

---
Investigation:

I'm curious if I've been using the right drivers this whole time? On the linux box I've been working on, one of the ports has a different drive than the others. There is a file in the SOEM library titled drvcomment.txt that mentions you should configure the port correctly. Only this one port I can change as they reccomend. I suspect not configuring the port and not having the right drivers might add to the wild jitter.