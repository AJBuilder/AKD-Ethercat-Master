# EthercatBench

Main library: AKDEcatController.cpp

Two tests are included!



Future improvements

I'd like to test to see what the jitter looks like between a sync0 signal and a master process data cycle. I suspect that the master jitter sometimes interferes with the sync0 signal?
To test you'd have to splice some kind of wireshark monitoring between the first slave and the next? Since you can't see sync signals from the master. Can't see the sync packets, but why would the master need them? Also, the clocks sync fine anyway, but they will be within 1,000 ns at 1ms cycle but randomly fall out of PLL lock when error is >40,000ns. This just generates a quick warning, but this shouldn't be happening...