{ "stackmonitor_start", SCHED_PRIORITY_DEFAULT, 2048, stackmonitor_start_main },
{ "rand", SCHED_PRIORITY_DEFAULT, 2048, rand_main },
{ "i2c", SCHED_PRIORITY_DEFAULT, 2048, i2c_main },
{ "adc", SCHED_PRIORITY_DEFAULT, 2048, adc_main },
{ "tee", 100, 100, tee_main },
{ "stackmonitor_stop", SCHED_PRIORITY_DEFAULT, 2048, stackmonitor_stop_main },
{ "ostest", SCHED_PRIORITY_DEFAULT, 2048, ostest_main },
{ "alarm", 100, 2048, alarm_main },