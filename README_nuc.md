This README lists several tools to investigate potential problems with the Intel NUC PC hardware.

- [1. Diagnose <a name="diag"/>](#1-diagnose--a-name--diag---)
  * [1.1. SysLog <a name="sys_log"/>](#11-syslog--a-name--sys-log---)
  * [1.2. Kernel Messages <a name="dmesg"/>](#12-kernel-messages--a-name--dmesg---)
  * [1.3. Machine Check Events <a name="mcelog"/>](#13-machine-check-events--a-name--mcelog---)
  * [1.4. CPU Frequency <a name="cpu_freq"/>](#14-cpu-frequency--a-name--cpu-freq---)
  * [1.5. CPU Performance <a name="cpu_perf"/>](#15-cpu-performance--a-name--cpu-perf---)
  * [1.6. Process Monitoring <a name="proc_monitor"/>](#16-process-monitoring--a-name--proc-monitor---)
  * [1.7. CPU Temperature <a name="cpu_temp"/>](#17-cpu-temperature--a-name--cpu-temp---)
  * [1.8. HD Usage <a name="hd_use"/>](#18-hd-usage--a-name--hd-use---)
  * [1.9. Network Load <a name="iftop"/>](#19-network-load--a-name--iftop---)
  * [1.10. Network Performance <a name="net_perf"/>](#110-network-performance--a-name--net-perf---)
- [2. StressTest <a name="stress"/>](#2-stresstest--a-name--stress---)
  * [2.1. MemTest <a name="mem_test"/>](#21-memtest--a-name--mem-test---)
  * [2.2. StressTest <a name="stress_test"/>](#22-stresstest--a-name--stress-test---)
    + [2.2.1. Default <a name="default_stress_test"/>](#221-default--a-name--default-stress-test---)
    + [2.2.2. All stressors <a name="all_stress_test"/>](#222-all-stressors--a-name--all-stress-test---)
    + [2.2.3. Aggressive stressors <a name="aggressive_stress_test"/>](#223-aggressive-stressors--a-name--aggressive-stress-test---)
    + [2.2.4. Hot CPU <a name="hot_stress_test"/>](#224-hot-cpu--a-name--hot-stress-test---)
    + [2.2.5. Memory pressure <a name="mem_stress_test"/>](#225-memory-pressure--a-name--mem-stress-test---)

# 1. Diagnose <a name="diag"/>
## 1.1. SysLog <a name="sys_log"/>
```
vim /var/log/syslog
```

## 1.2. Kernel Messages <a name="dmesg"/>
```
dmesg -w
```

## 1.3. Machine Check Events <a name="mcelog"/>
```
vim /var/log/mcelog
```

## 1.4. CPU Frequency <a name="cpu_freq"/>
```
watch "lscpu | grep MHz"
```

## 1.5. CPU Performance <a name="cpu_perf"/>
```
watch mpstat -P ALL
```

## 1.6. Process Monitoring <a name="proc_monitor"/>
```
top -H
htop
ps fax
```
**HINTS**: `top -H` then sort for Status by pressing: `f`, on S = Process Status press `s`, then `Q` and reverse sort order with `R`

## 1.7. CPU Temperature <a name="cpu_temp"/>
```
watch sensors
```

## 1.8. HD Usage <a name="hd_use"/>
```
df -h
```

## 1.9. Network Load <a name="iftop"/>
```
sudo iftop -i <interface>
```

## 1.10. Network Performance <a name="net_perf"/>
```
iperf -s
iperf -c <server_ip>
```

# 2. StressTest <a name="stress"/>
## 2.1. MemTest <a name="mem_test"/>
```
sudo memtester 1024 5
```

## 2.2. StressTest <a name="stress_test"/>
### 2.2.1. Default <a name="default_stress_test"/>
```
stress-ng --cpu 4 --io 4 --vm 2
```
### 2.2.2. All stressors <a name="all_stress_test"/>
```
stress-ng --all 4
```
### 2.2.3. Aggressive stressors <a name="aggressive_stress_test"/>
```
sudo stress-ng --all 4 --aggressive
```
### 2.2.4. Hot CPU <a name="hot_stress_test"/>
```
stress-ng --matrix 0 --matrix-size 64 --tz
```
### 2.2.5. Memory pressure <a name="mem_stress_test"/>
```
stress-ng --brk 2 --stack 2 --bigheap 2
```
### 2.2.6. High load but low CPU <a name="load_stress_test"/>
```
stress-ng -c 20 -l 10
```
