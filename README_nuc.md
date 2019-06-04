This README lists several tools to investigate potential problems with the Intel NUC PC hardware.

1. [Diagnose](#diag)
1.1. [SysLog](#sys_log)
1.2. [Kernel Messages](#dmesg)
1.3. [Machine Check Events](#mcelog)
1.4. [CPU Freq.](#cpu_freq)
1.5. [CPU Performance](#cpu_perf)
1.6. [Process Monitoring](#proc_monitor)
1.7. [CPU Temperature](#cpu_temp)
1.8. [HD Usage](#hd_use)
1.9. [Network Load](#iftop)
1.10. [Network Performance](#net_perf)

2. [StressTest](#stress)
2.1. [MemTest](#mem_test)
2.2. [StressTest](#stress_test)
2.2.1. [Default](#default_stress_test)
2.2.2. [All stressors](#all_stress_test)
2.2.3. [Aggressive stressors](#aggressive_stress_test)
2.2.4. [Hot CPU](#hot_stress_test)
2.2.5. [Memory pressure](#mem_stress_test)

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
