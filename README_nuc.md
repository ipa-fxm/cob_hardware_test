This README lists several tools to investigate potential problems with the Intel NUC PC hardware.

1. [Diagnose](#diag)
1.1. [SysLog](#sys_log)
1.2. [CPU Temperature](#cpu_temp)
1.3. [HD Usage](#hd_use)
2. [StressTest](#stress)
2.1. [MemTest](#mem_test)
2.2. [StressTest](#stress_test)

# 1. Diagnose <a name="diag"></a>
## 1.1. SysLog <a name="sys_log"></a>
```
vim /var/log/syslog
```

# 1.2. CPU Temperature <a name="cpu_temp"></a>
```
watch sensors
```

# 1.3. HD Usage <a name="hd_use"></a>
```
df -h
```

# 2. StressTest <a name="stress"></a>
# 2.1. MemTest <a name="mem_test"></a>
```
sudo memtester 1024 5
```

# 2.2. StressTest <a name="stress_test"></a>
```
stress-ng -c 4
```
