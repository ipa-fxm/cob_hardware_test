#!/usr/bin/env python2
#-*- coding: utf-8 -*-


from __future__ import print_function

from subprocess import Popen, PIPE

import fcntl
import os
import time
import signal
import datetime
import requests
import sys
import json
import tarfile
import StringIO

import argparse

class ThermalTest(object):

    def __init__(self, **kwargs):

        self.heating_time = kwargs.get('heating_time', 60*15)
        self.basepath = kwargs.get('basepath', '.')

        self.relevant_charts = [
            'cpu.core_throttling',
            'cpu.cpufreq',
            'sensors.coretemp-isa-0000_temperature',
        ]

        self.LEVEL_UNTESTED = 0
        self.LEVEL_OK = 1
        self.LEVEL_WARNING = 2
        self.LEVEL_CRITICAL = 3
        self.LUT_LEVEL_NAME = ['UNTESTED', 'OK', 'WARNING', 'CRITICAL']

        self.start_time = None
        self.stop_time = None

        self.proc = None
        self.interrupted_by_user = False
        signal.signal(signal.SIGINT, self.signal_handler)



    def signal_handler(self, sig, frame):
        #if self.proc is not None:
        #    print('send signal to proc')
        self.interrupted_by_user = True
        self.proc.send_signal(signal.SIGINT)


    def start_test(self):
        self.check_all()
        netdata_url_visual = 'http://%s:19999/#menu_cpu_submenu_throttling;after=%d;before=%d'


        print('heating up...')
        self.proc = Popen('stress-ng --matrix 0 --matrix-size 64 --tz'.split(' '), shell=False, stdout=PIPE)
        fcntl.fcntl(self.proc.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        time.sleep(0.01)

        self.start_time = time.time()
        dtime = 0
        running_marker = '/-\-'
        while self.proc.poll() is None and dtime < self.heating_time:
            dtime = time.time() - self.start_time
            self.stop_time = time.time()
            dt = datetime.timedelta(seconds=dtime)
            perc = dtime / self.heating_time * 100
            print('\r%s %.1f%% (running since %s)' % (running_marker[int(dtime) % len(running_marker)], perc, dt), end='')
            sys.stdout.flush()
            time.sleep(1)
        print('\rdone')

        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            self.proc.terminate()
        self.proc.wait()


        metric_levels = self.check_netdata_metrics()
        print('\nResults:')
        line_fmt = '  {chart_name:<%d}  {level}' % max(map(len, self.relevant_charts))
        for chart_name in self.relevant_charts:
            print(line_fmt.format(chart_name=chart_name, level=self.LUT_LEVEL_NAME[metric_levels[chart_name]]))

        print('\nthis is only a estimation and still in test phase, for more details visit the pure data from netdata')
        print('-->', netdata_url_visual % ('127.0.0.1', self.start_time * 1e3, self.stop_time * 1e3))
        try:
            proc = Popen('hostname -I'.split(' '), shell=False, stdout=PIPE)
            out, _ = proc.communicate()
            ip = out.strip('\n').strip()
            print('for remote robot access use')
            print('-->', netdata_url_visual % (ip, self.start_time * 1e3, self.stop_time * 1e3))
            #print('\nif needed, the plots are also avalable, you could copy them via scp')
            #print('--> scp ')
        except:
            pass



    def check_netdata_metrics(self):
        netdata_uri = 'http://127.0.0.1:19999/api/v1/data?chart={chart}&format=json&after={after}&before={before}'

        json_data = dict()

        tarfilename = 'thermal_test_{dtstr}_{identifier}.tar.gz'
        identifier = self.get_hostname_robot_identifier()
        tarfilename = tarfilename.format(identifier=identifier, dtstr=datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))


        with tarfile.open(os.sep.join([self.basepath, tarfilename]), "w|gz") as tar:

            for chart_name in self.relevant_charts:
                uri = netdata_uri.format(chart=chart_name, after=self.start_time, before=self.stop_time)
                result = self.do_netdata_request(uri)

                cfilename = 'thermal_test_{identifier}_{chartname}.json'
                identifier = self.get_hostname_robot_identifier()
                cfilename = cfilename.format(identifier=identifier, chartname=chart_name)

                data_io = StringIO.StringIO(result.text)
                info = tarfile.TarInfo(name=cfilename)
                info.size = len(data_io.buf)
                tar.addfile(tarinfo=info, fileobj=data_io)

                json_data[chart_name] = result.json()


            metric_levels = dict()
            for chart_name in self.relevant_charts:
                fnc_name = 'validate_netdata_metric__{chartname}'.format(chartname=chart_name.replace('.', '_').replace('-', '_'))
                metric_levels[chart_name] = self.LEVEL_UNTESTED
                if hasattr(self, fnc_name):
                    metric_levels[chart_name] = getattr(self, fnc_name)(json_data[chart_name])



            info_data = json.dumps(
                {
                    'start_time': self.start_time,
                    'start_time_fmt': '%s' % datetime.datetime.fromtimestamp(self.start_time),
                    'stop_time': self.stop_time,
                    'stop_time_fmt': '%s' % datetime.datetime.fromtimestamp(self.stop_time),
                    'heating_time': self.heating_time,
                    'interrupted_by_user': self.interrupted_by_user,
                    'host': os.uname()[1],
                    'env_robot': os.environ.get('ROBOT', ''),
                    'metric_levels_readable': {k: self.LUT_LEVEL_NAME[v] for k, v in metric_levels.items()},
                },
                indent=4
            )

            data_io = StringIO.StringIO(info_data)
            info = tarfile.TarInfo(name="info.json")
            info.size = len(data_io.buf)
            tar.addfile(tarinfo=info, fileobj=data_io)


        return metric_levels


    def validate_netdata_metric__sensors_coretemp_isa_0000_temperature(self, json_data):
        time_idx = json_data['labels'].index('time')
        zdata = zip(*json_data['data'])
        zdata.pop(time_idx)

        max_temp = max(map(max, zdata))

        if max_temp > 100:
            return self.LEVEL_CRITICAL
        elif max_temp > 90:
            return self.LEVEL_WARNING
        else:
            return self.LEVEL_OK


    def get_max_freq(self):
        try:
            proc = Popen('lscpu', shell=False, stdout=PIPE)
            out, _ = proc.communicate()
            for l in out.splitlines():
                if l.startswith('CPU max MHz:'):
                    return float(l.split(':')[1].strip(' '))
        except:
            return None


    def validate_netdata_metric__cpu_cpufreq(self, json_data):
        time_idx = json_data['labels'].index('time')
        zdata = zip(*json_data['data'])
        zdata.pop(time_idx)

        max_freq = self.get_max_freq()
        if max_freq is None:
            print('WARNING: lscpu faild, using estimated max cpu freq from data')
            max_freq = max(map(max, zdata))
        min_freq = min(map(min, zdata))

        #print('max_freq', max_freq)
        #print('min_freq', min_freq)

        if min_freq < max_freq * 0.75:
            return self.LEVEL_CRITICAL
        elif min_freq < max_freq * 0.95:
            return self.LEVEL_WARNING
        else:
            return self.LEVEL_OK


    def validate_netdata_metric__cpu_core_throttling(self, json_data):
        time_idx = json_data['labels'].index('time')
        zdata = zip(*json_data['data'])
        zdata.pop(time_idx)

        sum_throttling = sum(map(sum, zdata))

        return self.LEVEL_WARNING if sum_throttling > 0 else self.LEVEL_OK


    def get_hostname_robot_identifier(self):
        identifier = '{robot}.{hostname}'
        robot = os.environ.get('ROBOT', 'cobX-X')
        return identifier.format(hostname=os.uname()[1], robot=robot)


    def save_chartdata_to_file(self, chartname, chartdata):
        filename = 'thermal_test_{identifier}_{chartname}.json'
        identifier = self.get_hostname_robot_identifier()
        filename = filename.format(identifier=identifier, chartname=chartname)
        fullfilename = os.sep.join([self.basepath, filename])

        with open(fullfilename, 'w') as f:
            f.writelines(chartdata)
            f.flush()

        return filename, fullfilename


    def do_netdata_request(self, url):
        try:
            r = requests.get(url)
        except requests.ConnectionError as ex:
            print('NetData ConnectionError %r' % ex)
            return None

        if r.status_code != 200:
            print('NetData request not successful with status_code %d' % r.status_code)
            return None

        return r


    def check_netdata(self):
        check_ok = True

        url = 'http://127.0.0.1:19999/api/v1/charts'
        rdata = self.do_netdata_request(url).json()

        for chart in self.relevant_charts:
            if chart not in rdata['charts'].keys():
                check_ok = False
                print('Netdata-Chart %s is missing' % chart)

        return check_ok


    def check_stress_ng(self):
        try:
            proc = Popen('stress-ng -V'.split(' '), shell=False, stdout=PIPE)
            proc.wait()
        except OSError as ex:
            #if ex.errno is errno.ENOENT:
            #    print('not existing')
            #else:
            #    print(ex)
            return False
        return True


    def check_all(self):
        is_ok = True

        print('checking netdata... ', end='')
        if self.check_netdata():
            print('OK')
        else:
            print('FAILED')
            is_ok = False

        print('checking stress-ng... ', end='')
        if self.check_stress_ng():
            print('OK')
        else:
            print('FAILED')
            is_ok = False

        return is_ok


if __name__ == '__main__':
    sys.argv = [arg for idx, arg in enumerate(sys.argv) if not idx or not arg.startswith('__')]

    parser = argparse.ArgumentParser(conflict_handler='resolve')

    parser.add_argument('--outputdir', default='.', type=str, help="output directory where the results where saved")
    parser.add_argument('--testtime', default=15, type=int, help="how long the test should run in minuets, default: 15")
    argparse_result = parser.parse_args()

    tt = ThermalTest(basepath=argparse_result.outputdir, heating_time=argparse_result.testtime*60)
    tt.start_test()
