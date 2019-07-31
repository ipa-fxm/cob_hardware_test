#!/usr/bin/python
# -*- coding: utf-8 -*-


import sys
import os
import datetime

import numpy as np
import matplotlib.pyplot as plt

import json
import tarfile



def parse_netdata_data(rdata):
    raw_data = rdata['data']
    clean_data = [r for r in raw_data if not None in r]

    sdata = zip(*clean_data)

    d = dict()
    for idx, label in enumerate(rdata['labels']):
        np_array = np.array(sdata[idx])

        if np_array.dtype == np.object:
            print "Data from NetData malformed", label

        d[label] = np_array

    return d


def plot_parsed_data(data, axis, **plotargs):
    timeline = data['time']
    for k, v in data.items():
        if k == 'time':
            continue

        axis.plot(timeline, v, label=k, **plotargs)


def get_netdata_mean(data):
    data_list = list()
    for k, v in data.items():
        if k == 'time':
            continue
        data_list.append(v)

    data_stack = np.vstack(data_list)
    data_mean = np.mean(data_stack, axis=0)
    return data_mean


def plot_parsed_data_mean(data, axis, **plotargs):
    data_mean = get_netdata_mean(data)
    timeline = data['time']
    axis.plot(timeline, data_mean, **plotargs)


def plot_parsed_data_mean_sliding(data, axis, N, **plotargs):
    data_mean = get_netdata_mean(data)

    if N * 2 > len(data_mean):
        return

    timeline = data['time']
    conv = np.convolve(data_mean, np.ones((N,)) / N, mode='same')
    axis.plot(timeline[N:-N], conv[N:-N], **plotargs)

    pass


def plot_single_results_targz(tarfilename, show_plot=False, show_only=False, do_pdf=False):
    metric_names = [
        'cpu.cpufreq',
        'sensors.coretemp-isa-0000_temperature',
    ]

    filemap = list()

    tar = tarfile.open(tarfilename, "r:gz")
    for member in tar.getmembers():
        # print('member= ' + str(member))
        f = tar.extractfile(member)

        if f:
            if f.name == "info.json":
                continue
            # print f.name

            title_pattern = '{metric}'
            for metric in metric_names:
                if metric in f.name:
                    title = title_pattern.format(metric=metric)
                    filemap.append((title, f))
                    break

    nrows = len(filemap)
    fig, ax = plt.subplots(nrows=nrows, sharex=True, figsize=(12, 3.5 * nrows))

    min_timestamp = None
    pdata_list = list()
    for title, f in filemap:
        try:
            jdata = json.load(f)
            pdata = parse_netdata_data(jdata)
            current_min_timestamp = pdata['time'].min()
            min_timestamp = min(current_min_timestamp, min_timestamp) if min_timestamp else current_min_timestamp
            pdata_list.append((title, pdata))
        except IOError:
            print 'IOError for file %s' % f.name
            return

    max_xdata = None
    for idx, (title, pdata) in enumerate(pdata_list):
        cax = ax[idx]

        pdata['time'] -= min_timestamp
        current_max_xdata = pdata['time'].max()
        max_xdata = max(current_max_xdata, max_xdata) if max_xdata else current_max_xdata

        plot_parsed_data(pdata, cax, color='blue', linewidth=1, alpha=0.5)
        # plot_parsed_data_mean(pdata, cax, label='mean')
        plot_parsed_data_mean_sliding(pdata, cax, 30, color='orange', linewidth=2, alpha=0.9,
                                      label='sliding mean')
        # plot_parsed_data_mean_sliding(pdata, cax, 120, color='magenta', linewidth=2, alpha=0.7, label='sliding mean')

        plt.xlim(-10, max_xdata + 10)

        cax.set_title(title)
        cax.grid()

    dt_start = datetime.datetime.fromtimestamp(min_timestamp)
    plt.suptitle('data samples from %s - File: %s' % (dt_start, tarfilename))
    plt.tight_layout(rect=(0, 0, 1, 0.975))

    if not show_only:
        plt.savefig(tarfilename.replace('.tar.gz', '.png' if not do_pdf else '.pdf'))

    if show_plot or show_only:
        plt.show()



if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print('please provide test prefix (e.g. thermal_test_cobX-X.fmw-xps-13)')
        sys.exit(-1)

    show_plot = '--show' in sys.argv[1:]
    show_only = '--show-only' in sys.argv[1:]
    do_pdf = '--pdf' in sys.argv[1:]
    show_help = '--help' in sys.argv[1:]

    if show_help:
        print 'optional arguments:'
        print '  --show       opens a new window with the plot'
        print '  --show-only  same as --show but no files where written to disk'
        print '  --pdf        save as output an pdf instead an png-image'
        exit(0)

    for f in sys.argv[1:]:
        if os.path.isfile(f) and f.endswith('.tar.gz'):
            print('plotting file: ' + f)
            plot_single_results_targz(f, show_plot=show_plot, show_only=show_only, do_pdf=do_pdf)




