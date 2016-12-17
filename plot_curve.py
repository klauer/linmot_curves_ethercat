from __future__ import print_function
from results import curves
from linmot import parse_curve_info

import matplotlib

matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt


plt.ion()
for cid, curve in sorted(curves.items()):
    fig = plt.figure()
    name = 'Curve #{}'.format(cid)
    fig.set_label(name)
    plt.title(name)
    plt.plot(curve['data'], 'o')

    print('-', name)
    info = parse_curve_info(curve['info'])
    for field_name, type_, value in info.get_info():
        print('\t{} = {!r}'.format(field_name, value))

plt.show()

