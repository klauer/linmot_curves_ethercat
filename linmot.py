from __future__ import print_function
import sys
import epics
import time
import struct
import ctypes
import logging
from threading import Lock
from collections import namedtuple


Transition = namedtuple('Transition', 'initial final status')
State = namedtuple('State', 'control_word data')

logger = logging.getLogger(__name__)


class CurveInfoStruct(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('data_offset', ctypes.c_uint16),
                ('object_type', ctypes.c_uint16),
                ('num_setpoints', ctypes.c_uint16),
                ('data_type_size', ctypes.c_uint16),
                ('_name', ctypes.c_byte * 22),
                ('curve_id', ctypes.c_uint16),
                ('x_length', ctypes.c_uint32),
                ('x_dim_uuid', ctypes.c_uint16),
                ('y_dim_uuid', ctypes.c_uint16),
                ('wizard_type', ctypes.c_uint16),
                ('_wizard_params', ctypes.c_uint32 * 7),
                ]

    @property
    def name(self):
        return ctypes.string_at(self._name).decode('latin-1')

    @property
    def wizard_params(self):
        return list(self._wizard_params)

    def get_info(self):
        for field_name, type_ in self._fields_:
            field_name = field_name.lstrip('_')
            yield (field_name, type_, getattr(self, field_name))


assert ctypes.sizeof(CurveInfoStruct) == 70


def unpack_curve_to_bytes(info):
    if len(info) == 18:
        return b''.join(struct.pack('<i', d) for d in info)
    elif len(info) == 70:
        return info
    else:
        raise ValueError('Invalid curve data length')


def parse_curve_info(info):
    info = unpack_curve_to_bytes(info)
    st = CurveInfoStruct()
    ctypes.memmove(ctypes.addressof(st), info, len(info))
    return st


class Curve(object):
    def __init__(self, info, setpoints):
        self.info = parse_curve_info(info)
        self.setpoints = setpoints


class CurveAccess(object):
    def __init__(self, prefix='WIRE:B34:97:CONFIGMODULE:'):
        self._index_in = epics.PV(prefix + 'INDEXIN', auto_monitor=False)
        self._status_in = epics.PV(prefix + 'STATUSWORD', auto_monitor=False)
        self._value_in = epics.PV(prefix + 'VALUEIN', auto_monitor=False)

        self._control_word = epics.PV(prefix + 'CONTROLWORD',
                                      auto_monitor=False)
        self._index_out = epics.PV(prefix + 'INDEXOUT', auto_monitor=False)
        self._value_out = epics.PV(prefix + 'VALUEOUT', auto_monitor=False)

        self._pvs = [self._index_in, self._status_in, self._value_in,
                     self._control_word, self._index_out, self._value_out
                     ]

        self._lock = Lock()

        for pv in self._pvs:
            pv.wait_for_connection()
            assert pv.connected, '{} not connected'.format(pv)

    @property
    def index_in(self):
        return self._index_in.get()

    @property
    def status_in(self):
        return self._status_in.get()

    @property
    def value_in(self):
        return self._value_in.get()

    @property
    def control_word(self):
        return self._control_word.get()

    @control_word.setter
    def control_word(self, value):
        self._control_word.put(value)

    @property
    def index_out(self):
        return self._index_out.get()

    @index_out.setter
    def index_out(self, value):
        self._index_out.put(value)

    @property
    def value_out(self):
        return self._value_out.get()

    @value_out.setter
    def value_out(self, value):
        self._value_out.put(value)

    def _run(self, initial_state, transition_list, state_info):
        # with self._lock:
        state = initial_state
        cur_state_info = state_info[initial_state]
        self.control_word = cur_state_info.control_word
        data = {}

        transitions = {tr.initial: {} for tr in transition_list}
        for tr in transition_list:
            transitions[tr.initial][tr.status] = tr.final

        logger.debug('State %s %s', state, cur_state_info)
        while cur_state_info.data not in ('error', 'success'):
            status_in = self.status_in
            try:
                new_state = transitions[state][status_in]
            except KeyError:
                continue
            else:
                logger.debug('Received %s', status_in)
                logger.debug('Transition from state %s to state %s',
                             state, new_state)

                if cur_state_info.data is not None:
                    if cur_state_info.data not in data:
                        data[cur_state_info.data] = []

                    data[cur_state_info.data].append(self.value_in)
                    logger.debug('Received word for %r: %s',
                                 cur_state_info.data, self.value_in)

                state = new_state
                cur_state_info = state_info[state]

                logger.debug('State %s %s', state, cur_state_info)
                self.control_word = cur_state_info.control_word

        logger.debug('Final state %s %s', state, cur_state_info)
        logger.debug('Data:')
        for key, d in sorted(data.items()):
            logger.debug('\t%s = %r', key, d)
        return (state, data)

    def read_curve(self, curve_index):
        transitions = [Transition(40,  41, 0x001),
                       Transition(41,  42, 0x402),
                       Transition(41,  43, 0x002),
                       Transition(42,  41, 0x403),
                       Transition(42,  43, 0x003),
                       Transition(43,  44, 0x404),
                       Transition(43, 100, 0x004),
                       Transition(44,  43, 0x405),
                       Transition(44, 100, 0x005),
                       ]
        state_info = {
            40:  State(0x6001, None),
            41:  State(0x6102, 'info'),
            42:  State(0x6103, 'info'),
            43:  State(0x6204, 'data'),
            44:  State(0x6205, 'data'),
            90:  State(0x0000, 'error'),
            100: State(0x0000, 'success'),
        }

        self.index_out = curve_index
        self.control_word = 0x6001
        final_state, data = self._run(40, transitions, state_info)
        if final_state != 100:
            raise RuntimeError('invalid final state?')
        return data


def test(prefix):
    acc = CurveAccess(prefix)

    curves = (1, 2, 3, 5)
    data = {}
    for curve in curves:
        data[curve] = acc.read_curve(curve)

    for curve in curves:
        info = parse_curve_info(curve['info'])
        print('Curve {!r}'.format(info.name))
        for field_name, type_, value in info.get_info():
            print('\t{} = {!r}'.format(field_name, value))
        print('\t{!r}'.format(curve['data']))


if __name__ == '__main__':

    try:
        prefix = sys.argv[1]
    except IndexError:
        prefix = 'WIRE:B34:97:CONFIGMODULE:'

    logging.basicConfig()
    logger.setLevel(logging.DEBUG)

    test(prefix=prefix)
