from __future__ import print_function
import sys
import math
import struct
import ctypes
import logging
import time
from threading import Lock
from collections import (namedtuple, deque)

import epics


Transition = namedtuple('Transition', 'initial final status')
State = namedtuple('State', 'control_word data')

logger = logging.getLogger(__name__)


class CurveInfoStruct(ctypes.Structure):
    _size_ = 70  # not part of ctypes.Structure
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

    type_uuids = dict(position=0x0005,
                      time=0x001A,
                      encoder=0x001B,
                      )
    y_scale = dict(position=1e4,
                   time=1e5,
                   encoder=1
                   )

    _uuid_to_type = dict((v, k) for k, v in type_uuids.items())

    @property
    def name(self):
        return ctypes.string_at(self._name).decode('latin-1')

    @name.setter
    def name(self, name):
        buf = ctypes.create_string_buffer(len(name) + 1)
        buf.value = name.encode('latin-1') + b'\0'
        assert len(buf) < len(self._name)
        ctypes.memmove(self._name, buf, len(buf))

    @property
    def wizard_params(self):
        return list(self._wizard_params)

    @property
    def x_type(self):
        return self._uuid_to_type[self.x_dim_uuid]

    @property
    def y_type(self):
        return self._uuid_to_type[self.y_dim_uuid]

    @property
    def packed_block_size(self):
        # original calculation:
        # [(info[1] * 4) << 16] | [(info[0] << 16) >> 16]
        return ((self.num_setpoints << 18) | self._size_) & 0xffffffff

    def get_info(self):
        for field_name, type_ in self._fields_:
            field_name = field_name.lstrip('_')
            yield (field_name, type_, getattr(self, field_name))

    def to_uint32s(self):
        arr = (ctypes.c_uint32 * math.ceil(self._size_ / 4))()
        ctypes.memmove(arr, ctypes.pointer(self), self._size_)
        return arr

    @staticmethod
    def create(curve_index, name, x_length, num_setpoints, x_type='time',
               y_type='position', wizard_type=None, wizard_params=None):
        curve_struct = CurveInfoStruct()
        curve_struct.data_offset = curve_struct._size_

        type_uuids = CurveInfoStruct.type_uuids
        assert x_type in type_uuids
        assert y_type in type_uuids

        if x_type == 'time' and y_type == 'position':
            curve_struct.object_type = 0x0003
        elif x_type == 'position' and y_type == 'encoder':
            curve_struct.object_type = 0x0103
        else:
            raise ValueError('Unsupported x/y -> object type')

        if x_type == 'time':
            # for seconds, x_length is in units of 10us
            x_length = float(x_length) * 1e5

        x_length = int(x_length)
        assert x_length > 0

        curve_struct.num_setpoints = num_setpoints
        curve_struct.data_type_size = 4  # 4-byte setpoints
        curve_struct.name = name
        curve_struct.curve_id = curve_index
        curve_struct.x_length = x_length
        curve_struct.x_dim_uuid = type_uuids[x_type]
        curve_struct.y_dim_uuid = type_uuids[y_type]

        if wizard_type is None:
            wizard_type = 0
            wizard_params = [0] * 7

        curve_struct.wizard_type = wizard_type
        curve_struct._wizard_params[:] = wizard_params
        return curve_struct


assert ctypes.sizeof(CurveInfoStruct) == CurveInfoStruct._size_


def unpack_curve_to_bytes(info):
    if len(info) == 18:
        return b''.join(struct.pack('<i', d) for d in info)
    elif len(info) == CurveInfoStruct._size_:
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

    def _run(self, initial_state, transition_list, state_info,
             write_data=None):
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
                if write_data:
                    self.value_out = write_data[cur_state_info.data].popleft()
                self.control_word = cur_state_info.control_word

        logger.debug('Final state %s %s', state, cur_state_info)
        logger.debug('Data:')
        for key, d in sorted(data.items()):
            logger.debug('\t%s = %r', key, d)
        return (state, data)

    def read_curve(self, curve_index):
        '''Read a curve from the drive's RAM

        Parameters
        ----------
        curve_index : int

        Raises
        ------
        RuntimeError
            Invalid index specified (or other error reading)

        Returns
        -------
        curve_info : dict
            Curve data dictionary with keys ('info', 'data')

        Note
        ----
        This is a direct translation of the PLC code, but it can be done more
        intelligently:

        There are 3 main stages:
        [1] start reading curve id
            command word: 0x6001

        [2] read curve info blocks
            command words: 0x6102, 0x6103
            Upper status word 0x04 means to toggle between the states, 0 means
            continue to [3]

        [3] read curve data blocks (similar to [2])
            command words: 0x6204, 0x6205
            Upper status word 0x04 means to toggle between the states, 0 done

        TODO: Once this is implemented in the sequencer, there should be better
        error-handling between states, and better error reporting to the user
        '''
        transitions = [Transition(40,  41, 0x0001),
                       Transition(40,  90, 0xD401),  # invalid curve id
                       Transition(41,  42, 0x0402),
                       Transition(41,  43, 0x0002),
                       Transition(42,  41, 0x0403),
                       Transition(42,  43, 0x0003),
                       Transition(43,  44, 0x0404),
                       Transition(43, 100, 0x0004),
                       Transition(44,  43, 0x0405),
                       Transition(44, 100, 0x0005),
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

        self._start_command()

        self.index_out = curve_index
        self.control_word = 0x6001
        final_state, data = self._run(40, transitions, state_info)
        if final_state != 100:
            raise RuntimeError('invalid final state?')
        return data

    def _start_command(self):
        self.index_out = 0
        self.value_out = 0
        self.control_word = 0xF

        # timeout, yadda yadda
        while self.status_in != 0xF:
            time.sleep(0.05)
            logger.debug('Initializing (status=%x)', self.status_in)

    def write_curve(self, curve_index, name, x_length, setpoints,
                    x_type='time', y_type='position', wizard_type=None,
                    wizard_params=None, curve_struct=None):
        '''Write a curve to the drive's RAM

        Parameters
        ----------
        curve_index : int
        name : str
        x_length : float
            Units depend on x_type:
            'time' -> seconds
            'encoder' -> encoder counts (integer)
        setpoints : list of float or int
            Units depend on y_type:
            'position' -> millimeters
            'time' -> seconds
            'encoder' -> encoder counts (integer)
        x_type : str, optional
            One of {'position', 'time', 'encoder'}
            Defaults to 'time'
        y_type : str, optional
            One of {'position', 'time', 'encoder'}
            Defaults to 'position'
        wizard_type : int, optional
            Normally should not be set, only if duplicating a curve that was
            generated in the linmot talk software
        wizard_params : list of 7 ints, optional
            Normally should not be set, only if duplicating a curve that was
            generated in the linmot talk software
        curve_struct : CurveInfoStruct, optional
            Ignore the above parameters and specify your own CurveInfoStruct

        Raises
        ------
        RuntimeError
            Invalid index specified (or other error reading)

        Note
        ----
        This is a direct translation of the PLC code, but it can be done more
        intelligently:

        There are 3 main stages:
        [1] start adding curve
            command word: 0x5001

        [2] write curve info blocks
            command words: 0x5102, 0x5103
            Upper status word 0x04 means to toggle between the states, 0 means
            continue to [3]

        [3] write curve data blocks (similar to [2])
            command words: 0x5204, 0x5205
            Upper status word 0x04 means to toggle between the states, 0 done

        TODO: Once this is implemented in the sequencer, there should be better
        error-handling between states, and better error reporting to the user
        '''

        if curve_struct is None:
            curve_struct = CurveInfoStruct.create(curve_index, name,
                                                  x_length=x_length,
                                                  num_setpoints=len(setpoints),
                                                  x_type=x_type, y_type=y_type,
                                                  wizard_type=wizard_type,
                                                  wizard_params=wizard_params)

        assert curve_struct.num_setpoints == len(setpoints)

        self._start_command()

        transitions = [Transition(20,  21, 0x0001),
                       Transition(20,  90, 0xD401),  # invalid curve id
                       Transition(21,  22, 0x0402),
                       Transition(21,  23, 0x0002),
                       Transition(22,  21, 0x0403),
                       Transition(22,  23, 0x0003),
                       Transition(23,  24, 0x0404),
                       Transition(23, 100, 0x0004),
                       Transition(24,  23, 0x0405),
                       Transition(24, 100, 0x0005),
                       ]
        state_info = {
            20:  State(0x5001, None),
            21:  State(0x5102, 'info'),
            22:  State(0x5103, 'info'),
            23:  State(0x5204, 'data'),
            24:  State(0x5205, 'data'),
            90:  State(0x0000, 'error'),
            100: State(0x0000, 'success'),
        }

        scale = CurveInfoStruct.y_scale[y_type]
        write_data = {'info': deque(curve_struct.to_uint32s()),
                      'data': deque(int(float(setpoint) * scale)
                                    for setpoint in setpoints),
                      'success': deque([0]),
                      'error': deque([0]),
                      }

        print(write_data)
        logger.debug('Writing new curve %d', curve_struct.curve_id)
        for field_name, type_, value in curve_struct.get_info():
            logger.debug('\t%s = %r', field_name, value)

        self.index_out = curve_index
        self.value_out = curve_struct.packed_block_size
        self.control_word = 0x5001

        logger.debug('Index %d, block size=%x, control word=%x',
                     self.index_out, self.value_out, self.control_word)

        final_state, data = self._run(20, transitions, state_info,
                                      write_data=write_data)
        if final_state != 100:
            raise RuntimeError('invalid final state?')
        return data


def _test_copy(acc, new_curve_id=7):
    from results import curve_1
    info = parse_curve_info(curve_1['info'])

    setpoints = [float(setpoint) * 1e-4 for setpoint in curve_1['data']]
    acc.write_curve(new_curve_id,
                    '{}(copy-{})'.format(info.name, new_curve_id),
                    info.x_length * 1e-5, setpoints,
                    x_type=info.x_type, y_type=info.y_type,
                    wizard_type=info.wizard_type,
                    wizard_params=info.wizard_params)


def _test_read(acc, curves):
    data = {}
    for cid in curves:
        data[cid] = acc.read_curve(cid)

    for cid in curves:
        curve = data[cid]
        info = parse_curve_info(curve['info'])
        print('Curve {!r}'.format(info.name))
        for field_name, type_, value in info.get_info():
            print('\t{} = {!r}'.format(field_name, value))
        print('\t{!r}'.format(curve['data']))

    return data


def test(prefix):
    acc = CurveAccess(prefix)

    return _test_copy(acc)
    return _test_read(acc, (10, ))


if __name__ == '__main__':

    try:
        prefix = sys.argv[1]
    except IndexError:
        prefix = 'WIRE:B34:97:CONFIGMODULE:'

    logging.basicConfig()
    logger.setLevel(logging.DEBUG)

    # use ipython -i linmot.py
    # to get access to these
    data = test(prefix=prefix)
