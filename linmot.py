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


class PackableStructure(ctypes.LittleEndianStructure):
    '''Base class which makes it easy to pack any of these structures into the
    32-bit mc parameter uints or info blocks etc
    '''
    def to_uint32s(self, pad=None, pad_value=0):
        arr = (ctypes.c_uint32 * math.ceil(self._size_ / 4))()
        ctypes.memmove(arr, ctypes.pointer(self), self._size_)

        if pad and len(arr) < pad:
            pad_count = pad - len(arr)
            return list(arr) + [pad_value] * pad_count
        return arr

    def get_info(self):
        for field_name, type_ in self._fields_:
            field_name = field_name.lstrip('_')
            yield (field_name, type_, getattr(self, field_name))


class TimeCurveScaled(PackableStructure):
    '''Time curve with adjustable offset, time scale, and amplitude scale'''
    command_header = 0x0440
    _size_ = 10
    _pack_ = 1
    _fields_ = [('curve_id', ctypes.c_uint16),
                ('curve_offset', ctypes.c_int32),
                ('time_scale', ctypes.c_uint16),
                ('amplitude_scale', ctypes.c_int16),
                ]

    @staticmethod
    def create(curve_id, curve_offset, time_scale, amplitude_scale):
        st = TimeCurveScaled()
        st.curve_id = curve_id
        st.curve_offset = int(curve_offset * 1e4)

        # time scale is normalized here, but linmot expects a percentage
        # and units are of 0.01 % (so scale = 100 * 100)
        assert 0 < time_scale <= 2.0
        st.time_scale = int(time_scale * 10000.0)

        # similarly, amplitude scale is in units of 0.1 %
        assert -20.0 <= amplitude_scale <= 20.0
        st.amplitude_scale = int(amplitude_scale * 1000.0)
        return st


assert ctypes.sizeof(TimeCurveScaled) == TimeCurveScaled._size_


class TimeCurveTotal(PackableStructure):
    '''Time curve with adjustable offset, total time, and amplitude scale'''
    command_header = 0x0450
    _size_ = 12
    _pack_ = 1
    _fields_ = [('curve_id', ctypes.c_uint16),
                ('curve_offset', ctypes.c_int32),
                ('time', ctypes.c_int32),
                ('amplitude_scale', ctypes.c_int16),
                ]

    @staticmethod
    def create(curve_id, curve_offset, time_sec, amplitude_scale):
        st = TimeCurveTotal()
        st.curve_id = curve_id
        st.curve_offset = int(curve_offset * 1e4)
        st.time = int(time_sec * 1e5)
        # amplitude scale is normalized, but expected to be in units of 0.1 %
        assert -20.0 <= amplitude_scale <= 20.0
        st.amplitude_scale = int(amplitude_scale * 1000.0)
        return st


assert ctypes.sizeof(TimeCurveTotal) == TimeCurveTotal._size_


class CurveInfoStruct(PackableStructure):
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
    ctypes.memmove(ctypes.addressof(st), info,
                   min((len(info), ctypes.sizeof(st))))
    return st


class Curve(object):
    def __init__(self, info, setpoints):
        self.info = parse_curve_info(info)
        self.setpoints = setpoints


class CurveStates:  # py3k enum, really
    ST_INIT = 20

    # Info block transfer states
    ST_INFO1 = 21
    ST_INFO2 = 22

    # Data block states
    ST_DATA1 = 23
    ST_DATA2 = 24

    # Final states
    ST_ERROR = 90
    ST_DONE = 100

    final_states = (ST_DONE, )
    error_states = (ST_ERROR, )


class LinmotControl(object):
    CMD_NOOP = 0x0000
    CMD_PREPARE_CONFIG_MODULE = 0x000F
    CMD_SAVE_TO_FLASH = 0x4001
    CMD_DELETE_ALL_CURVES = 0x4101
    CMD_RESTART_DRIVE = 0x3000
    CMD_STOP_MC = 0x3500
    CMD_START_MC = 0x3600

    def __init__(self, prefix, config_prefix='CONFIGMODULE:',
                 output_prefix='DO:', input_prefix='DI:'):
        config_prefix = prefix + config_prefix
        input_prefix = prefix + input_prefix
        output_prefix = prefix + output_prefix
        self._index_in = epics.PV(config_prefix + 'INDEXIN',
                                  auto_monitor=False)
        self._status_in = epics.PV(config_prefix + 'STATUSWORD',
                                   auto_monitor=False)
        self._value_in = epics.PV(config_prefix + 'VALUEIN',
                                  auto_monitor=False)

        self._control_word = epics.PV(config_prefix + 'CONTROLWORD',
                                      auto_monitor=False)
        self._index_out = epics.PV(config_prefix + 'INDEXOUT',
                                   auto_monitor=False)
        self._value_out = epics.PV(config_prefix + 'VALUEOUT',
                                   auto_monitor=False)

        self._command_pars = [epics.PV('{}MOTIONCOMMANDPAR{}'
                                       ''.format(output_prefix, i),
                                       auto_monitor=False)
                              for i in range(1, 6)]
        self._command_hdr = epics.PV('{}MOTIONCOMMANDHEADER'
                                     ''.format(output_prefix),
                                     auto_monitor=False)
        self._state_var = epics.PV('{}STATEVAR'.format(input_prefix),
                                   auto_monitor=False)

        self._pvs = [self._index_in, self._status_in, self._value_in,
                     self._control_word, self._index_out, self._value_out,
                     self._command_hdr, self._state_var,
                     ]

        self._pvs.extend(self._command_pars)

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

    @property
    def state_var(self):
        return self._state_var.get()

    @property
    def command_parameters(self):
        return [par.get() for par in self._command_pars]

    @command_parameters.setter
    def command_parameters(self, values):
        for par, value in zip(self._command_pars, values):
            par.put(value)

    @property
    def command_header(self):
        return self._command_hdr.get()

    @command_header.setter
    def command_header(self, value):
        self._command_hdr.put(value)

    def write_and_wait(self, control_word, index_out, value_out,
                       expected_status):
        self.index_out = index_out
        self.value_out = value_out
        self.control_word = control_word

        # timeout, yadda yadda
        while self.status_in != expected_status:
            time.sleep(0.05)
            logger.debug('Waiting for status=%x (currently=%x)',
                         expected_status, self.status_in)


class CurveAccess(LinmotControl):
    def __init__(self, prefix, **kwargs):
        super().__init__(prefix=prefix, **kwargs)

        self.transitions = [
            Transition(CurveStates.ST_INIT,  CurveStates.ST_INFO1, 0x0001),
            Transition(CurveStates.ST_INFO1, CurveStates.ST_INFO2, 0x0402),
            Transition(CurveStates.ST_INFO1, CurveStates.ST_DATA1, 0x0002),
            Transition(CurveStates.ST_INFO2, CurveStates.ST_INFO1, 0x0403),
            Transition(CurveStates.ST_INFO2, CurveStates.ST_DATA1, 0x0003),
            Transition(CurveStates.ST_DATA1, CurveStates.ST_DATA2, 0x0404),
            Transition(CurveStates.ST_DATA1, CurveStates.ST_DONE,  0x0004),
            Transition(CurveStates.ST_DATA2, CurveStates.ST_DATA1, 0x0405),
            Transition(CurveStates.ST_DATA2, CurveStates.ST_DONE,  0x0005),

            # Error - invalid curve id
            Transition(CurveStates.ST_INIT,  CurveStates.ST_ERROR, 0xD401),
            # Error - size error (?)
            Transition(CurveStates.ST_INIT,  CurveStates.ST_ERROR, 0xD101),
            ]

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

        States are also pretty much identical between modify/add/get, only
        differing in the output values (TODO simplify)
        '''
        state_info = {
            CurveStates.ST_INIT:  State(0x6001, None),
            CurveStates.ST_INFO1: State(0x6102, 'info'),
            CurveStates.ST_INFO2: State(0x6103, 'info'),
            CurveStates.ST_DATA1: State(0x6204, 'data'),
            CurveStates.ST_DATA2: State(0x6205, 'data'),
            CurveStates.ST_ERROR: State(0x0000, 'error'),
            CurveStates.ST_DONE:  State(0x0000, 'success'),
        }

        self._start_sequence(0x6001, curve_index)
        final_state, data = self._run(CurveStates.ST_INIT, self.transitions,
                                      state_info)
        if final_state not in CurveStates.final_states:
            raise RuntimeError('invalid final state?')
        return data

    def _start_sequence(self, control_word, index_out, value_out=0):
        self.write_and_wait(control_word=self.CMD_PREPARE_CONFIG_MODULE,
                            index_out=0, value_out=0, expected_status=0xF)

        self.index_out = index_out
        self.value_out = value_out
        self.control_word = control_word

    def write_curve(self, curve_index, name, x_length, setpoints,
                    x_type='time', y_type='position', wizard_type=None,
                    wizard_params=None, curve_struct=None, modify=False):
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
        '''

        if curve_struct is None:
            curve_struct = CurveInfoStruct.create(curve_index, name,
                                                  x_length=x_length,
                                                  num_setpoints=len(setpoints),
                                                  x_type=x_type, y_type=y_type,
                                                  wizard_type=wizard_type,
                                                  wizard_params=wizard_params)

        assert curve_struct.num_setpoints == len(setpoints)

        if modify:
            state_info = {
                CurveStates.ST_INIT:  State(0x5301, None),
                CurveStates.ST_INFO1: State(0x5402, 'info'),
                CurveStates.ST_INFO2: State(0x5403, 'info'),
                CurveStates.ST_DATA1: State(0x5504, 'data'),
                CurveStates.ST_DATA2: State(0x5505, 'data'),
                CurveStates.ST_ERROR: State(0x0000, 'error'),
                CurveStates.ST_DONE:  State(0x0000, 'success'),
            }
        else:
            state_info = {
                CurveStates.ST_INIT:  State(0x5001, None),
                CurveStates.ST_INFO1: State(0x5102, 'info'),
                CurveStates.ST_INFO2: State(0x5103, 'info'),
                CurveStates.ST_DATA1: State(0x5204, 'data'),
                CurveStates.ST_DATA2: State(0x5205, 'data'),
                CurveStates.ST_ERROR: State(0x0000, 'error'),
                CurveStates.ST_DONE:  State(0x0000, 'success'),
            }

        scale = CurveInfoStruct.y_scale[y_type]
        write_data = {'info': deque(curve_struct.to_uint32s()),
                      'data': deque(int(float(setpoint) * scale)
                                    for setpoint in setpoints),
                      'success': deque([0]),
                      'error': deque([0]),
                      }

        if modify:
            logger.debug('Modifying curve %d', curve_struct.curve_id)
            control_word = 0x5301
        else:
            logger.debug('Writing new curve %d', curve_struct.curve_id)
            control_word = 0x5001
        self._start_sequence(control_word, curve_index,
                             curve_struct.packed_block_size)

        for field_name, type_, value in curve_struct.get_info():
            logger.debug('\t%s = %r', field_name, value)

        logger.debug('Index %d, block size=%x, control word=%x',
                     self.index_out, self.value_out, self.control_word)

        final_state, data = self._run(CurveStates.ST_INIT, self.transitions,
                                      state_info, write_data=write_data)
        if final_state not in CurveStates.final_states:
            raise RuntimeError('invalid final state?')
        return data

    def write_to_flash(self):
        sequence = [('Preparing config module', self.CMD_PREPARE_CONFIG_MODULE,
                     self.CMD_PREPARE_CONFIG_MODULE),
                    ('Stopping MC module', self.CMD_STOP_MC, 0),
                    ('Saving to flash', self.CMD_SAVE_TO_FLASH, 1),
                    ('Starting MC module', self.CMD_START_MC, 0),
                    ]
        for msg, control, status in sequence:
            logger.info('%s - (%x -> %x)', msg, control, status)
            self.write_and_wait(control_word=control, index_out=0, value_out=0,
                                expected_status=status)

    def next_sequence_id(self):
        '''Motion commands require sequential sequence identifiers'''
        old_count = (self.state_var & 0xF)
        return (old_count + 1) % 16

    def run_curve(self, curve_id, offset=0, time_scale=None,
                  amplitude_scale=1.0, time_sec=None, wait=True):
        '''Run a curve

        Parameters
        ----------
        offset : float, optional
            Y position offset of curve [mm]
        time_scale : float, optional
            In (0, 2.00] scale of time (x) axis
            Cannot be specified with time_sec (below)
        time_sec : float, optional
            Alternatively, specify the total time of the curve
            Cannot be specified with time_scale
        amplitude_scale : float, optional
            In [-20.0, 20.0] scale of position (y) axis
        wait : bool, optional
            Block until the curve has completed
        '''
        if time_scale is not None and time_sec is not None:
            raise ValueError('Cannot specify both time_scale and time_sec')
        elif (time_scale is None and time_sec is None) or time_scale:
            if not time_scale:
                time_scale = 1.0
            cmd = TimeCurveScaled.create(curve_id, offset, time_scale,
                                         amplitude_scale)
        else:
            # specify time
            cmd = TimeCurveTotal.create(curve_id, offset, time_sec,
                                        amplitude_scale)

        self.command_parameters = cmd.to_uint32s(pad=5)

        sequence_id = self.next_sequence_id()
        self.command_header = cmd.command_header + sequence_id
        logger.debug('Run curve header 0x%x', self.command_header)
        logger.debug('Parameters %s', list(self.command_parameters))

    def invalidate_curve(self, curve_id):
        sequence_id = self.next_sequence_id()
        # the manual says 0x0FFF but I think it's wrong
        self.command_parameters = [0xFFFF0000 | (curve_id & 0xFFFF),
                                   0, 0, 0, 0]
        self.command_header = 0x0500 + sequence_id


def _test_copy(acc, new_curve_id=8, modify=False):
    from results import curve_1
    info = parse_curve_info(curve_1['info'])

    setpoints = [float(setpoint * 2) / info.y_scale['position']
                 for setpoint in curve_1['data']]

    acc.write_curve(new_curve_id,
                    '{}(copy-{})'.format(info.name, new_curve_id),
                    info.x_length * 1e-5, setpoints,
                    x_type=info.x_type, y_type=info.y_type,
                    wizard_type=info.wizard_type,
                    wizard_params=info.wizard_params,
                    modify=modify)


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


def _test_wirescan(acc, speed=4, max_speed=10, dt=0.01, smoothing=3,
                   curve_id=10):
    acc.invalidate_curve(curve_id)

    import wirescan
    vel_profile = wirescan.build_velocity_profile(speed, max_speed, dt=dt)
    x_time, pos_profile = wirescan.build_position_profile(dt, vel_profile,
                                                          smoothing=smoothing)

    import numpy as np
    vel_profile = -np.array(vel_profile)
    pos_profile = -pos_profile
    if True:
        import matplotlib
        matplotlib.use('agg')
        import matplotlib.pyplot as plt
        print(len(pos_profile), len(vel_profile))
        plt.plot(x_time, pos_profile)
        plt.title('speed={} max_speed={} dt={} smoothing={}\nnum_points={}'
                  ''.format(speed, max_speed, dt, smoothing, len(x_time)))

        ax2 = plt.twinx()
        ax2.plot(x_time, vel_profile, '--r')
        plt.savefig('trajectory.png')
        plt.show()

    # things to note:
    #   * ensure the profile is inverted
    #   * ensure x_length is not dt but the full x length
    acc.write_curve(curve_id, name='Wirescan-{}'.format(curve_id),
                    x_length=dt * len(pos_profile),
                    setpoints=pos_profile, modify=False)

    # this works, but for now i prefer to verify it on linmot talk and then
    # run it...
    # acc.run_curve(curve_id)
    return pos_profile


def test(prefix):
    acc = CurveAccess(prefix)

    cid = 10
    # _test_copy(acc, cid, modify=False)
    # data = _test_read(acc, (cid, ))
    # data = {}
    data = _test_wirescan(acc, curve_id=cid)
    return acc, data


if __name__ == '__main__':

    try:
        prefix = sys.argv[1]
    except IndexError:
        # 97: is A, 97-1: is B
        prefix = 'WIRE:B34:97-1:'

    logging.basicConfig()
    logger.setLevel(logging.DEBUG)

    # use ipython -i linmot.py
    # to get access to these
    acc, data = test(prefix=prefix)
