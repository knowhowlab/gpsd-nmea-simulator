import collections
import datetime
import math
import operator
import random
import sys
import threading
from thread import *
import time
import socket

import constants
from geographiclib.geodesic import Geodesic

sys.stdout = sys.stderr

fix_types = collections.OrderedDict()
fix_types[constants.INVALID_FIX] = '0'
fix_types[constants.SPS_FIX] = '1'
fix_types['DGPS_FIX'] = '2'
fix_types['PPS_FIX'] = '3'
fix_types['RTK_FIX'] = '4'
fix_types['FLOAT_RTK_FIX'] = '5'
fix_types['DEAD_RECKONING_FIX'] = '6'
fix_types['MANUAL_INPUT_FIX'] = '7'
fix_types['SIMULATED_FIX'] = '8'

solution_modes = collections.OrderedDict()
solution_modes[''] = ''
solution_modes[constants.AUTONOMOUS_SOLUTION] = 'A'
solution_modes['DIFFERENTIAL_SOLUTION'] = 'D'
solution_modes['ESTIMATED_SOLUTION'] = 'E'
solution_modes[constants.INVALID_SOLUTION] = 'N'
solution_modes['SIMULATOR_SOLUTION'] = 'S'


class TimeZone(datetime.tzinfo):
    ''' Generic time zone class that implements the Python tzinfo interface
    Provides non-DST aware offsets from UTC in seconds (e.g. from time.timezone)
    '''

    def __init__(self, utcdeltasec=time.timezone):
        self.utcdeltasec = utcdeltasec

    def utcoffset(self, date_time):
        return datetime.timedelta(seconds=self.utcdeltasec) + self.dst(date_time)

    def dst(self, date_time):
        return datetime.timedelta(0)

    def tzname(self, date_time):
        hh = int(self.utcdeltasec / 3600)
        mm = int(self.utcdeltasec / 60 - hh * 60)
        ss = int(self.utcdeltasec - mm * 60)
        return 'GMT +%02d:%02d:%02d' % (hh, mm, ss)


tz_local = TimeZone(time.timezone)


class ModelSatellite(object):
    ''' Model class for a GNSS satellite
    '''

    def __init__(self, prn, elevation=0, azimuth=0, snr=40):
        self.prn = prn
        self.elevation = elevation
        self.azimuth = azimuth
        self.snr = snr


class ModelGnssReceiver(object):
    ''' Model class for a GNSS receiver
    Takes in a model GNSS parameters and outputs the requested NMEA sentences.
    The model has the capability to project forward 2-D coordinates based on
    a speed and heading over a given time.
    '''

    __GSA_SV_LIMIT = 12  # Maximum number of satellites per GSA message
    __GSV_SV_LIMIT = 4  # Maximum number of satellites per GSV message

    # Constants for RMC and GLL validity
    __VALID_FIX = 'A'
    __INVALID_FIX = 'V'

    # Constants for the NMEA 2.3 specified 'mode'
    __AUTOMATIC_MODE = 'A'
    __MANUAL_MODE = 'M'
    __SOLUTION_NA = '1'
    __SOLUTION_2D = '2'
    __SOLUTION_3D = '3'

    __KNOTS_PER_KPH = 1.852

    def __recalculate(self):
        ''' Recalculate and fix internal state data for the GNSS instance.
        Should be executed after external modification of parameters and prior to doing any calculations.
        '''
        self.__visible_prns = []
        for satellite in self.satellites:
            # Fix elevation wrap around (overhead and opposite side of earth)
            if satellite.elevation > 90:
                satellite.elevation = 180 - satellite.elevation
                satellite.azimuth += 180
            elif satellite.elevation < -90:
                satellite.elevation = -180 - satellite.elevation
                satellite.azimuth += 180

            # Fix azimuth wrap around
            satellite.azimuth %= 360

            # Fix SNR going over or under limits
            if satellite.snr < 0:
                satellite.snr = 0
            elif satellite.snr > 99:
                satellite.snr = 99

            if satellite.elevation > 0:
                # If above horizon, treat as visible
                self.__visible_prns.append(satellite.prn)

        # Optional NMEA 2.3 solution 'mode' has priority if present when
        # determining validity
        if self.solution == constants.INVALID_SOLUTION:
            self.fix = constants.INVALID_FIX

        # For real fixes correct for number of satellites
        if self.fix != 'DEAD_RECKONING_FIX' and self.fix != 'MANUAL_INPUT_FIX' and self.fix != 'SIMULATED_FIX':
            # Cannot have GNSS time without satellites
            if self.num_sats == 0 and not self.has_rtc:
                self.date_time = None

            # Cannot have a fix if too few satellites
            if self.num_sats < 4:
                if self.manual_2d and self.num_sats == 3:
                    # 3 satellites sufficient for 2-D fix if forced
                    self.altitude = None
                else:
                    self.fix = constants.INVALID_FIX

        # Force blank fields if there is no fix
        if self.fix == constants.INVALID_FIX:
            self.__validity = self.__INVALID_FIX
            self.__mode = self.__SOLUTION_NA
        else:
            self.__validity = self.__VALID_FIX
            self.__mode = self.__SOLUTION_3D

        # Force blanks for 2-D fix
        if self.altitude is None:
            if self.__mode != self.__SOLUTION_NA:
                self.__mode = self.__SOLUTION_2D

        # Convert decimal latitude to NMEA friendly form
        if self.lat is not None:
            self.__lat_sign = 'S' if self.lat < 0 else 'N'
            self.__lat_degrees = int(abs(self.lat))
            self.__lat_minutes = (abs(self.lat) - self.__lat_degrees) * 60
            # Take care of weird rounding
            if round(self.__lat_minutes, self.horizontal_dp) >= 60:
                self.__lat_degrees += 1
                self.__lat_minutes = 0

        # Convert decimal longitude to NMEA friendly form
        if self.lon is not None:
            self.__lon_sign = 'W' if self.lon < 0 else 'E'
            self.__lon_degrees = int(abs(self.lon))
            self.__lon_minutes = (abs(self.lon) - self.__lon_degrees) * 60
            # Take care of weird rounding
            if round(self.__lon_minutes, self.horizontal_dp) >= 60:
                self.__lon_degrees += 1
                self.__lon_minutes = 0

        # Convert decimal magnetic variation to NMEA friendly form
        if self.mag_var is not None:
            self.__mag_sign = 'W' if self.mag_var < 0 else 'E'
            self.__mag_value = abs(self.mag_var)

        # Convert metric speed to imperial form
        if self.kph is not None:
            self.__knots = self.kph / self.__KNOTS_PER_KPH

        # Fix heading wrap around
        if self.heading is not None:
            self.heading %= 360

        # Fix magnetic heading wrap around
        if self.mag_heading is not None:
            self.mag_heading %= 360

        # Generate string specifications for various fields
        self.__vertical_spec = '%%.%df' % self.vertical_dp
        self.__angle_spec = '%%.%df' % self.angle_dp
        self.__speed_spec = '%%.%df' % self.speed_dp

        if self.time_dp > 0:
            self.__time_spec = ('%%0%d' % (self.time_dp + 3)
                                ) + ('.%df' % self.time_dp)
        else:
            self.__time_spec = '%02d'

        if self.horizontal_dp > 0:
            self.__horizontal_spec = ('%%0%d' % (
                self.horizontal_dp + 3)) + ('.%df' % self.horizontal_dp)
        else:
            self.__horizontal_spec = '%02d'

    def __format_sentence(self, data):
        ''' Format an NMEA sentence, pre-pending with '$' and post-pending checksum.
        '''
        sum = 0
        for ch in data:
            sum ^= ord(ch)
        return '$' + data + '*%02X' % sum

    def __nmea_lat_lon(self):
        ''' Generate an NMEA lat/lon string (omits final trailing ',').
        '''
        data = ''
        if self.lat is not None:
            data += ('%02d' % self.__lat_degrees) + (self.__horizontal_spec %
                                                     self.__lat_minutes) + ',' + self.__lat_sign + ','
        else:
            data += ',,'

        if self.lon is not None:
            data += ('%03d' % self.__lon_degrees) + (self.__horizontal_spec %
                                                     self.__lon_minutes) + ',' + self.__lon_sign
        else:
            data += ','
        return data

    def __nmea_time(self):
        ''' Generate an NMEA time string (omits final trailing ',').
        '''
        if self.date_time is not None:
            ts = self.date_time.utctimetuple()
            return ('%02d' % ts.tm_hour) + ('%02d' % ts.tm_min) + (
            self.__time_spec % (ts.tm_sec + self.date_time.microsecond * 1e-6))
        else:
            return ''

    def __gga(self):
        ''' Generate an NMEA GGA sentence.
        '''
        data = ''

        data += self.__nmea_time() + ','

        data += self.__nmea_lat_lon() + ','

        data += fix_types[self.fix] + ',' + ('%02d' % self.num_sats) + ','

        if self.hdop is not None:
            data += ('%.1f' % self.hdop)
        data += ','

        if self.altitude is not None:
            data += (self.__vertical_spec % self.altitude)
        data += ',M,'

        if self.geoid_sep is not None:
            data += (self.__vertical_spec % self.geoid_sep)
        data += ',M,'

        if self.last_dgps is not None:
            data += (self.__time_spec % self.last_dgps)
        data += ','

        if self.dgps_station is not None:
            data += ('%04d' % self.dgps_station)

        return [self.__format_sentence(self._prefix + 'GGA,' + data)]

    def __rmc(self):
        ''' Generate an NMEA RMC sentence.
        '''
        data = ''

        data += self.__nmea_time() + ','

        data += self.__validity + ','

        data += self.__nmea_lat_lon() + ','

        if self.kph is not None:
            data += (self.__speed_spec % self.__knots)
        data += ','

        if self.heading is not None:
            data += (self.__angle_spec % self.heading)
        data += ','

        if self.date_time is not None:
            ts = self.date_time.utctimetuple()
            data += ('%02d' % ts.tm_mday) + ('%02d' %
                                             ts.tm_mon) + ('%02d' % (ts.tm_year % 100))
        data += ','

        if self.mag_var is not None:
            data += (self.__angle_spec % self.__mag_value) + \
                    ',' + self.__mag_sign
        else:
            data += ','

        if self.solution is not None:
            data += ',' + solution_modes[self.solution]

        return [self.__format_sentence(self._prefix + 'RMC,' + data)]

    def __gsa(self):
        ''' Generate an NMEA GSA sentence.
        '''
        data = (self.__MANUAL_MODE if self.manual_2d else self.__AUTOMATIC_MODE) + ','

        data += self.__mode + ','

        if self.num_sats >= self.__GSA_SV_LIMIT:
            for i in xrange(self.__GSA_SV_LIMIT):
                data += ('%d' % self.__visible_prns[i]) + ','
        else:
            for prn in self.__visible_prns:
                data += ('%d' % prn) + ','
            data += ',' * (self.__GSA_SV_LIMIT - self.num_sats)

        if self.pdop is not None:
            data += ('%.1f' % self.pdop)
        data += ','

        if self.hdop is not None:
            data += ('%.1f' % self.hdop)
        data += ','

        if self.vdop is not None:
            data += ('%.1f' % self.vdop)

        return [self.__format_sentence(self._prefix + 'GSA,' + data)]

    def __gsv(self):
        ''' Generate a sequence of NMEA GSV sentences.
        '''
        if self.num_sats == 0:
            return []

        # Work out how many GSV sentences are required to show all satellites
        messages = [''] * ((self.num_sats + self.__GSV_SV_LIMIT -
                            1) / self.__GSV_SV_LIMIT)
        prn_i = 0

        # Iterate through each block of satellites
        for i in xrange(len(messages)):
            data = ''
            data += ('%d' % len(messages)) + ','
            data += ('%d' % (i + 1)) + ','
            data += ('%d' % self.num_sats) + ','

            # Iterate through each satellite in the block
            for j in xrange(self.__GSV_SV_LIMIT):
                if prn_i < self.num_sats:
                    satellite = next((sat for sat in self.satellites if sat.prn == self.__visible_prns[prn_i]))
                    data += ('%d' % satellite.prn) + ','
                    data += ('%d' % int(satellite.elevation)) + ','
                    data += ('%d' % int(satellite.azimuth)) + ','
                    data += ('%d' % int(satellite.snr))
                    prn_i += 1
                else:
                    data += ',,,'

                # Final satellite in block does not have any fields after it so
                # don't add a ','
                if j != self.__GSV_SV_LIMIT - 1:
                    data += ','

            # Generate the GSV sentence for this block
            messages[i] = self.__format_sentence(self._prefix + 'GSV,' + data)

        return messages

    def __vtg(self):
        ''' Generate an NMEA VTG sentence.
        '''
        data = ''

        if self.heading is not None:
            data += (self.__angle_spec % self.heading)
        data += ',T,'

        if self.mag_heading is not None:
            data += (self.__angle_spec % self.mag_heading)
        data += ',M,'

        if self.kph is not None:
            data += (self.__speed_spec % self.__knots) + ',N,'
            data += (self.__speed_spec % self.kph) + ',K'
        else:
            data += ',N,,K'

        if self.solution is not None:
            data += ',' + solution_modes[self.solution]

        return [self.__format_sentence(self._prefix + 'VTG,' + data)]

    def __gll(self):
        ''' Generate an NMEA GLL sentence.
        '''
        data = ''

        data += self.__nmea_lat_lon() + ','

        data += self.__nmea_time() + ','

        data += self.__validity

        if self.solution is not None:
            data += ',' + solution_modes[self.solution]

        return [self.__format_sentence(self._prefix + 'GLL,' + data)]

    def __zda(self):
        ''' Generate an NMEA ZDA sentence.
        '''
        data = ''

        if self.date_time is None:
            return []

        data += self.__nmea_time() + ','

        ts = self.date_time.utctimetuple()
        data += ('%02d' % ts.tm_mday) + ',' + ('%02d' % ts.tm_mon) + \
                ',' + ('%04d' % (ts.tm_year % 10000)) + ','

        offset = self.date_time.utcoffset()
        if offset is not None:
            hh = int(offset.total_seconds() / 3600)
            mm = int(offset.total_seconds() / 60 - hh * 60)
            data += ('%02d' % hh) + ',' + ('%02d' % mm)
        else:
            data += ','

        return [self.__format_sentence(self._prefix + 'ZDA,' + data)]

    def __init__(self,
                 min_sv_number,
                 max_sv_number,
                 total_sv_limit,
                 output=('GGA', 'GLL', 'GSA', 'GSV', 'RMC', 'VTG', 'ZDA'),
                 solution=constants.AUTONOMOUS_SOLUTION,
                 fix=constants.SPS_FIX, manual_2d=False,
                 horizontal_dp=3,
                 vertical_dp=1,
                 speed_dp=1,
                 time_dp=3,
                 angle_dp=1,
                 date_time=0,
                 lat=0.0,
                 lon=0.0,
                 altitude=0.0,
                 geoid_sep=0.0,
                 kph=0.0,
                 heading=0.0,
                 mag_heading=None,
                 mag_var=0.0,
                 num_sats=12,
                 hdop=1.0,
                 vdop=1.0,
                 pdop=1.0,
                 last_dgps=None,
                 dgps_station=None,
                 has_rtc=False):
        ''' Initialise the GNSS instance with initial configuration.
        '''
        # Populate the sentence generation table

        self._prefix = 'GN'
        self.__min_sv_number = min_sv_number  # Minimum satellite prn
        self.__max_sv_number = max_sv_number  # Maximum satellite prn
        self.__total_sv_limit = total_sv_limit  # Maximum possible satellite constellation size

        self.__gen_nmea = {}
        self.__gen_nmea['GGA'] = self.__gga
        self.__gen_nmea['GSA'] = self.__gsa
        self.__gen_nmea['GSV'] = self.__gsv
        self.__gen_nmea['RMC'] = self.__rmc
        self.__gen_nmea['VTG'] = self.__vtg
        self.__gen_nmea['GLL'] = self.__gll
        self.__gen_nmea['ZDA'] = self.__zda

        # Record parameters
        self.solution = solution
        self._fix = fix
        self.manual_2d = manual_2d
        if (date_time == 0):
            self.date_time = datetime.datetime.now(tz_local)
        else:
            self.date_time = date_time
        self._lat = lat
        self._lon = lon
        self.horizontal_dp = horizontal_dp
        self.vertical_dp = vertical_dp
        self.speed_dp = speed_dp
        self.angle_dp = angle_dp
        self.time_dp = time_dp
        self._altitude = altitude
        self._geoid_sep = geoid_sep
        self._kph = kph
        self._heading = heading
        self._mag_heading = mag_heading
        self._mag_var = mag_var
        self._hdop = hdop
        self._vdop = vdop
        self._pdop = pdop
        self.last_dgps = last_dgps
        self.dgps_station = dgps_station
        self.output = output
        self.has_rtc = has_rtc

        # Create all dummy satellites with random conditions
        self.satellites = []
        for prn in xrange(self.__min_sv_number, self.__max_sv_number + 1):
            self.satellites.append(ModelSatellite(
                prn, azimuth=random.random() * 360, snr=30 + random.random() * 10))

        # Smart setter will configure satellites as appropriate
        self.num_sats = num_sats

        self.__recalculate()

    @property
    def lat(self):
        return self._lat

    @lat.setter
    def lat(self, new_lat):
        self._lat = new_lat

    @property
    def lon(self):
        return self._lon

    @lon.setter
    def lon(self, new_lon):
        self._lon = new_lon

    @property
    def altitude(self):
        return self._altitude

    @altitude.setter
    def altitude(self, new_altitude):
        self._altitude = new_altitude

    @property
    def geoid_sep(self):
        return self._geoid_sep

    @geoid_sep.setter
    def geoid_sep(self, new_geoid_sep):
        self._geoid_sep = new_geoid_sep

    @property
    def hdop(self):
        return self._hdop

    @hdop.setter
    def hdop(self, new_hdop):
        self._hdop = new_hdop

    @property
    def vdop(self):
        return self._vdop

    @vdop.setter
    def vdop(self, new_vdop):
        self._vdop = new_vdop

    @property
    def pdop(self):
        return self._pdop

    @pdop.setter
    def pdop(self, new_pdop):
        self._pdop = new_pdop

    @property
    def kph(self):
        return self._kph

    @kph.setter
    def kph(self, new_kph):
        self._kph = new_kph

    @property
    def heading(self):
        return self._heading

    @heading.setter
    def heading(self, new_heading):
        self._heading = new_heading

    @property
    def mag_heading(self):
        return self._mag_heading

    @mag_heading.setter
    def mag_heading(self, new_mag_heading):
        self._mag_heading = new_mag_heading

    @property
    def mag_var(self):
        return self._mag_var

    @mag_var.setter
    def mag_var(self, new_mag_var):
        self._mag_var = new_mag_var

    @property
    def num_sats(self):
        return len(self.__visible_prns)

    @num_sats.setter
    def num_sats(self, value):
        assert value <= self.__total_sv_limit
        # Randomly make the requested number visible, make the rest invisible
        # (negative elevation)
        random.shuffle(self.satellites)
        for i in xrange(value):
            self.satellites[i].elevation = random.random() * 90
        for i in xrange(value, len(self.satellites)):
            self.satellites[i].elevation = -90
        self.satellites.sort(key=operator.attrgetter('prn', ))
        self.__recalculate()

    @property
    def output(self):
        return self.__output

    @output.setter
    def output(self, value):
        for item in value:
            assert item in self.__gen_nmea.keys()
        self.__output = value

    @property
    def fix(self):
        return self._fix

    @fix.setter
    def fix(self, value):
        assert value in fix_types
        self._fix = value

    @property
    def solution(self):
        if self.fix == constants.INVALID_FIX:
            return constants.INVALID_SOLUTION
        return self.__solution

    @solution.setter
    def solution(self, value):
        assert (value is None or value in solution_modes)
        self.__solution = value

    def move(self, duration=1.0):
        ''' 'Move' the GNSS instance for the specified duration in seconds based on current heading and velocity.
        '''
        self.__recalculate()
        if self.lat is not None and self.lon is not None and self.heading is not None and self.kph is not None and self.kph > sys.float_info.epsilon:
            speed_ms = self.kph * 1000.0 / 3600.0
            d = speed_ms * duration
            out = Geodesic.WGS84.Direct(self.lat, self.lon, self.heading, d)
            self.lat = out['lat2']
            self.lon = out['lon2']
            self.__recalculate()

    def distance(self, other_lat, other_lon):
        ''' Returns the current distance (in km) between the GNSS instance and an arbitrary lat/lon coordinate.
        '''
        out = Geodesic.WGS84.Inverse(self.lat, self.lon, other_lat, other_lon)
        return out['s12'] / 1000.0

    def get_output(self):
        ''' Returns a list of NMEA sentences (not new line terminated) that the GNSS instance was configured to output.
        '''
        self.__recalculate()
        outputs = []
        for format in self.output:
            outputs += self.__gen_nmea[format]()
        return outputs

    def supported_output(self):
        ''' Returns a tuple of supported NMEA sentences that the GNSS model class is capable of producing.
        '''
        return self.__gen_nmea.keys()


class ModelGpsReceiver(ModelGnssReceiver):
    def __init__(self, *args, **kwargs):
        super(ModelGpsReceiver, self).__init__(min_sv_number=1,
                                               max_sv_number=32,
                                               total_sv_limit=32,
                                               *args, **kwargs)
        self._prefix = 'GP'


class ModelGlonassReceiver(ModelGnssReceiver):
    def __init__(self, *args, **kwargs):
        super(ModelGlonassReceiver, self).__init__(min_sv_number=65,
                                                   max_sv_number=96,
                                                   total_sv_limit=24,
                                                   *args, **kwargs)
        self._prefix = 'GL'


class GpsSim(object):
    ''' GPS simulator class based on a ModelGpsReceiver
    Provides simulated NMEA output based on a ModelGpsReceiver instance over serial and/or stdout.
    Supports satellite model perturbation and random walk heading adjustment.
    '''

    def __init__(self, gps=None, glonass=None, static=False, heading_variation=45):
        ''' Initialise the GPS simulator instance with initial configuration.
        '''
        self.__worker = threading.Thread(target=self.__action)
        self.__run = threading.Event()
        if gps is None:
            gps = ModelGpsReceiver()
        self.gps = gps
        self.glonass = glonass
        self.gnss = [gps]
        if glonass is not None:
            self.gnss.append(glonass)
        self.heading_variation = heading_variation
        self.static = static
        self.interval = 1.0
        self.step = 1.0
        self.lock = threading.Lock()

    def __step(self, duration=1.0):
        ''' Iterate a simulation step for the specified duration in seconds, moving the GPS instance and updating state.
        Should be called while under lock conditions.
        '''
        if self.static:
            return

        for gnss in self.gnss:
            if gnss.date_time is not None:
                gnss.date_time += datetime.timedelta(seconds=duration)

                perturbation = math.sin(gnss.date_time.second * math.pi / 30) / 2
                for satellite in gnss.satellites:
                    satellite.snr += perturbation
                    satellite.elevation += perturbation
                    satellite.azimuth += perturbation

            if self.heading_variation and gnss.heading is not None:
                gnss.heading += (random.random() - 0.5) * \
                                self.heading_variation

            gnss.move(duration)

    def __action(self, conn):
        ''' Worker thread action for the GPS simulator - outputs data to the specified serial port at 1PPS.
        '''
        self.__run.set()
        while self.__run.is_set():
            start = time.time()
            if self.__run.is_set():
                with self.lock:
                    output = []
                    for gnss in self.gnss:
                        output += gnss.get_output()
            if self.__run.is_set():
                for sentence in output:
                    if not self.__run.is_set():
                        break
                    conn.sendall(sentence + "\r\n")

            if self.__run.is_set():
                time.sleep(0.1)  # Minimum sleep to avoid long lock ups
            while self.__run.is_set() and time.time() - start < self.interval:
                time.sleep(0.1)
            if self.__run.is_set():
                with self.lock:
                    if self.step == self.interval:
                        self.__step(time.time() - start)
                    else:
                        self.__step(self.step)


    def serve(self, port, blocking=True):
        ''' Start serving GPS simulator on the specified COM port (and stdout)
            and optionally blocks until an exception (e.g KeyboardInterrupt).
          Port may be None to send to stdout only.
        '''
        # self.kill()
        # self.__worker = threading.Thread(target=self.__action)
        # self.__worker.daemon = True
        # self.__worker.start()
        if blocking:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    s.bind(('', port))
                except socket.error as msg:
                    self.kill()
                s.listen(10)
                while 1:
                    conn, addr = s.accept()
                    start_new_thread(self.__action, (conn,))
                s.close()
                # while True:
                #     self.__worker.join(60)
            except:
                self.kill()

    def kill(self):
        ''' Issue the kill command to the GPS simulator thread and wait for it to die.
        '''
        try:
            while self.__worker.is_alive():
                self.__run.clear()
                self.__worker.join(0.1)
        except KeyboardInterrupt:
            pass

    def is_running(self):
        ''' Is the simulator currently running?
        '''
        return self.__run.is_set() or self.__worker.is_alive()

    def generate(self, duration):
        ''' Instantaneous generator for the GPS simulator - outputs data to stdout synchronously.
        '''
        with self.lock:
            start = self.gps.date_time
        now = start
        while (now - start).total_seconds() < duration:
            with self.lock:
                output = []
                for gnss in self.gnss:
                    output += gnss.get_output()
                # for sentence in output:
                #     print sentence
                self.__step(self.step)
                now = self.gps.date_time

    def output_latest(self):
        '''Ouput the latest fix to a specified COM port.
        '''


if __name__ == '__main__':
    sim = GpsSim()

    # How to output specific sentence types from the model
    model = ModelGpsReceiver()
    model.output = ('GGA', 'RMC')
    sentences = model.get_output()

    # Modify settings under lock protection
    with sim.lock:
        sim.gps.output = ('GGA', 'GLL', 'GSA', 'GSV',
                          'RMC', 'VTG', 'ZDA')  # can re-order or drop some
        sim.gps.num_sats = 14
        sim.gps.lat = 1
        sim.gps.lon = 3
        sim.gps.altitude = -13
        sim.gps.geoid_sep = -45.3
        sim.gps.mag_var = -1.1
        sim.gps.kph = 60.0
        sim.gps.heading = 90.0
        sim.gps.mag_heading = 90.1
        sim.gps.date_time = datetime.datetime.now(
            TimeZone(time.timezone))  # PC current time, local time zone
        sim.gps.hdop = 3.1
        sim.gps.vdop = 5.0
        sim.gps.pdop = (sim.gps.hdop ** 2 + sim.gps.vdop ** 2) ** 0.5

        # Precision decimal points for various measurements
        sim.gps.horizontal_dp = 4
        sim.gps.vertical_dp = 1
        sim.gps.speed_dp = 1
        sim.gps.time_dp = 2
        sim.gps.angle_dp = 1

        # Keep straight course for simulator - don't randomly change the
        # heading
        sim.heading_variation = None

    # How to synchronously generate simulated data to stdout
    sim.generate(1)

    port = 8888
    if len(sys.argv) > 1:
        if sys.argv[1] == '--help' or sys.argv[1] == '-h':
            print "Usage: %s [serial port]" % sys.argv[0]
            sys.exit(0)
        port = sys.argv[1]
    sim.serve(port)
