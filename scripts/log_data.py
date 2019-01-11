import cflib.crtp
from cflib.crazyflie import Crazyflie as CF
from cflib.crazyflie.log import LogConfig

class LogData:
	def __init__(self, radio_uri):

		cflib.crtp.init_drivers(enable_debug_driver=False)
		# try:
		# with SyncCrazyflie(self._uri) as scf:
		self.cf_active = False


		self.cf = CF(rw_cache="./cache")
		self.cf.connected.add_callback(self.connected)
		self.cf.disconnected.add_callback(self.disconnected)
		self.cf.connection_failed.add_callback(self.connection_lost)
		self.cf.connection_lost.add_callback(self.connection_failed)
		
		print('Connecting to %s' % radio_uri)
		self.cf.open_link(radio_uri)

	## CALLBACKS ##
	def connected(self, uri):
		print("Connected to Crazyflie at URI: %s" % uri)

		self.cf_active = True

		try:
			self.log_data_acc = LogConfig(name="AccData", period_in_ms=10)
			self.log_data_acc.add_variable('acc.x', 'float')
			self.log_data_acc.add_variable('acc.y', 'float')
			self.log_data_acc.add_variable('acc.z', 'float')


			self.log_data_other = LogConfig(name="OtherData", period_in_ms=10)
			self.log_data_other.add_variable('pm.vbat', 'float')
			self.log_data_other.add_variable('stateEstimate.z', 'float')


			self.log_data_kalman = LogConfig(name="KalmanData", period_in_ms=10)
			self.log_data_kalman.add_variable('kalman_states.vx', 'float')
			self.log_data_kalman.add_variable('kalman_states.vy', 'float')


			self.log_data_mag = LogConfig(name="MagData", period_in_ms=10)
			self.log_data_mag.add_variable('mag.x', 'float')
			self.log_data_mag.add_variable('mag.y', 'float')
			self.log_data_mag.add_variable('mag.z', 'float')


			self.cf.log.add_config(self.log_data_acc)
			self.cf.log.add_config(self.log_data_other)
			self.cf.log.add_config(self.log_data_kalman)
			self.cf.log.add_config(self.log_data_mag)

			self.log_data_acc.data_received_cb.add_callback(self.received_data)
			self.log_data_other.data_received_cb.add_callback(self.received_data)
			self.log_data_kalman.data_received_cb.add_callback(self.received_data)
			self.log_data_mag.data_received_cb.add_callback(self.received_data)

			#begins logging and publishing
			self.log_data_acc.start()
			self.log_data_other.start()
			self.log_data_kalman.start()
			self.log_data_mag.start()

			print("Logging Setup Complete. Starting...")
		except KeyError as e:
			print('Could not start log configuration,'
				  '{} not found in TOC'.format(str(e)))
		except AttributeError:
			print('Could not add log config, bad configuration.')


	def disconnected(self, uri):
		self.cf_active = False
		print("Disconnected from Crazyflie at URI: %s" % uri)

	def connection_failed(self, uri, msg):
		self.cf_active = False
		print("Connection Failed")

	def connection_lost(self, uri, msg):
		self.cf_active = False
		print("Connection Lost")



	def received_data(self, timestamp, data, logconf):
		
		if logconf.name == 'AccData':
			pass
		elif logconf.name == 'OtherData':
			pass
		elif logconf.name == 'KalmanData':
			pass
		elif logconf.name == 'MagData':
			pass

	def loop(self):
		while True:
			pass

if __name__ == '__main__':
	logger = LogData('radio://0/80/250K')
	logger.loop()