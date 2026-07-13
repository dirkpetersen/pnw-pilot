import cereal.messaging as messaging
from cereal import car

msg = messaging.new_message('carParams')
msg.carParams.fingerprintSource = car.CarParams.FingerprintSource.fixed
print(repr(msg.carParams.fingerprintSource))
print(msg.carParams.fingerprintSource == car.CarParams.FingerprintSource.fixed)
print(msg.carParams.fingerprintSource == 'fixed')
