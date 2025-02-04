import qi
import sys 
import time

class Authenticator:

    def __init__(self, username, password):
        self.username = username
        self.password = password

    # This method is expected by libqi and must return a dictionary containing
    # login information with the keys 'user' and 'token'.
    def initialAuthData(self):
        return {'user': self.username, 'token': self.password}


class AuthenticatorFactory:

    def __init__(self, username, password):
        self.username = username
        self.password = password

    # This method is expected by libqi and must return an object with at least
    # the `initialAuthData` method.
    def newAuthenticator(self):
        return Authenticator(self.username, self.password)

# Connect to the robot fails at app.start() => RuntimeError: disconnected
app = qi.Application(sys.argv, url="tcps://10.186.13.37:9503")
logins = ("nao", "venere")
factory = AuthenticatorFactory(*logins)
app.session.setClientAuthenticatorFactory(factory)
app.start()
print("started")

# This doesn't work either => RuntimeError: disconnected
# session = qi.Session()
# logins = ("nao", "OMITTED")
# factory = AuthenticatorFactory(*logins)
# session.setClientAuthenticatorFactory(factory)
# session.connect("tcp://192.168.1.59:9503")

tts = app.session.service("ALTextToSpeech")
tts.say("Hello there!")
motion = app.session.service("ALMotion")
x     = 0.0
y     = 0.0
theta = 1.0
frequency = 1.0
motion.moveToward(x, y, theta)
time.sleep(3)
motion.stopMove()
