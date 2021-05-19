from controller import Supervisor, Robot, Node, Display

supervisor = Supervisor()

emitter = supervisor.getDevice('emitter')
print emitter

srobot = supervisor.getFromDef('S1')
print srobot

supervisor.step(1000)

while supervisor.step(100) != -1:
    if srobot:
        print srobot.getPosition()
        print srobot.getOrientation()
        print
    if emitter:
        emitter.send('S1 position: {}'.format(srobot.getPosition()))
