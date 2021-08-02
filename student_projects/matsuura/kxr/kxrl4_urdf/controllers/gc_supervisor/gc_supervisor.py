from controller import Supervisor, Robot, Node, Display

supervisor = Supervisor()

emitter = supervisor.getDevice('emitter')
# print emitter

## collect garbedges
garbedges = []
rt = supervisor.getRoot()
fd = rt.getField('children')
for i in range(fd.getCount()):
    nd = fd.getMFNode(i)
    f = nd.getField('name')
    if f:
        #f.getTypeName()
        #f.getCount()
        nm = f.getSFString()
        if nm[0] == 'G': ## Solid whose name start with G as garbedge
            garbedges.append(nd)

def in_trash_box(g):
    pos = g.getPosition()
    if pos[0] > -1 and pos[0] < 1 and pos[2] > 3.3:
        return True
    return False

initial_garbedge_num = len(garbedges)
print ('{} garbeges found'.format(initial_garbedge_num))
supervisor.step(100)

while supervisor.step(400) != -1:
    for g in garbedges:
        if in_trash_box(g):
            print( '{} is collected'.format(g))
            garbedges.remove(g)
            g.remove()
            break

    print( '{} garbedges are collected'.format(initial_garbedge_num - len(garbedges)))

    if emitter:
        pass
        #emitter.send('S1 position: {}'.format(srobot.getPosition()))
