from nexstar import NexStar

PORT = '/dev/ttyUSB.cg5'

nex = NexStar(PORT, False)
nex.sync_home_position()
nex.close()
