import krpc

def setup(connName = 'basicSetup'):
    conn = krpc.connect(name=connName)
    vessel = conn.space_center.active_vessel
    return [conn, vessel]
