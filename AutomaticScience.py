import krpc
import time
from BasicSetup import setup


def automaticScience(conn, vessel):
    remote_tech_available = conn.remote_tech.available

    def doScience():
        if remote_tech_available and conn.remote_tech.comms(vessel).has_connection_to_ground_station:
            transmit_possible = True
        elif vessel.comms.can_transmit_science:
            transmit_possible = True
        else:
            transmit_possible = False
        for part in vessel.parts.with_module('ModuleScienceExperiment'):
            if part.experiment.has_data:
                for data in part.experiment.data:
                    print("Handling", part.title)
                    if data.transmit_value > 0.5:
                        if transmit_possible:
                            print("Transmitting", part.title, "for", data.transmit_value)
                            part.experiment.transmit()
                        else:
                            print("No transmitting possible")
                    elif data.science_value < 0.5:
                        print("Dumping", part.title, "for", data.science_value)
                        part.experiment.dump()
            if part.experiment.has_data == False and part.experiment.inoperable == False:
                print("Running", part.title)
                part.experiment.run()

    def _biomeCallback(newBiome):
        print("Entered new biome -", newBiome, "- doing science!")
        doScience()
    def _situationCallback(newSituation):
        print("Changed circumstances -", newSituation, "- doing science!")
        doScience()
    biome = conn.add_stream(getattr, vessel, 'biome')
    situation = conn.add_stream(getattr, vessel, 'situation')
    biome.start()
    situation.start()
    biome.add_callback(_biomeCallback)
    situation.add_callback(_situationCallback)


if __name__ == "__main__":
    ConVes = setup()
    automaticScience(ConVes[0], ConVes[1])
    while True:
        continue
