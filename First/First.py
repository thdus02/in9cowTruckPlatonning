# Step 1: Add modulesto provide access to specific libraries and functions
import os # Module provides functions to handle file paths, directories, environment variables
import sys # Module provides access to Python-specific system parameters and functions

# Step 2: Establish path to SUMO (SUMO_HOME) -> 환경변수 편집해놓은거
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 3: Add Traci module to provide access to specific libraries and functions
import traci # Static network information (sucn as reading and analyzing network files)

# Step 4: Define Sumo configuration
Sumo_config = [
    'sumo-gui',
    '-c', 'First.sumocfg',
    '--step-length', '0.05',
    '--delay', '1000',
    '--lateral-resolution', '0.1'
]

# Step 5: Open connection between SUMO and Traci
traci.start(Sumo_config)

# Step 6: Define Variables
vehicle_speed = 0
total_speed = 0

# Step 7: Define Functions

# Step 8: Take simulation steps until there are no more vehicles in the network
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep() # Move simulation forward 1 step
    # Here you can decide what to do with simulation data at each step
    # print(traci.vehicle.getIDList())
    if 'Leader' in traci.vehicle.getIDList(): # 아씨.. 차량이 flow인지 trip인지 확인.. trip이어야 아이디 가져오는듯.
        vehicle_speed = traci.vehicle.getSpeed('Leader')
        total_speed = total_speed + vehicle_speed
    # step_count = step_count + 1
    print(f"Vehicle speed: {vehicle_speed} m/s")

# Step 9: Close connection between SUMO and Traci
traci.close()

