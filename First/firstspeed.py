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

    if 'Leader' in traci.vehicle.getIDList() and 'Follower' in traci.vehicle.getIDList():
        # 위치
        leader_pos = traci.vehicle.getPosition('Leader')[0]
        follower_pos = traci.vehicle.getPosition('Follower')[0]

        # 간격 계산
        distance = leader_pos - follower_pos

        # 속도
        leader_speed = traci.vehicle.getSpeed('Leader')
        follower_speed = traci.vehicle.getSpeed('Follower')

        # 초기 step에서 속도 맞춤
        if traci.simulation.getTime() == 0.0:
            traci.vehicle.setSpeed('Follower', leader_speed)

        print(f"Time: {traci.simulation.getTime():.2f}s | Distance: {distance:.2f} m | Leader: {leader_speed:.2f} m/s | Follower: {follower_speed:.2f} m/s")
        
# Step 9: Close connection between SUMO and Traci
traci.close()