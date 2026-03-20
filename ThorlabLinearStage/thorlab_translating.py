from pylablib.devices import Thorlabs

SCALE = 409600  # counts per mm for LTS300
MAX_TRAVEL = 250  # mm, LTS300 max travel


stage = Thorlabs.KinesisMotor("/dev/ttyUSB1")
print("Connected!")

# Self-calibrating home
print("Homing...")
stage.home()
stage.wait_move()
HOME_RAW = 0#stage.get_position()
print(f"Home calibrated at raw position: {HOME_RAW}")

def move_to_mm(stage, mm):
    if mm < 0 or mm > MAX_TRAVEL:
        print(f"ERROR: {mm} mm is out of range (0-{MAX_TRAVEL} mm), move aborted!")
        return
    stage.move_to(int(HOME_RAW + mm * SCALE))
    stage.wait_move()
    print(f"Moved to {(HOME_RAW + stage.get_position()) / SCALE:.3f} mm")

def get_position_mm(stage):
    return (stage.get_position() - HOME_RAW) / SCALE

def go_home(stage):
    stage.move_to(HOME_RAW)
    stage.wait_move()
    print(f"Back at home: {get_position_mm(stage):.3f} mm")

# --- Example usage ---
print(f"Position: {get_position_mm(stage):.3f} mm")

target_position = int(input("Insert desired position in mm:\n "))

move_to_mm(stage, target_position)
#move_to_mm(stage, 10)
#move_to_mm(stage, 20)
#move_to_mm(stage, 100)

#go_home(stage)

stage.close()
