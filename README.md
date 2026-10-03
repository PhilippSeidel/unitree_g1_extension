This interface uses the *right.commands.arm.ee.velocity* command hook from Incar to teleoperate a Unitree G1 robots right arm end effector. 

### Dependencies
Lerobot for G1: https://huggingface.co/docs/lerobot/en/unitree_g1

Incar: https://incar-robotics.github.io/

### Starting
#### Incar side
1. Source the venv
2. Start the incar app as usual. Start the G1 interface with:
```bash
python -m robot_interface.run --robot sim #MuJoCo simulation environment
python -m robot_interface.run --robot real #Real robot
```
#### Robot side
Start the Lerobot G1 server:
```bash
python ~/lerobot/src/lerobot/robots/unitree_g1/run_g1_server.py
```