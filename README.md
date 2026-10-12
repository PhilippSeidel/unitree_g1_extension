
### Install
1. Source your incar venv. Install [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) first, as described there:
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```
Then install this extension by:
```bash
git clone https://github.com/PhilippSeidel/unitree_g1_extension.git
cd unitree_g1_extension
pip install .
```
2. Add a `g1_zmq` (real robot) or `g1_sim` (simulation) camera to the `cameras` in the `workspace_config.json`:
```json
"cameras": {
    "ego_view": {
        "type": "g1_zmq",
        "camera_name": "g1_head_camera_sim"
    },
    "head_camera": {
        "type": "g1_sim",
        "camera_name": "g1_head_camera_sim"
    }
}
```
### Start
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

