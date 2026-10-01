# Python Tools

The Python code provides a minimum-snap trajectory generation server for the baseline PD and MPC controllers in the Unity project. The server generates trajectories from waypoint requests and serves them at `http://127.0.0.1:5000/generate`. The implementation is based on the paper ["Minimum snap trajectory generation and control for quadrotors" by D. Mellinger and V. Kumar](https://doi.org/10.1109/ICRA.2011.5980409).

## Installation

Make sure, Python 3.10.12 is installed. From the repository root, create a virtual environment and install the dependencies:

```bash
cd Python
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Usage

With the virtual environment activated, start the trajectory server from inside the Python folder:

```bash
python -m scripts.trajectory_generation_server
```

The Unity client connects to the server on port `5000`.

## ML-Agents training configuration

The ML-Agents training configuration is available in `config/Tello.yaml`.
