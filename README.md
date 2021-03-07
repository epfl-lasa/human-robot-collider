# human-robot-collider
This software package simulates and analyses collisions between a walking pedestrian model and a mobile robot to investigate the risk associated to such human-robot collisions and to support safety-guided robot design. The software lets both agents approach each other and collide with different angles and speeds, using the simulation engine Bullet (via `pybullet`).

## Prerequisites
It is recommended to use a virtual environment to setup the simulation. Process to setup the environment and install all dependencies is given below.

```shell
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Executing simulation
The simulation can be excuted by the following command,

```
python path/to/human-robot-collider/src/hrc.py
```

| Argument              | Options                                 | Help                                                  |
| ---                   | ---                                     | ---                                                   |
|`-h`, `--help`         |                                         | Show the help message and exit                        |
|`-b`, `--human`        | `man`, `child`                          | Human to collide with the robot (default = `man`)     |
|`-r`, `--robot`        | `qolo`, `wheelchair`, `pepper`          | Robot to collide with the human (default = `qolo`)    |
|`-c`, `--controller`   | `no_control`, `admittance`, `passive_ds`| Adaptive controller to use (default = `no_control`)   |
|`-g`, `--gui`          |                                         | Set to show GUI                                       |

If `--gui` argument is activated, plot of *collision force* are drawn after each collision.

At the end of all simulations, `collision_forces` array for all collisions are saven in `controlled_collision.npy` for future use.

### Examples
- To run simulation of collision between a man and qolo using admittance controller (with gui),
```shell
python src/hrc.py -b adult -r qolo -c admittance -g
```

- To run simulation of collision between a child and qolo using admittance controller (with gui),
```shell
python src/hrc.py -b child -r qolo -c admittance -g
```

- To run simulation of collision between a child and qolo using no compliance controller (with gui),
```shell
python src/hrc.py -b child -r qolo -c no_control -g
```
