# RaceTrackAI-Probabilistic-Positioning

### Clone the Repository

First, clone the repository to your local machine using the following command:

```bash
git clone https://github.com/AngelaZhang915/RaceTrackAI-Probabilistic-Positioning.git
cd RaceTrackAI-Probabilistic-Positioning
```

### Install Dependencies

Install the required dependencies using:

```bash
pip install -r requirements.txt
```

### Run the Project

To run the graphical user interface (GUI) with specified parameters for particle filtering and Kalman filtering, use the following command:

```bash
python3 gui.py --num_particles 100 --max_sensor_range 100 --sensor_noise_std 2.0
```

To generate plots for particle filtering and Kalman filtering using a pre-recorded drive around the track, run:

```bash
python3 plots.py -w pf --num_particles 100 --max_sensor_range 100 --sensor_noise_std 2.0 --filename pf.png
```

#### Additional Information

- Run `python3 plots.py -h` to get details on how to specify parameters for the plotting script.
- Run `python3 gui.py -h` to view parameter options for the GUI-based simulation.
- Make sure you have implemented the necessary code for particle filtering and Kalman filtering before running these commands.

#### Parameter Descriptions:
- `--num_particles`: Number of particles used in the particle filter (default: 100)
- `--max_sensor_range`: Maximum range of sensors in the simulation (default: 100)
- `--sensor_noise_std`: Standard deviation of sensor noise (default: 2.0)

Feel free to adjust the parameters to explore different configurations.
