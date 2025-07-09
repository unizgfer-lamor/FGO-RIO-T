# Radar-Inertial Odometry with Online Temporal Offset Calibration (FGO-RIO-T) 

**FGO-RIO-T** is an open-source implementation of a radar-inertial odometry system that integrates online temporal delay calibration within a factor graph optimization framework. This repository accompanies the paper:
> 📄 **Impact of Temporal Delay on Radar-Inertial Odometry**  
> Vlaho-Josip Štironja, Luka Petrović, Juraj Peršić, Ivan Marković, Ivan Petrović  
> [[arXiv:2503.02509](https://arxiv.org/abs/2503.02509)]

---

## News / Events
- **June 16, 2025** - The paper is accepted to IROS 2025.
- **March 04, 2025** - The manuscript has been online at the [arXiv](https://arxiv.org/pdf/2503.02509).
- **March 01, 2025** - The paper has been submitted to [IROS 2025]([https://www.iros25.org/).





## 📄 Paper Summary

In this work, we introduce a novel Radar-Inertial Odometry (RIO) system that fuses data from an automotive radar and an inertial measurement unit (IMU). The key contribution is the integration of online temporal delay calibration within the factor graph optimization framework, compensating for potential time offsets between radar and IMU measurements. Our experimental analysis demonstrates that, even without scan matching or target tracking, the integration of online temporal calibration significantly reduces localization error compared to systems that disregard time synchronization. This highlights the crucial role of accurate temporal alignment in radar-based sensor fusion systems for autonomous navigation.

---

## 📁 Project Structure

```
FGO-RIO-T/
├── Dockerfile                # Docker environment specification
├── README.md                 # Project documentation
└── fgo_rio_t/                # ROS package with the implementation
└── IRS/                      # IRS test examples and scripts for running the evaluation
```

---

## 🚀 Quickstart with Docker

To facilitate easy setup and reproducibility, this repository is Dockerized. Follow the steps below to get started:

### 1. Clone the Repository

```bash
git clone https://github.com/unizgfer-lamor/FGO-RIO-T.git
cd FGO-RIO-T
```

### 2. Build the Docker Image

```bash
docker build -t rio-t .
```

### 3. Run the Docker Container

```bash
docker run -it --rm --net=host \
    --env="DISPLAY" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged rio-t
```

### 4. Inside Docker: Build and Launch

```bash
./run_rio_t.sh
```

This script will automatically run the radar-inertial odometry pipeline on all provided IRS sequences, and store the odometry and time offset estimation results.


---

## 📚 Citation

If you use this work in your research, please consider citing our paper:

```bibtex
@article{stironja2025impact,
  title={Impact of Temporal Delay on Radar-Inertial Odometry},
  author={Štironja, Vlaho-Josip and Petrović, Luka and Peršić, Juraj and Marković, Ivan and Petrović, Ivan},
  journal={arXiv preprint arXiv:2503.02509},
  year={2025}
}
```

---



## Note
This repository contains an updated implementation of the method described in the paper "Impact of Temporal Delay on Radar-Inertial Odometry". While the results presented in the paper were generated using an earlier version of the code, this repository includes several improvements and refinements. When tested on the IRS dataset, these updates lead to slightly improved results compared to those reported in the original paper.

 ---


## ⭐ Star this Repo!

If you find this project useful, please consider giving it a star ⭐. Your support helps increase the visibility of our work and encourages future development.

---

## 📢 Acknowledgments

This work builds upon the implementation of the [RIO](https://github.com/ethz-asl/rio) package by ETH Zurich's Autonomous Systems Lab. Specific functions and modules were adapted
to support our implementation. We gratefully acknowledge their contribution to the open-source robotics community. 

This research has been supported by the European Regional Development Fund under grant agreement PK.1.1.10.0007 (DATACROSS) and the Croatian Science Foundation under grant agreement DOK-NPOO-2023-10-6705.

