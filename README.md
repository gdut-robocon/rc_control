<a id="readme-top"></a>

<!-- LANGUAGE SWITCH -->

---

<!-- PROJECT LOGO -->

<br />
<div align="center">

<h3 align="center">rc_control</h3>

<p align="center">
    This project provides a comprehensive robotics control framework for real-time sensor processing, hardware abstraction, and simulation integration. Built on ROS, it supports IMU filtering, CAN bus communication, GPIO control, and RC input handling. Key features include sensor fusion, hardware interfacing for actuators and laser sensors, Gazebo simulation plugins, and real-time control loops. Designed for modular deployment in robotic systems, it enables precise control, state estimation, and simulation for applications like mobile robots or robotic arms.
    <br />
    <a href="https://github.com/gdut-robocon/rc_control"><strong>Explore the docs »</strong></a>
    <br />
  </p>

<!-- PROJECT SHIELDS -->

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

<p align="center">
    <a href="https://github.com/gdut-robocon/rc_control">View Demo</a>
    ·
    <a href="https://github.com/gdut-robocon/rc_control/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/gdut-robocon/rc_control/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
  </p>
</div>

<!-- TABLE OF CONTENTS -->

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->

## 📖 About The Project

This project provides a comprehensive robotics control framework for real-time sensor processing, hardware abstraction, and simulation integration. Built on ROS (Robot Operating System), it supports IMU filtering, CAN bus communication, GPIO control, and RC input handling. Key features include sensor fusion, hardware interfacing for actuators and laser sensors, Gazebo simulation plugins, and real-time control loops. Designed for modular deployment in robotic systems, it enables precise control, state estimation, and simulation for applications like mobile robots or robotic arms.

### Key Features

- **Robot Control Framework**: Modular architecture for integration with various robotic platforms.
- **Sensor Fusion Filters**: Implements complementary filters, Kalman filters, and adaptive filtering for IMU data.
- **Real-time Hardware Interface**: Supports CAN bus communication, GPIO control, and serial device integration.
- **ROS Integration**: Full compatibility with ROS ecosystem including message types, launch files, and control interfaces.
- **Gazebo Simulation Support**: Custom plugins for simulating hardware components in Gazebo.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![ROS][ROS-shield]][ROS-url]
* [![Gazebo][Gazebo-shield]][Gazebo-url]
* [![C++][C++-shield]][C++-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### 📁 Project Structure

<details>
<summary>Click to expand project structure</summary>

```
rc_control/
├── .clang-format
├── .clang-tidy
├── .pre-commit-config.yaml
├── rc_common/
├── rc_gazebo/
├── rc_hw/
├── rc_ibus/
├── rc_msgs/
```

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->

## 🚀 Getting Started

This is an example of how you may give instructions on setting up your project locally. To get a local copy up and running follow these simple steps.

### Prerequisites

- ROS (Noetic or later)
- Gazebo
- C++14 or higher
- CMake
- Git

### Installation

1. Clone the repo
   
   ```sh
   git clone https://github.com/gdut-robocon/rc_control.git
   ```

2. Build with Catkin
   
   ```sh
   cd rc_control
   catkin_make
   ```

3. Source the workspace
   
   ```sh
   source devel/setup.bash
   ```

### Configuration

- Modify hardware configuration in `rc_hw/config/hw_config_template.yaml`
- Adjust IMU filter parameters in `rc_common/include/rc_common/filters/`
- Update CAN bus settings in `rc_hw/src/hardware_interface/can_bus.cpp`

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->

## 💻 Usage

To launch the hardware interface:

```sh
roslaunch rc_hw rc_hw.launch
```

To run the Gazebo simulation:

```sh
roslaunch rc_gazebo field_rc23.launch
```

To interface with RC input:

```sh
roslaunch rc_ibus rc_ibus.launch
```

For more detailed usage, refer to the documentation linked in the introduction.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->

## 🗺️ Roadmap

- **v1.0** - Initial release with core framework and hardware interface
- **v1.1** - Added Gazebo simulation plugins and IMU filtering
- **v1.2** - Introduced CAN bus support and GPIO control
- **v2.0** - Enhanced real-time capabilities and added advanced filtering techniques
- **v2.1** - Improved documentation and modular structure for easier integration

See the [open issues](https://github.com/gdut-robocon/rc_control/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->

## 🤝 Contributing

Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->

## 🎗 License

Copyright © 2024-2025 [rc_control][rc_control]. <br />
Released under the [MIT][license-url] license.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->

## 📧 Contact

Email: jialonglongliu@gmail.com

Project Link: [https://github.com/gdut-robocon/rc_control](https://github.com/gdut-robocon/rc_control)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## REFERENCE LINKS 

[rc_control]: https://github.com/gdut-robocon/rc_control
[rm_control](https://github.com/rm-controls/rm_control)
<!-- MARKDOWN LINKS & IMAGES -->

[contributors-shield]: https://img.shields.io/github/contributors/gdut-robocon/rc_control.svg?style=flat-round
[contributors-url]: https://github.com/gdut-robocon/rc_control/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/gdut-robocon/rc_control.svg?style=flat-round
[forks-url]: https://github.com/gdut-robocon/rc_control/network/members
[stars-shield]: https://img.shields.io/github/stars/gdut-robocon/rc_control.svg?style=flat-round
[stars-url]: https://github.com/gdut-robocon/rc_control/stargazers
[issues-shield]: https://img.shields.io/github/issues/gdut-robocon/rc_control.svg?style=flat-round
[issues-url]: https://github.com/gdut-robocon/rc_control/issues
[license-shield]: https://img.shields.io/github/license/gdut-robocon/rc_control.svg?style=flat-round
[license-url]: https://github.com/gdut-robocon/rc_control/blob/master/LICENSE.txt

<!-- Tech Stack -->

[ROS-shield]: https://img.shields.io/badge/ROS-2960CA?style=flat-round&logo=ros&logoColor=white
[ROS-url]: https://www.ros.org/
[Gazebo-shield]: https://img.shields.io/badge/Gazebo-000000?style=flat-round&logo=gazebo&logoColor=white
[Gazebo-url]: https://gazebosim.org/
[C++-shield]: https://img.shields.io/badge/C++-00599C?style=flat-round&logo=cplusplus&logoColor=white
[C++-url]: https://isocpp.org/
<p align="right">(<a href="#readme-top">back to top</a>)</p>