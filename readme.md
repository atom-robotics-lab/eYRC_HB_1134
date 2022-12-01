<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/atom-robotics-lab/assets/blob/main/logo_1.png?raw=true">
    <img src="https://github.com/atom-robotics-lab/assets/blob/main/logo_1.png?raw=true" alt="Logo" width="120" height="120">
  </a>

<h3 align="center">Atom eYRC_HB_1134</h3>

  <p align="center">
    This is the repo for the <a href="https://portal.e-yantra.org/">eYRC competition</a> submisison by the team hb_1134, a documentation for the our Project. The Project's purpose was to simulate Holonomic drive, by making a 3 wheel omi-directional bot. The simulation was achieved with the help of Gazebo and ROS.
    If you’re interested in helping us improve our Project find out how to <a href="https://github.com/atom-robotics-lab/eYRC_HB_1134/blob/main/contributing.md">contribute<a>. 
    <br />
    <br />
    <a href="https://github.com/atom-robotics-lab/eYRC_HB_1134/issues/new?labels=bug&assignees=Kartik9250,MGupta28,krrish-jindal">Report Bug</a>
    ·
    <a href="https://github.com/atom-robotics-lab/eYRC_HB_1134/issues/new?labels=enhancement&assignees=Kartik9250,MGupta28,krrish-jindal">Request Feature</a>
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
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

Cities, by nature, are constantly under development. In the future, the process of achieving the ever-demanding upgrades in the infrastructure of a city will be automated. The impact of such automation can lead to lesser construction time, little to no manual labour and lower execution costs. With more time, energy and finance to spare, this automation will be empowered by technology and imagination of the mind, and leave room for artistic expression.
Humans have an inherent need to make sense of the world around them, understand it and then create something new out of it. For a more inclusive city, this is even more true. Demarcations on the road, signage on billboards and bus stops, and well-landscaped gardens provide order to the chaos and create an environment where the inhabitants can connect to their primal need to belong.

Keeping the above scenario in mind, in eYRC 2022-23 we were presented with the theme HolA Bot (HB)! HolA Bot is short for Holonomic Art Bot. As the full name suggests, this theme contained two major components to it: Holonomic Drive Bot and Art!

In this theme, our team had to deploy holonomic robot in Simulated Environment. Unlike the usual, more popular differential drive robots, the holonomic drive robots can control all the three degrees of freedom possible on a plane (translation along the x, y-axis and rotation along the z-axis). This gives the robot the ability to make art that would otherwise not be possible with the usual two-wheeled differential drive robot. This ability is demonstrated by the following video/gif of KUKA omniMove:
<br>
<center><image src="https://i.ibb.co/Fwskw4y/ezgif-com-gif-maker.gif" height="350" width="600"></center>

<p align="right">(<a href="#readme-top">back to top</a>)</p>



### Built With

* [![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)](https://www.sphinx-docs.org)
* [![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org/)
* [![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)
* [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may set-up your our project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.

- ROS
- OpenCV

### Installation

1. Clone the repo in your ROS-workspace
    ```sh
    git clone git@github.com:atom-robotics-lab/eYRC_HB_1134.git
    cd ~/catkin_ws/src
    ```
2. Compile the package
    ```sh
    cd ~/catkin_ws
    catkin_make
    ``` 
3. Source the setup script
    ```sh
    source ~/catkin_ws/devel/setup.bash
    ```
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additionally you may check the resources added below.

- For running the Task02
    ```sh
    roslaunch Task02 gazebo.launch
    roslaunch Task02 controller.py
    ```

- For running the Task02
    ```sh
    roslaunch Task02 gazebo.launch
    roslaunch Task02 feedback.py
    roslaunch Task02 controller.py
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

- [x] task01
- [x] task02
- [x] hardware implementation
    - [x] testing individual parts and components
    - [x] making bot chassis
    - [ ] printing holonomic wheels
    - [ ] fabricating power delivery, daughterboard pcb
    - [ ] final testing

See the [open issues](https://github.com/atom-robotics-lab/eYRC_HB_1134/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

For more info refer to [contributing.md](https://github.com/atom-robotics-lab/eYRC_HB_1134/blob/main/contributing.md)
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Our Socials - [Linktree](https://linktr.ee/atomlabs) - atom@inventati.org

eYRC_HB_1134 Link: [https://github.com/atom-robotics-lab/eYRC_HB_1134](https://github.com/atom-robotics-lab/eYRC_HB_1134)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS-->
## Acknowledgments

* [ROS Official Documentation](http://wiki.ros.org/Documentation)
* [Opencv Official Documentation](https://docs.opencv.org/4.x/)
* [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)
* [Ubuntu Installation guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) 

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/atom-robotics-lab/eYRC_HB_1134.svg?style=for-the-badge
[contributors-url]: https://github.com/atom-robotics-lab/eYRC_HB_1134/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/atom-robotics-lab/eYRC_HB_1134.svg?style=for-the-badge
[forks-url]: https://github.com/atom-robotics-lab/eYRC_HB_1134/network/members
[stars-shield]: https://img.shields.io/github/stars/atom-robotics-lab/eYRC_HB_1134.svg?style=for-the-badge
[stars-url]: https://github.com/atom-robotics-lab/eYRC_HB_1134/stargazers
[issues-shield]: https://img.shields.io/github/issues/atom-robotics-lab/eYRC_HB_1134.svg?style=for-the-badge
[issues-url]: https://github.com/atom-robotics-lab/eYRC_HB_1134/issues
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/company/a-t-o-m-robotics-lab/